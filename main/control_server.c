#include "control_server.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "app_config.h"
#include "control_protocol.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gimbal_control.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"

static const char *TAG = "control";

/* Long enough to ride out a Wi-Fi hiccup, short enough that a viewer which
 * died without closing its socket cannot hold the single client slot. The
 * motion watchdog in gimbal_control.c stops the axes long before this. */
#define CONTROL_RECEIVE_TIMEOUT_MS 2000
#define CONTROL_IDLE_DROP_MS 10000

static bool receive_exactly(int socket_fd, void *destination, size_t bytes,
                            bool *timed_out)
{
    uint8_t *next_byte = destination;
    size_t bytes_left = bytes;

    *timed_out = false;
    while (bytes_left > 0) {
        const int received = recv(socket_fd, next_byte, bytes_left, 0);
        if (received == 0) {
            return false; /* orderly close by the desktop */
        }
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* A clean timeout between commands is survivable. One part-way
                 * through a command is not: this used to keep waiting for as
                 * long as TCP keepalive took to notice a vanished desktop --
                 * two hours with lwIP's defaults -- holding the only client
                 * slot and never reaching the idle check. Drop the client. */
                *timed_out = bytes_left == bytes;
                if (!*timed_out) {
                    ESP_LOGW(TAG, "Command stalled part-way through; dropping the client");
                }
                return false;
            }
            return false;
        }
        next_byte += received;
        bytes_left -= (size_t)received;
    }
    return true;
}

static bool send_all(int socket_fd, const void *data, size_t bytes)
{
    const uint8_t *next_byte = data;
    size_t bytes_left = bytes;

    while (bytes_left > 0) {
        const int sent = send(socket_fd, next_byte, bytes_left, 0);
        if (sent <= 0) {
            return false;
        }
        next_byte += sent;
        bytes_left -= (size_t)sent;
    }
    return true;
}

static bool decode_command(const gimbal_command_t *wire, gimbal_command_t *host)
{
    host->magic = ntohl(wire->magic);
    host->version = ntohs(wire->version);
    host->message_bytes = ntohs(wire->message_bytes);
    host->sequence = ntohl(wire->sequence);
    host->pan = (int16_t)ntohs((uint16_t)wire->pan);
    host->tilt = (int16_t)ntohs((uint16_t)wire->tilt);
    host->flags = ntohs(wire->flags);
    host->captured_at_ms = ntohs(wire->captured_at_ms);

    return host->magic == GIMBAL_COMMAND_MAGIC &&
           host->version == GIMBAL_PROTOCOL_VERSION &&
           host->message_bytes == sizeof(gimbal_command_t);
}

static void encode_status(const gimbal_status_t *host, gimbal_status_t *wire)
{
    wire->magic = htonl(host->magic);
    wire->version = htons(host->version);
    wire->message_bytes = htons(host->message_bytes);
    wire->sequence = htonl(host->sequence);
    wire->pan_steps = (int32_t)htonl((uint32_t)host->pan_steps);
    wire->pan_rate = (int16_t)htons((uint16_t)host->pan_rate);
    wire->tilt_angle_decidegrees =
        (int16_t)htons((uint16_t)host->tilt_angle_decidegrees);
    wire->state_flags = htons(host->state_flags);
    wire->vision_latency_ms = htons(host->vision_latency_ms);
}

static int open_listening_socket(void)
{
    const int server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_fd < 0) {
        ESP_LOGE(TAG, "Could not create the control socket: errno %d", errno);
        return -1;
    }

    const int reuse_address = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR,
               &reuse_address, sizeof(reuse_address));

    const struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(CONTROL_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(server_fd, (const struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(server_fd, 1) != 0) {
        ESP_LOGE(TAG, "Could not listen on TCP port %d: errno %d",
                 CONTROL_TCP_PORT, errno);
        close(server_fd);
        return -1;
    }
    return server_fd;
}

static void configure_client_socket(int client_fd)
{
    const int enabled = 1;
    const struct timeval timeout = {
        .tv_sec = CONTROL_RECEIVE_TIMEOUT_MS / 1000,
        .tv_usec = (CONTROL_RECEIVE_TIMEOUT_MS % 1000) * 1000,
    };

    /* Commands are 20 bytes and must not wait for a full segment. */
    setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled));
    setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

static void serve_one_client(int client_fd)
{
    int64_t last_command_us = esp_timer_get_time();
    uint32_t commands = 0;

    while (true) {
        gimbal_command_t wire_command;
        bool timed_out = false;
        if (!receive_exactly(client_fd, &wire_command, sizeof(wire_command),
                             &timed_out)) {
            if (!timed_out) {
                ESP_LOGI(TAG, "Tracking client disconnected after %" PRIu32
                              " commands", commands);
                return;
            }
            /* The motion watchdog has already stopped the axes. Give the
             * desktop a little longer to come back before freeing the slot. */
            if ((esp_timer_get_time() - last_command_us) >
                ((int64_t)CONTROL_IDLE_DROP_MS * 1000)) {
                ESP_LOGW(TAG, "No tracking commands for %d ms; dropping client",
                         CONTROL_IDLE_DROP_MS);
                return;
            }
            continue;
        }

        gimbal_command_t command;
        if (!decode_command(&wire_command, &command)) {
            /* A bad header means the byte stream is out of step, and there is
             * no safe way to resynchronise a fixed-size protocol. */
            ESP_LOGW(TAG, "Malformed command (magic 0x%08" PRIx32
                          ", version %u, %u bytes); closing",
                     command.magic, command.version, command.message_bytes);
            return;
        }

        gimbal_control_submit(&command);
        last_command_us = esp_timer_get_time();
        commands++;

        gimbal_status_t status;
        gimbal_control_fill_status(command.sequence, &status);

        gimbal_status_t wire_status;
        encode_status(&status, &wire_status);
        if (!send_all(client_fd, &wire_status, sizeof(wire_status))) {
            ESP_LOGW(TAG, "Could not reply to the tracking client (errno %d)", errno);
            return;
        }
    }
}

static void control_server_task(void *unused)
{
    (void)unused;
    ESP_LOGI(TAG, "Control task is running on CPU %d", xPortGetCoreID());

    while (true) {
        const int server_fd = open_listening_socket();
        if (server_fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Tracker can connect on TCP port %d", CONTROL_TCP_PORT);
        struct sockaddr_in client_address;
        socklen_t client_address_size = sizeof(client_address);
        const int client_fd = accept(
            server_fd, (struct sockaddr *)&client_address, &client_address_size);
        close(server_fd);

        if (client_fd < 0) {
            ESP_LOGW(TAG, "Accept failed: errno %d", errno);
            continue;
        }

        configure_client_socket(client_fd);
        ESP_LOGI(TAG, "Tracker connected from %s",
                 inet_ntoa(client_address.sin_addr));

        serve_one_client(client_fd);

        /* Whatever ended the session, the gimbal must not keep moving on the
         * last instruction it happened to receive. */
        gimbal_control_stop_motion();
        shutdown(client_fd, SHUT_RDWR);
        close(client_fd);
    }
}

esp_err_t control_server_start(void)
{
    const BaseType_t created = xTaskCreatePinnedToCore(control_server_task,
                                                       "gimbal_control_rx",
                                                       CONTROL_TASK_STACK_BYTES,
                                                       NULL,
                                                       CONTROL_TASK_PRIORITY,
                                                       NULL,
                                                       CONTROL_TASK_CORE);
    return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
