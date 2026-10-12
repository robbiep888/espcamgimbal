#include "stream_server.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "app_config.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "stream_protocol.h"

/*
 * Taken unchanged from the optimizedcam project, where these exact tasks and
 * queues were measured at about 22 fps on this board. The gimbal deliberately
 * does not touch this file: tracking rides on a separate socket and a separate
 * task so it can never add latency to the video path.
 */

static const char *TAG = "stream";

typedef struct {
    uint8_t *jpeg;
    size_t jpeg_bytes;
    uint16_t width;
    uint16_t height;
    uint32_t sequence;
    int64_t captured_at_us;
    uint32_t dropped_before_this_frame;
} queued_frame_t;

static QueueHandle_t latest_frame_queue;
static QueueHandle_t available_buffer_queue;
static uint8_t *staging_buffers[STREAM_STAGING_BUFFER_COUNT];
static uint32_t dropped_frames;

static uint64_t host_to_network_u64(uint64_t value)
{
    const uint32_t high = htonl((uint32_t)(value >> 32));
    const uint32_t low = htonl((uint32_t)(value & 0xffffffffU));
    return ((uint64_t)low << 32) | high;
}

static void return_queued_frame(const queued_frame_t *queued)
{
    if (queued->jpeg != NULL) {
        /* Every staging buffer has exactly one owner, so this should never
         * block. A zero timeout makes an ownership bug visible immediately. */
        if (xQueueSend(available_buffer_queue, &queued->jpeg, 0) != pdPASS) {
            ESP_LOGE(TAG, "Staging-buffer ownership error");
        }
    }
}

/* Prefer an unused staging buffer. If both are busy, reclaim the queued stale
 * frame; the network-owned frame is never touched. Returning NULL means the
 * network currently owns one buffer and won the race to take the other. */
static uint8_t *acquire_staging_buffer(void)
{
    uint8_t *buffer = NULL;
    if (xQueueReceive(available_buffer_queue, &buffer, 0) == pdPASS) {
        return buffer;
    }

    queued_frame_t stale_frame = {0};
    if (xQueueReceive(latest_frame_queue, &stale_frame, 0) == pdPASS) {
        dropped_frames++;
        return stale_frame.jpeg;
    }
    return NULL;
}

/* Put a frame in the one-item queue. If it is full, discard the older frame.
 * This small function is the heart of the low-latency design. */
static void publish_latest_frame(const queued_frame_t *new_frame)
{
    if (xQueueSend(latest_frame_queue, new_frame, 0) == pdPASS) {
        return;
    }

    queued_frame_t stale_frame = {0};
    if (xQueueReceive(latest_frame_queue, &stale_frame, 0) == pdPASS) {
        return_queued_frame(&stale_frame);
        dropped_frames++;
    }

    if (xQueueSend(latest_frame_queue, new_frame, 0) != pdPASS) {
        /* The sender won the race and filled the queue again. Never leak a
         * camera buffer: return this frame and try again on the next capture. */
        return_queued_frame(new_frame);
        dropped_frames++;
    }
}

static void camera_capture_task(void *unused)
{
    (void)unused;
    uint32_t sequence = 0;

    ESP_LOGI(TAG, "Camera task is running on CPU %d", xPortGetCoreID());

    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            ESP_LOGW(TAG, "Camera did not return a frame");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (frame->format != PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "Camera returned non-JPEG data; dropping frame");
            esp_camera_fb_return(frame);
            continue;
        }

        const bool has_jpeg_markers =
            frame->len >= 4 &&
            frame->buf[0] == 0xff && frame->buf[1] == 0xd8 &&
            frame->buf[frame->len - 2] == 0xff &&
            frame->buf[frame->len - 1] == 0xd9;
        if (!has_jpeg_markers || frame->len > STREAM_STAGING_BUFFER_BYTES) {
            ESP_LOGW(TAG, "Dropping incomplete or oversized JPEG (%u bytes)",
                     (unsigned)frame->len);
            esp_camera_fb_return(frame);
            dropped_frames++;
            continue;
        }

        uint8_t *jpeg_copy = acquire_staging_buffer();
        if (jpeg_copy == NULL) {
            /* The sender and mailbox are both busy. Drop this new frame rather
             * than retaining a camera buffer and stopping sensor capture. */
            esp_camera_fb_return(frame);
            dropped_frames++;
            continue;
        }

        const size_t jpeg_bytes = frame->len;
        const uint16_t width = (uint16_t)frame->width;
        const uint16_t height = (uint16_t)frame->height;
        memcpy(jpeg_copy, frame->buf, jpeg_bytes);
        esp_camera_fb_return(frame);

        const queued_frame_t queued = {
            .jpeg = jpeg_copy,
            .jpeg_bytes = jpeg_bytes,
            .width = width,
            .height = height,
            .sequence = sequence++,
            .captured_at_us = esp_timer_get_time(),
            .dropped_before_this_frame = dropped_frames,
        };
        publish_latest_frame(&queued);
    }
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

static bool send_frame(int client_fd, const queued_frame_t *queued)
{
    const stream_frame_header_t header = {
        .magic = htonl(STREAM_MAGIC),
        .version = htons(STREAM_PROTOCOL_VERSION),
        .header_bytes = htons(sizeof(stream_frame_header_t)),
        .sequence = htonl(queued->sequence),
        .captured_at_us = host_to_network_u64((uint64_t)queued->captured_at_us),
        .jpeg_bytes = htonl((uint32_t)queued->jpeg_bytes),
        .width = htons(queued->width),
        .height = htons(queued->height),
        .dropped_frames = htonl(queued->dropped_before_this_frame),
    };

    return send_all(client_fd, &header, sizeof(header)) &&
           send_all(client_fd, queued->jpeg, queued->jpeg_bytes);
}

static void configure_client_socket(int client_fd)
{
    const int enabled = 1;
    const struct timeval timeout = {
        .tv_sec = CLIENT_SEND_TIMEOUT_MS / 1000,
        .tv_usec = (CLIENT_SEND_TIMEOUT_MS % 1000) * 1000,
    };
    const int send_buffer_bytes = CLIENT_SEND_BUFFER_BYTES;

    setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled));
    setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(client_fd, SOL_SOCKET, SO_SNDBUF,
               &send_buffer_bytes, sizeof(send_buffer_bytes));
}

static int open_listening_socket(void)
{
    const int server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_fd < 0) {
        ESP_LOGE(TAG, "Could not create TCP socket: errno %d", errno);
        return -1;
    }

    const int reuse_address = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR,
               &reuse_address, sizeof(reuse_address));

    const struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(STREAM_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(server_fd, (const struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(server_fd, 1) != 0) {
        ESP_LOGE(TAG, "Could not listen on TCP port %d: errno %d",
                 STREAM_TCP_PORT, errno);
        close(server_fd);
        return -1;
    }
    return server_fd;
}

static void network_stream_task(void *unused)
{
    (void)unused;
    ESP_LOGI(TAG, "Network task is running on CPU %d", xPortGetCoreID());

    while (true) {
        const int server_fd = open_listening_socket();
        if (server_fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Viewer can connect on TCP port %d", STREAM_TCP_PORT);
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
        ESP_LOGI(TAG, "Viewer connected from %s",
                 inet_ntoa(client_address.sin_addr));

        int64_t stats_started_us = esp_timer_get_time();
        uint32_t frames_sent = 0;
        uint64_t bytes_sent = 0;

        while (true) {
            queued_frame_t queued = {0};
            if (xQueueReceive(latest_frame_queue,
                              &queued,
                              pdMS_TO_TICKS(2000)) != pdPASS) {
                continue;
            }

            const bool success = send_frame(client_fd, &queued);
            if (success) {
                frames_sent++;
                bytes_sent += queued.jpeg_bytes;
            }
            return_queued_frame(&queued);

            if (!success) {
                ESP_LOGW(TAG, "Viewer disconnected or became too slow (errno %d)", errno);
                break;
            }

            const int64_t now_us = esp_timer_get_time();
            if (now_us - stats_started_us >= 5000000) {
                const float seconds = (now_us - stats_started_us) / 1000000.0f;
                ESP_LOGI(TAG, "%.1f fps, %.2f Mbit/s, %" PRIu32 " stale frames dropped",
                         frames_sent / seconds,
                         (bytes_sent * 8.0f) / (seconds * 1000000.0f),
                         dropped_frames);
                stats_started_us = now_us;
                frames_sent = 0;
                bytes_sent = 0;
            }
        }

        shutdown(client_fd, SHUT_RDWR);
        close(client_fd);
    }
}

esp_err_t stream_server_start(void)
{
    latest_frame_queue = xQueueCreate(1, sizeof(queued_frame_t));
    available_buffer_queue = xQueueCreate(
        STREAM_STAGING_BUFFER_COUNT, sizeof(uint8_t *));
    if (latest_frame_queue == NULL || available_buffer_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    for (size_t index = 0; index < STREAM_STAGING_BUFFER_COUNT; ++index) {
        staging_buffers[index] = heap_caps_malloc(
            STREAM_STAGING_BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (staging_buffers[index] == NULL ||
            xQueueSend(available_buffer_queue, &staging_buffers[index], 0) != pdPASS) {
            ESP_LOGE(TAG, "Could not allocate PSRAM stream buffer %u",
                     (unsigned)index);
            return ESP_ERR_NO_MEM;
        }
    }
    ESP_LOGI(TAG, "Allocated %u x %u-byte PSRAM stream buffers",
             STREAM_STAGING_BUFFER_COUNT, STREAM_STAGING_BUFFER_BYTES);

    BaseType_t created = xTaskCreatePinnedToCore(
        camera_capture_task,
        "camera_capture",
        CAMERA_TASK_STACK_BYTES,
        NULL,
        CAMERA_TASK_PRIORITY,
        NULL,
        CAMERA_TASK_CORE);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreatePinnedToCore(
        network_stream_task,
        "network_stream",
        NETWORK_TASK_STACK_BYTES,
        NULL,
        NETWORK_TASK_PRIORITY,
        NULL,
        NETWORK_TASK_CORE);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
