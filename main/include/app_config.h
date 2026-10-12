#pragma once

#include "sensor.h"

/*
 * Camera and streaming settings. These values are carried over from the
 * optimizedcam build that already streams smoothly on this hardware; the
 * reasoning behind each one is preserved so they are not "tuned" away by
 * accident.
 */

/* Video frames go out on 5000. Tracking commands come back in on 5001 so the
 * hot streaming path never has to stop and poll for input. */
#define STREAM_TCP_PORT 5000
#define CONTROL_TCP_PORT 5001

/* Advertised over mDNS so the board can be reached as espgimbal.local when it
 * is running from a bench supply with no serial console attached. */
#define MDNS_HOSTNAME "espgimbal"

/*
 * VGA is the best starting point for smooth OV3660 video over 2.4 GHz Wi-Fi,
 * and it is also what the desktop hand detector wants: larger frames cost
 * inference time without making a hand easier to find.
 */
#define CAMERA_FRAME_SIZE FRAMESIZE_VGA

/* JPEG quality is 0..63; a LOWER number means a better, larger image.
 * Quality 16 keeps VGA readable while leaving more radio time for 25+ fps. */
#define CAMERA_JPEG_QUALITY 16

/* The optimizedcam module was mounted upside down and needed this; on the
 * gimbal yoke the camera sits upright, so the rotation is off. Set back to 1
 * if the module is ever remounted inverted. Both operations together rotate
 * the sensor image by 180 degrees, and the sensor does it for free.
 *
 * Note this is NOT independent of the axis direction flags: the desktop
 * measures tracking error on the final image, so rotating the picture inverts
 * both the pan and tilt error signs. Settle the orientation first, then
 * GIMBAL_PAN_INVERT / GIMBAL_TILT_INVERT. */
#define CAMERA_ROTATE_180 0

/* Two camera buffers are stable with the safe, buffered DMA path. */
#define CAMERA_FRAME_BUFFER_COUNT 2

/* TCP must never retain a camera-owned buffer. These two PSRAM buffers let the
 * network send one JPEG while the camera task prepares the next one. */
#define STREAM_STAGING_BUFFER_COUNT 2
#define STREAM_STAGING_BUFFER_BYTES (128 * 1024)

/* This board's 20 MHz camera clock polluted its 2.4 GHz radio path: the stream
 * began near 27 fps, then collapsed below 1 fps. The external 16 MHz clock plus
 * the internal PLL settings below sustains about 22 fps without that collapse. */
#define CAMERA_XCLK_HZ 16000000

/* The stock OV3660 16 MHz profile uses an 8 MHz pixel clock. This profile
 * raises only the sensor's internal clocks while XCLK stays at the radio-safe
 * 16 MHz. Multiplier 30 produces a 160 MHz VCO, 40 MHz system clock, and
 * 10 MHz pixel clock. A higher multiplier was rejected by the sensor driver.
 *
 * Off, because on the gimbal it made the camera driver miss end-of-frame: 53-61%
 * of buffers held 2-4 JPEGs glued together. The viewer decodes only the first,
 * oldest one, so ~10 fps arrived and each picture was up to 3 frames stale;
 * buffers over 61 KB were dropped outright (cam_hal: FB-OVF). The stock
 * 32 MHz/8 MHz profile measured 1-4% merged at 15-17 fps on the same power.
 * The fast profile has run clean at 22 fps from the bench supply, so it is
 * marginal rather than broken. Before turning it back on, check that received
 * JPEGs contain exactly one start-of-image marker (FF D8 FF) each. */
#define CAMERA_USE_FAST_16MHZ_PLL 0
#define CAMERA_PLL_MULTIPLIER 30
#define CAMERA_PLL_PIXEL_DIVIDER 8

/*
 * Task placement. Wi-Fi runs on CPU 0, camera work on CPU 1. The gimbal loop
 * joins CPU 1: it wakes 100 times a second, writes two hardware registers and
 * sleeps again, so it stays out of the sender's way while remaining immune to
 * network stalls.
 */
#define NETWORK_TASK_CORE 0
#define CAMERA_TASK_CORE 1
#define GIMBAL_TASK_CORE 1
#define CONTROL_TASK_CORE 0

#define NETWORK_TASK_PRIORITY 20
#define CAMERA_TASK_PRIORITY 6
#define GIMBAL_TASK_PRIORITY 10
#define CONTROL_TASK_PRIORITY 8

#define NETWORK_TASK_STACK_BYTES (6 * 1024)
#define CAMERA_TASK_STACK_BYTES (5 * 1024)
#define GIMBAL_TASK_STACK_BYTES (4 * 1024)
#define CONTROL_TASK_STACK_BYTES (4 * 1024)

/* Meant to limit TCP's own queue to a few JPEGs, but lwIP does not implement
 * SO_SNDBUF and silently ignores this. The cap that actually applies is
 * CONFIG_LWIP_TCP_SND_BUF_DEFAULT in sdkconfig.defaults; change that instead. */
#define CLIENT_SEND_BUFFER_BYTES (32 * 1024)

/* A blocked client is disconnected quickly so stale footage cannot pile up. */
#define CLIENT_SEND_TIMEOUT_MS 1500

/* Diagnostic switches used to isolate camera/Wi-Fi/motor interaction. Keep all
 * three at 0 for normal firmware. */
#define WIFI_ONLY_DIAGNOSTIC 0
#define CAMERA_IDLE_DIAGNOSTIC 0
#define GIMBAL_DISABLED_DIAGNOSTIC 0
