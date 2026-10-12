#pragma once

/*
 * Camera pin table for the dual-USB-C ESP32-S3-CAM carrier (Freenove
 * ESP32-S3-WROOM CAM layout, also sold as Keyestudio MB0184 and under several
 * generic names). "ESP32-S3 N16R8" names the module's memory, not the board.
 *
 * These values are carried over unchanged from the working optimizedcam build,
 * so the stream in this project behaves identically.
 *
 * If a later board revision differs, this is the ONLY file to edit.
 * Camera labels Y2..Y9 correspond to sensor data bits D0..D7.
 *
 * WARNING: every pin below is occupied whenever the camera is running. The
 * gimbal outputs in gimbal_pins.h must not reuse any of them. A compile-time
 * check in that file enforces this.
 */

#define CAMERA_PIN_PWDN  -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK  15
#define CAMERA_PIN_SIOD   4
#define CAMERA_PIN_SIOC   5

#define CAMERA_PIN_D0    11 /* camera Y2 */
#define CAMERA_PIN_D1     9 /* camera Y3 */
#define CAMERA_PIN_D2     8 /* camera Y4 */
#define CAMERA_PIN_D3    10 /* camera Y5 */
#define CAMERA_PIN_D4    12 /* camera Y6 */
#define CAMERA_PIN_D5    18 /* camera Y7 */
#define CAMERA_PIN_D6    17 /* camera Y8 */
#define CAMERA_PIN_D7    16 /* camera Y9 */

#define CAMERA_PIN_VSYNC  6
#define CAMERA_PIN_HREF   7
#define CAMERA_PIN_PCLK  13
