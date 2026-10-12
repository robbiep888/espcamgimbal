#pragma once

#include "board_pins.h"

/*
 * ============================ READ THIS FIRST =============================
 *
 * The originally planned wiring (STEP=4, DIR=5, ENABLE=6, servo=7) CANNOT be
 * used on this board. All four of those pins belong to the camera:
 *
 *     GPIO 4 -> SCCB SDA      GPIO 6 -> VSYNC
 *     GPIO 5 -> SCCB SCL      GPIO 7 -> HREF
 *
 * Driving them from the gimbal makes esp_camera_init() fail, or produces
 * torn frames if it does come up. The four pins below are free on this
 * carrier and are what the driver should actually be wired to.
 *
 * Pins that are NOT available on an ESP32-S3-WROOM-1-N16R8 camera board:
 *   4-13, 15-18  camera bus                 26-32  SPI flash
 *   33-37        octal PSRAM (the R8 part)  43, 44 UART0 console
 *   19, 20       native USB D-/D+           0, 3, 45, 46  strapping
 *   38, 39, 40   routed to the microSD slot on this layout
 *
 * Verified-free alternatives if one of the defaults is inconvenient on your
 * board's headers: 14, 21, 47, 48.
 * ==========================================================================
 */

/* Step/direction driver for the NEMA 17 pan axis (A4988 / DRV8825 / TMC2209). */
#define GIMBAL_PIN_STEPPER_STEP    1
#define GIMBAL_PIN_STEPPER_DIR     2
#define GIMBAL_PIN_STEPPER_ENABLE 42

/* MG996R tilt servo signal line. */
#define GIMBAL_PIN_SERVO_PWM      41

/*
 * REQUIRED external resistors. Neither is optional on this build.
 *
 *   10 k from GIMBAL_PIN_STEPPER_ENABLE to the driver's 3V3 logic rail.
 *   10 k from GIMBAL_PIN_STEPPER_STEP   to GND.
 *
 * A4988, DRV8825 and TMC2209 all hold ENABLE low internally, so with the pin
 * undriven the driver is ENABLED. The ESP32-S3 leaves every GPIO
 * high-impedance from power-on until app_main runs -- about 885 ms, or 445 ms
 * with CONFIG_SPIRAM_MEMTEST off. Through that whole window the coils are live
 * and the floating STEP line picks up switching noise from the buck
 * converters, which the driver faithfully turns into steps: the motor spins
 * for a second or two on every power-up and reset, with no firmware involved.
 *
 * The pull-up keeps the driver disabled until firmware takes over; the STEP
 * pull-down stops noise being mistaken for pulses. No software fix exists,
 * because the CPU is not running yet.
 */

/* ---------------------------------------------------------------------- */
/* Compile-time protection. Editing a pin above to a camera or system pin  */
/* now fails the build instead of failing silently on the bench.           */
/* ---------------------------------------------------------------------- */

#define GIMBAL_PIN_IS_CAMERA_PIN(pin)                                          \
    ((pin) == CAMERA_PIN_XCLK  || (pin) == CAMERA_PIN_SIOD  ||                 \
     (pin) == CAMERA_PIN_SIOC  || (pin) == CAMERA_PIN_VSYNC ||                 \
     (pin) == CAMERA_PIN_HREF  || (pin) == CAMERA_PIN_PCLK  ||                 \
     (pin) == CAMERA_PIN_D0    || (pin) == CAMERA_PIN_D1    ||                 \
     (pin) == CAMERA_PIN_D2    || (pin) == CAMERA_PIN_D3    ||                 \
     (pin) == CAMERA_PIN_D4    || (pin) == CAMERA_PIN_D5    ||                 \
     (pin) == CAMERA_PIN_D6    || (pin) == CAMERA_PIN_D7)

#define GIMBAL_PIN_IS_RESERVED(pin)                                            \
    ((pin) < 0 || (pin) > 48 ||                                                \
     (pin) == 0  || (pin) == 3  || (pin) == 45 || (pin) == 46 || /* strap   */ \
     (pin) == 19 || (pin) == 20 ||                               /* USB     */ \
     (pin) == 43 || (pin) == 44 ||                               /* UART0   */ \
     ((pin) >= 22 && (pin) <= 25) ||                        /* not bonded   */ \
     ((pin) >= 26 && (pin) <= 32) ||                        /* SPI flash    */ \
     ((pin) >= 33 && (pin) <= 37))                          /* octal PSRAM  */

#define GIMBAL_ASSERT_PIN_USABLE(pin, name)                                    \
    _Static_assert(!GIMBAL_PIN_IS_CAMERA_PIN(pin),                             \
                   name " collides with a camera pin; see gimbal_pins.h");     \
    _Static_assert(!GIMBAL_PIN_IS_RESERVED(pin),                               \
                   name " is a strapping, flash, PSRAM, USB or UART pin")

GIMBAL_ASSERT_PIN_USABLE(GIMBAL_PIN_STEPPER_STEP, "GIMBAL_PIN_STEPPER_STEP");
GIMBAL_ASSERT_PIN_USABLE(GIMBAL_PIN_STEPPER_DIR, "GIMBAL_PIN_STEPPER_DIR");
GIMBAL_ASSERT_PIN_USABLE(GIMBAL_PIN_STEPPER_ENABLE, "GIMBAL_PIN_STEPPER_ENABLE");
GIMBAL_ASSERT_PIN_USABLE(GIMBAL_PIN_SERVO_PWM, "GIMBAL_PIN_SERVO_PWM");

_Static_assert(GIMBAL_PIN_STEPPER_STEP != GIMBAL_PIN_STEPPER_DIR &&
                   GIMBAL_PIN_STEPPER_STEP != GIMBAL_PIN_STEPPER_ENABLE &&
                   GIMBAL_PIN_STEPPER_STEP != GIMBAL_PIN_SERVO_PWM &&
                   GIMBAL_PIN_STEPPER_DIR != GIMBAL_PIN_STEPPER_ENABLE &&
                   GIMBAL_PIN_STEPPER_DIR != GIMBAL_PIN_SERVO_PWM &&
                   GIMBAL_PIN_STEPPER_ENABLE != GIMBAL_PIN_SERVO_PWM,
               "Each gimbal output needs its own GPIO");
