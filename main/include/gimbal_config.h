#pragma once

/*
 * Motion tuning for the NEMA 17 pan base and the MG996R tilt joint.
 *
 * The desktop only reports WHERE the hand is. Everything about HOW the gimbal
 * moves -- gain, speed ceiling, acceleration, soft limits, failsafe -- is
 * decided here, on the board. That split means a late or dropped Wi-Fi packet
 * degrades tracking accuracy but can never produce a violent motion command.
 */

/* ----------------------------- control loop ----------------------------- */

/* Fixed-rate loop. 100 Hz is far above the ~20 fps vision rate, so motion is
 * interpolated smoothly between detections instead of stepping per frame. */
#define GIMBAL_CONTROL_HZ 100

/* Failsafe: with no fresh command for this long, ramp the pan axis to a stop
 * and hold tilt where it is. Covers viewer crashes and Wi-Fi dropouts. */
#define GIMBAL_COMMAND_TIMEOUT_MS 400

/* De-energise the stepper after this long with no commanded motion. The pan
 * axis is vertical, so nothing back-drives it, and the motor and driver stay
 * cool. Set to 0 to always keep holding torque. */
#define GIMBAL_IDLE_DISABLE_MS 5000

/*
 * Ignore errors smaller than this fraction of the working area. Without it the
 * gimbal hunts around dead centre forever.
 *
 * The number that matters is not this one on its own: nothing moves until the
 * commanded rate also clears GIMBAL_PAN_MIN_STEP_RATE, so the two together set
 * where the axis actually wakes up. At 0.042 with a 25 steps/s floor that is
 * about 1.9 degrees off axis, or 18 px -- against 4.7 degrees and 45 px before,
 * which is most of why tracking used to settle so loosely.
 */
#define GIMBAL_ERROR_DEADBAND 0.040f

/* ------------------------------- pan axis ------------------------------- */

/*
 * Motor and driver geometry.
 *
 * MICROSTEPS *must* match the driver's MS pins. Nothing checks this, and a
 * mismatch silently rescales every speed, distance and soft limit in this file:
 * with the firmware set to 8 while an A4988 sat at its default full step, one
 * arrow press moved about 45 degrees instead of 8, and the +/-90 degree soft
 * limit was really +/-720 degrees -- two full revolutions of camera ribbon.
 *
 * A4988 at 1/16 is MS1, MS2 and MS3 all tied to the driver's 3V3 logic rail.
 * Leaving them unconnected is full step, because each has an internal
 * pull-down -- which is what a bare, unjumpered module does.
 */
#define GIMBAL_STEPPER_FULL_STEPS_PER_REV 200
#define GIMBAL_STEPPER_MICROSTEPS 16
#define GIMBAL_STEPPER_STEPS_PER_REV \
    (GIMBAL_STEPPER_FULL_STEPS_PER_REV * GIMBAL_STEPPER_MICROSTEPS)

/* Extra reduction between motor shaft and pan axis. 1.0 for a direct drive. */
#define GIMBAL_PAN_GEAR_RATIO 1.0f

/*
 * Proportional gain: step rate commanded when the hand sits at the edge of the
 * working area (error magnitude 1.0).
 *
 * Sized against measurements from this build rather than guessed. The camera's
 * horizontal field of view was measured at 66 deg (a commanded 13.1 deg tilt
 * moved the picture 120 px of 480), so with the 0.8 working area an error of
 * 1.0 means 26.5 deg off axis. End-to-end delay is about 156 ms: capture, Wi-Fi,
 * decode, inference, the 50 Hz command hop, and the centre smoothing's own lag.
 *
 * A rate-controlled loop with delay L goes unstable as loop gain K approaches
 * pi/2 / L. At 900 steps/s, K was 7.6 /s and K*L = 1.19 -- only 25% of margin,
 * and that is before any extra latency from a busy network. 375 steps/s puts
 * K*L near 0.5, which settles without ringing and makes a wrong-direction
 * runaway slow enough to watch and stop.
 */
#define GIMBAL_PAN_GAIN_STEPS_PER_S 1050.0f

/* Speed ceiling, and the rate below which it is not worth moving at all.
 * Doubled along with the gain for 1/16, so 520 steps/s is the same 58 deg/s
 * that 260 steps/s was meant to be at 1/8. */
#define GIMBAL_PAN_MAX_STEP_RATE 1350.0f
#define GIMBAL_PAN_MIN_STEP_RATE 25.0f

/* Acceleration limit, in steps/s per second. This is what stops a NEMA 17 from
 * skipping when the hand appears suddenly, and it sets how abruptly the axis
 * starts and stops. 1200 takes about 0.2 s to reach full speed. */
#define GIMBAL_PAN_ACCEL_STEPS_PER_S2 8000.0f

/* Low-pass on the commanded step rate, in Hz.
 *
 * The acceleration limit caps how fast the rate may change but does nothing
 * about chasing jitter within that cap: the hand centre wobbles by a few pixels
 * every frame, and without this the axis follows every wobble, which is what
 * makes the motion feel nervous rather than smooth. 3 Hz passes real hand
 * movement and rejects per-frame noise. Lower it for glassier motion at the
 * cost of response; it adds roughly 50 ms of lag at this setting. */
#define GIMBAL_PAN_RATE_SMOOTHING_HZ 5.5f

/* Soft travel limits measured from the power-on position, to protect the
 * camera ribbon and motor wiring from winding up. Also caps how far a
 * wrong-direction runaway can travel before it is stopped by something other
 * than the hand leaving the frame. */
#define GIMBAL_PAN_LIMIT_DEGREES 90.0f

/* Recentring approaches home proportionally: this many steps/s of speed for
 * every step still to travel. The commanded rate is floored at
 * GIMBAL_PAN_MIN_STEP_RATE on the way in, because a rate below that is not
 * emitted at all -- without the floor the axis parks a few steps short of home
 * and never arrives. */
#define GIMBAL_RECENTER_GAIN 3.0f

/* Recentring is latched so one keypress finishes the move, and it outranks
 * tracking while it runs. This bound guarantees it can never wedge and leave
 * hand-following permanently disabled. */
#define GIMBAL_RECENTER_TIMEOUT_MS 15000

/* Confirmed on this rig with desktop/jog.py once 1/16 microstepping was in and
 * the arrow keys actually worked: with 0 the camera turned away from the hand.
 * Which value is right depends on how the coils happen to be wired. The p key
 * in jog.py and the tracker tries the other value live, without reflashing. */
#define GIMBAL_PAN_INVERT 0

/* ------------------------------ tilt axis ------------------------------- */

/* MG996R pulse envelope. 600-2400 us spans its full ~180 degrees. Narrow these
 * if your particular servo buzzes at an extreme. */
#define GIMBAL_SERVO_MIN_PULSE_US 600
#define GIMBAL_SERVO_MAX_PULSE_US 2400
#define GIMBAL_SERVO_RANGE_DEGREES 180.0f

/* Mechanical travel actually available on the yoke. Staying inside this keeps
 * the servo from driving the camera into the frame and stalling at 2.5 A. */
/*
 * WARNING: these are a guess unless you have measured your own yoke. Driving an
 * MG996R into a mechanical stop stalls it at about 2.5 A, which browns out its
 * supply and will eventually destroy the servo. The previous 50-130 range was
 * never measured against this build and testing drove it into both ends.
 *
 * Widen this only after checking by hand where the camera actually fouls the
 * frame, and leave a few degrees of margin at each end.
 */
#define GIMBAL_TILT_CENTER_DEGREES 90.0f
/* LOWER this number for more UPWARD travel: increasing the servo angle pitches
 * the camera down, so the minimum is the top of the camera's reach. */
#define GIMBAL_TILT_MIN_DEGREES 10.0f
/* RAISE this for more downward travel. */
#define GIMBAL_TILT_MAX_DEGREES 160.0f
/* The servo is position-controlled, so vertical error is integrated into an
 * angle. This is the slew rate commanded at error magnitude 1.0. */
/*
 * These were cut to 22/30 along with the pan axis when the stepper needed
 * calming, but the two problems were unrelated: pan was jolting, tilt was not.
 * An MG996R slews about 350 deg/s unloaded at 6 V, so 30 deg/s was under a
 * tenth of what the servo can do and it looked broken rather than smooth.
 * Nothing about this is limited by supply current.
 */
#define GIMBAL_TILT_GAIN_DEG_PER_S 120.0f
#define GIMBAL_TILT_MAX_RATE_DEG_PER_S 160.0f

/* Low-pass on the commanded angle, in Hz. An MG996R is geared and a little
 * loose; smoothing here is cheaper than fighting the backlash. */
#define GIMBAL_TILT_SMOOTHING_HZ 7.0f

/* Flip if the gimbal tilts away from your hand instead of toward it. */
#define GIMBAL_TILT_INVERT 0

/* -------------------------- predictive tracking ------------------------- */

/*
 * Used whenever the desktop sends measurements (hand angle plus the frame's
 * capture time) rather than manual speeds. The jog tools still use the
 * proportional settings above; this section governs following a hand.
 *
 * Tuned in a closed-loop simulation of this rig -- 17 fps, ~100 ms from
 * exposure to arrival, 0.25 deg landmark jitter, the stepper limits above and
 * an MG996R modelled as a 45 ms lag -- then checked against a world that was
 * wrong: exposure delay 25 or 70 ms, 12 or 22 fps, Wi-Fi 80 ms slower, twice
 * the jitter. Against the proportional loop it replaces, a 20 degree jump now
 * settles in about 540 ms with no overshoot (was 970 ms and 5 degrees), a
 * 40 deg/s sweep is followed within 4.6 degrees RMS (was 11), and a still hand
 * is centred to 0.1 degree (was 0.3). The proportional loop fell apart with
 * the slower Wi-Fi (13 degrees of overshoot); this one stayed under 1.
 */

/* Time from the middle of a frame's exposure to the capture timestamp in the
 * stream header: rolling-shutter readout, JPEG output and the copy out of the
 * driver. Guessing long hurts less than guessing short, hence 55 ms against a
 * best estimate of 45. */
#define GIMBAL_CAMERA_EXPOSURE_DELAY_MS 55

/* How far the MG996R trails the angle it was sent. Only used to look up where
 * the camera really pointed; the servo is not driven ahead of its target,
 * which the simulation showed overshooting whenever the real lag was shorter
 * than assumed. */
#define GIMBAL_TILT_SERVO_LAG_MS 45

/* Stop following if no new frame has arrived for this long, even while
 * commands keep coming. The desktop also withholds tracking from stale video. */
#define GIMBAL_MEASUREMENT_TIMEOUT_MS 350

/* Ignore a measurement whose frame is older than this when it arrives. */
#define GIMBAL_MEASUREMENT_MAX_AGE_MS 800

/* Alpha-beta filter on the hand's absolute angle. See target_tracker.h. */
#define GIMBAL_TRACK_FILTER_ALPHA 0.6f
#define GIMBAL_TRACK_FILTER_BETA 0.4f
#define GIMBAL_TRACK_JUMP_DEGREES 7.0f
#define GIMBAL_TRACK_SPEED_NOISE_DEG_S 8.0f
#define GIMBAL_TRACK_MAX_SPEED_DEG_S 180.0f
#define GIMBAL_TRACK_REACQUIRE_GAP_MS 300

/* How far past a frame's exposure the hand's position is projected. Frames are
 * 100-150 ms old by the time they steer, so a short horizon trails a moving
 * hand and a long one overshoots when it stops. Simulated, as trail during a
 * sweep / overshoot after it stops, at 20 and 40 deg/s:
 *     60 ms  2.6 / 0.9 and 4.2 / 2.4 degrees   (measured on the board: 2.8 trail)
 *    120 ms  1.9 / 1.6 and 2.3 / 4.0
 *    150 ms  1.6 / 2.0 and 1.6 / 4.7
 * 60 is also the only one that kept a still hand steady with noisy landmarks. */
#define GIMBAL_TRACK_MAX_EXTRAPOLATION_MS 60

/* Pan approach: degrees/s commanded per degree still to go near the target.
 * Far from it the braking curve takes over, planned at this share of
 * GIMBAL_PAN_ACCEL_STEPS_PER_S2 so the acceleration limit never has to cut in
 * and cause the overshoot it would otherwise leave behind. */
#define GIMBAL_TRACK_PAN_GAIN 16.0f
#define GIMBAL_TRACK_ACCEL_SHARE 0.8f

/* Share of the hand's own speed added on top, so a moving hand is followed
 * rather than chased. */
#define GIMBAL_TRACK_FEEDFORWARD 1.0f

/* Hold-still hysteresis for both axes, in degrees: a settled axis wakes when
 * the hand is more than START away and settles again inside STOP. */
#define GIMBAL_TRACK_START_DEGREES 0.8f
#define GIMBAL_TRACK_STOP_DEGREES 0.25f

/* ------------------------------- hardware ------------------------------- */

/* LEDC resources. Timer 0 / channel 0 belong to the camera's XCLK -- do not
 * reuse them. Timer 1 emits the step pulse train and timer 2 the servo frame,
 * both entirely in hardware, so neither costs a single CPU cycle per pulse. */
#define GIMBAL_STEPPER_LEDC_TIMER LEDC_TIMER_1
#define GIMBAL_STEPPER_LEDC_CHANNEL LEDC_CHANNEL_1
#define GIMBAL_SERVO_LEDC_TIMER LEDC_TIMER_2
#define GIMBAL_SERVO_LEDC_CHANNEL LEDC_CHANNEL_2

/* 13-bit step pulses from the 80 MHz APB clock give a usable range of roughly
 * 10 Hz to 9.7 kHz, which comfortably contains the rates configured above. */
#define GIMBAL_STEPPER_DUTY_RESOLUTION LEDC_TIMER_13_BIT

/* 14 bits across a 20 ms servo frame resolves to about 1.2 us of pulse width,
 * finer than an MG996R can act on. */
#define GIMBAL_SERVO_DUTY_RESOLUTION LEDC_TIMER_14_BIT
#define GIMBAL_SERVO_FRAME_HZ 50

/* Most drivers want the direction line settled before the next step edge. */
#define GIMBAL_STEPPER_DIR_SETUP_US 20

/* ENABLE polarity. A4988, DRV8825 and TMC2209 all enable on a LOW level. */
#define GIMBAL_STEPPER_ENABLE_ACTIVE_LOW 1
