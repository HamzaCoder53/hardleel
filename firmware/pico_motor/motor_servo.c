/**
 * Motor + servo firmware - Raspberry Pi Pico SDK (RP2040)
 *
 * Control 4 TB6612 motors and 2 servos (same angle) from a PC / Raspberry Pi
 * over USB serial. Optional quadrature encoders (ENABLE_ENCODERS).
 *
 * Motor index:  0 = LF, 1 = LR, 2 = RF, 3 = RR
 *
 * Commands (PC -> Pico, newline terminated, case sensitive):
 *   M,<id>,<pwm>      one motor, pwm -255..255 (sign = direction, 0 = brake)
 *   ALL,<pwm>         all four motors, same pwm
 *   STBY,<d>,<v>      driver standby pin: d = 0 (both) / 1 (left driver) / 2 (right driver)
 *                     v = 1 enable driver, 0 disable (motors coast)
 *   STOP              all pwm = 0 -> short brake (standby unchanged)
 *   COAST             all motors free-wheel (both INs low)
 *   WD,<ms>           command watchdog in ms (0 = disabled, default 200)
 *                     (motors only; the servos hold their last angle)
 *   SV,<deg>          both servos to <deg> (0..180) at the default speed
 *   SV,<deg>,<dps>    same, slewing at <dps> degrees/second (0 = jump straight there)
 *   ENCRESET          zero the encoder counts            (ENABLE_ENCODERS only)
 *
 * Output (Pico -> PC):
 *   STAT,<stby1>,<stby2>,<pwm_lf>,<pwm_lr>,<pwm_rf>,<pwm_rr>        after each command
 *   SV,moving,<deg>   servos started moving to <deg>
 *   SV,done,<deg>     servos reached <deg> and settled. Hobby servos have no position
 *                     feedback, so this is a timed confirmation (slew finished + settle time).
 *                     If the servos are already at <deg>, "SV,done,<deg>" comes straight back.
 *   ENC,<t_ms>,<lf>,<lr>,<rf>,<rr>,<v_lf>,<v_lr>,<v_rf>,<v_rr>     every 50 ms (ENABLE_ENCODERS)
 *   ERR,<line>                                                       bad command
 *   WD,stopped                                                       watchdog tripped
 *
 * The drivers start DISABLED (standby low). Send "STBY,0,1" first.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

// Set to 1 only if the encoders are wired to pins that do NOT clash with the
// servos (GP12 / GP13) - see the encoder pin block below.
#ifndef ENABLE_ENCODERS
#define ENABLE_ENCODERS 0
#endif

// ============================================================================
// PINS  (PWM / IN1 / IN2)
// ============================================================================

// TB6612 Driver #1 (Left side)
#define PIN_STBY1      0
#define PIN_PWMA_LF    6
#define PIN_AIN1_LF    1
#define PIN_AIN2_LF    2
#define PIN_PWMB_LR    5
#define PIN_BIN1_LR    3
#define PIN_BIN2_LR    4

// TB6612 Driver #2 (Right side)
#define PIN_STBY2      10
#define PIN_PWMA_RF    7
#define PIN_AIN1_RF    17
#define PIN_AIN2_RF    19
#define PIN_PWMB_RR    16
#define PIN_BIN1_RR    20
#define PIN_BIN2_RR    21

// Servos: GP12 and GP13 are the A and B outputs of PWM slice 6, so they share
// one 50 Hz timer and no motor uses that slice (motors use slices 0, 2, 3).
#define PIN_SERVO1     12
#define PIN_SERVO2     13

#if ENABLE_ENCODERS
// Encoders (A, B). Power the encoders from 3.3 V, NOT 5 V.
// GP12 / GP13 are taken by the servos, so these must be remapped to free pins.
#define PIN_ENC_LF_A   11
#define PIN_ENC_LF_B   14
#define PIN_ENC_LR_A   15
#define PIN_ENC_LR_B   22
#define PIN_ENC_RF_A   26
#define PIN_ENC_RF_B   27
#define PIN_ENC_RR_A   28
#define PIN_ENC_RR_B   9      // <-- needs a free pin; GP8/GP9/GP18 are the ones that failed on your board
#endif

// ============================================================================
// CONSTANTS
// ============================================================================

#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_WRAP      255
#define DEFAULT_WD_MS       300      // final stack: 0.3 s command timeout (was 200)
#define HW_WATCHDOG_MS      500      // RP2040 hardware watchdog: a hung loop reboots the Pico (motors off)

#define SERVO_MIN_US        500      // pulse for 0 deg   (MG90: trim if it buzzes at the ends)
#define SERVO_MAX_US        2500     // pulse for 180 deg
#define SERVO2_INVERT       false    // true if servo 2 is mounted mirrored
#define SERVO_START_DEG     90       // angle held from power-up
#define SERVO_DEFAULT_DPS   240      // default slew speed, deg/s
#define SERVO_SETTLE_MS     150      // extra wait after the slew ends before "done"
#define SERVO_JUMP_MS_PER_DEG 2      // settle estimate for a jump (about 500 deg/s)

#define ENC_PERIOD_MS       50       // ENC report period (20 Hz)

// ============================================================================
// STATE
// ============================================================================

typedef struct {
    uint pwm_pin;
    uint in1_pin;
    uint in2_pin;
} motor_t;

static const motor_t MOTORS[4] = {
    { PIN_PWMA_LF, PIN_AIN1_LF, PIN_AIN2_LF },   // 0 LF
    { PIN_PWMB_LR, PIN_BIN1_LR, PIN_BIN2_LR },   // 1 LR
    { PIN_PWMA_RF, PIN_AIN1_RF, PIN_AIN2_RF },   // 2 RF
    { PIN_PWMB_RR, PIN_BIN1_RR, PIN_BIN2_RR },   // 3 RR
};

static int  pwm_cmd[4] = { 0, 0, 0, 0 };
// final stack: per-motor deadband compensation (MIN,<id>,<pwm>; 0 = off, the default) and the zero-PWM mode
// (ZERO,1 = short brake as before, the default; ZERO,0 = coast). Use deadband compensation in ONE place only:
// here OR motor_bridge (min_pwm_left/right), never both.
static int  min_pwm[4] = { 0, 0, 0, 0 };
static bool zero_brake = true;
static bool stby[2]    = { false, false };   // [0] = driver 1 (left), [1] = driver 2 (right)

static uint32_t        wd_ms = DEFAULT_WD_MS;
static absolute_time_t last_cmd_time;
static bool            wd_tripped = false;

// Servos (both always at the same angle)
static float           servo_cur   = SERVO_START_DEG;   // angle currently commanded
static int             servo_target = SERVO_START_DEG;
static float           servo_dps   = SERVO_DEFAULT_DPS;
static bool            servo_moving   = false;          // slewing towards target
static bool            servo_settling = false;          // slew done, waiting before "done"
static absolute_time_t servo_settle_end;
static absolute_time_t servo_last_update;

#if ENABLE_ENCODERS
static const uint ENC_PINS[4][2] = {
    { PIN_ENC_LF_A, PIN_ENC_LF_B },
    { PIN_ENC_LR_A, PIN_ENC_LR_B },
    { PIN_ENC_RF_A, PIN_ENC_RF_B },
    { PIN_ENC_RR_A, PIN_ENC_RR_B },
};
// Per-encoder sign: set to -1 if "robot forward" counts DOWN for that wheel.
static const int ENC_DIR[4] = { 1, 1, 1, 1 };
// index = (prev_state << 2) | new_state, state = (A << 1) | B
static const int8_t QDEC[16] = { 0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0 };
static volatile int32_t enc_count[4] = { 0, 0, 0, 0 };
static uint8_t          enc_prev[4]  = { 0, 0, 0, 0 };
#endif

// ============================================================================
// MOTORS
// ============================================================================

static void motor_init(const motor_t *m, float clkdiv) {
    gpio_init(m->in1_pin); gpio_set_dir(m->in1_pin, GPIO_OUT); gpio_put(m->in1_pin, 0);
    gpio_init(m->in2_pin); gpio_set_dir(m->in2_pin, GPIO_OUT); gpio_put(m->in2_pin, 0);

    gpio_set_function(m->pwm_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(m->pwm_pin);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, clkdiv);
    pwm_config_set_wrap(&cfg, MOTOR_PWM_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(m->pwm_pin, 0);
}

// pwm == 0 -> short brake (both INs high). Standby low releases the brake.
static void motor_set(const motor_t *m, int pwm) {
    if (pwm > 255)  pwm = 255;
    if (pwm < -255) pwm = -255;

    if (pwm == 0) {
        // brake: both INs high (output FETs switched on). coast: both low, every FET off - the safe choice if a
        // driver channel may be damaged (a failed FET + brake can short the supply)
        gpio_put(m->in1_pin, zero_brake);
        gpio_put(m->in2_pin, zero_brake);
        pwm_set_gpio_level(m->pwm_pin, 0);
        return;
    }

    gpio_put(m->in1_pin, pwm > 0);
    gpio_put(m->in2_pin, pwm < 0);

    int duty = abs(pwm);
    int idx = (int)(m - MOTORS);                       // deadband: |p| -> min + |p| * (255 - min) / 255
    if (idx >= 0 && idx < 4 && min_pwm[idx] > 0)
        duty = min_pwm[idx] + duty * (255 - min_pwm[idx]) / 255;
    if (duty > 255) duty = 255;
    // level > wrap keeps the pin high for the whole period (true 100 %)
    pwm_set_gpio_level(m->pwm_pin, duty == 255 ? MOTOR_PWM_WRAP + 1 : duty);
}

// High-impedance stop: both INs low -> motor free-wheels
static void motor_coast(const motor_t *m) {
    gpio_put(m->in1_pin, 0);
    gpio_put(m->in2_pin, 0);
    pwm_set_gpio_level(m->pwm_pin, 0);
}

static void apply_stby(void) {
    gpio_put(PIN_STBY1, stby[0]);
    gpio_put(PIN_STBY2, stby[1]);
}

static void stop_all(void) {
    for (int i = 0; i < 4; i++) {
        pwm_cmd[i] = 0;
        motor_set(&MOTORS[i], 0);
    }
}

static void coast_all(void) {
    for (int i = 0; i < 4; i++) {
        pwm_cmd[i] = 0;
        motor_coast(&MOTORS[i]);
    }
}

// ============================================================================
// SERVOS  (GP12 + GP13, 50 Hz, 1 us per PWM tick)
// ============================================================================

static void servo_write(float deg) {
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    float span = (float)(SERVO_MAX_US - SERVO_MIN_US);
    uint32_t us1 = (uint32_t)((float)SERVO_MIN_US + span * deg / 180.0f + 0.5f);
    uint32_t us2 = SERVO2_INVERT ? (uint32_t)(SERVO_MIN_US + SERVO_MAX_US) - us1 : us1;
    pwm_set_gpio_level(PIN_SERVO1, us1);
    pwm_set_gpio_level(PIN_SERVO2, us2);
}

static void servo_init(void) {
    gpio_set_function(PIN_SERVO1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_SERVO2, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_SERVO1);   // same slice as PIN_SERVO2

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);  // 1 us per tick
    pwm_config_set_wrap(&cfg, 19999);                                         // 20 ms = 50 Hz
    pwm_init(slice, &cfg, true);

    servo_cur = SERVO_START_DEG;
    servo_target = SERVO_START_DEG;
    servo_write(servo_cur);
    servo_last_update = get_absolute_time();
}

// Start a move. Returns true if the servos have to move (a "done" will follow
// later from servo_update), false if they are already at deg and idle.
static bool servo_move(int deg, int dps) {
    float delta = fabsf((float)deg - servo_cur);
    bool  idle  = !servo_moving && !servo_settling;

    servo_target = deg;
    servo_dps = (float)dps;
    servo_last_update = get_absolute_time();

    if (idle && delta < 0.5f) return false;

    if (dps <= 0) {                          // jump
        servo_cur = (float)deg;
        servo_write(servo_cur);
        servo_moving = false;
        servo_settling = true;
        servo_settle_end = make_timeout_time_ms(
            SERVO_SETTLE_MS + (uint32_t)(delta * SERVO_JUMP_MS_PER_DEG));
    } else {                                 // slew, stepped in servo_update()
        servo_moving = true;
        servo_settling = false;
    }
    return true;
}

// Call from the main loop
static void servo_update(void) {
    if (servo_moving) {
        absolute_time_t now = get_absolute_time();
        int64_t dt_us = absolute_time_diff_us(servo_last_update, now);
        if (dt_us < 10000) return;           // step every 10 ms
        servo_last_update = now;

        float step = servo_dps * (float)dt_us * 1e-6f;
        float diff = (float)servo_target - servo_cur;
        if (fabsf(diff) <= step) {
            servo_cur = (float)servo_target;
            servo_moving = false;
            servo_settling = true;
            servo_settle_end = make_timeout_time_ms(SERVO_SETTLE_MS);
        } else {
            servo_cur += (diff > 0.0f) ? step : -step;
        }
        servo_write(servo_cur);
    } else if (servo_settling &&
               absolute_time_diff_us(get_absolute_time(), servo_settle_end) <= 0) {
        servo_settling = false;
        printf("SV,done,%d\n", servo_target);
    }
}

// ============================================================================
// ENCODERS  (GPIO interrupts, x4 quadrature decoding)
// ============================================================================
#if ENABLE_ENCODERS

static void enc_isr(uint gpio, uint32_t events) {
    (void)events;
    for (int i = 0; i < 4; i++) {
        if (gpio == ENC_PINS[i][0] || gpio == ENC_PINS[i][1]) {
            uint8_t s = (uint8_t)((gpio_get(ENC_PINS[i][0]) << 1) | gpio_get(ENC_PINS[i][1]));
            enc_count[i] += QDEC[(enc_prev[i] << 2) | s] * ENC_DIR[i];
            enc_prev[i] = s;
            break;
        }
    }
}

static void encoders_init(void) {
    bool first = true;
    for (int i = 0; i < 4; i++) {
        for (int k = 0; k < 2; k++) {
            uint p = ENC_PINS[i][k];
            gpio_init(p);
            gpio_set_dir(p, GPIO_IN);
            gpio_pull_up(p);
        }
        enc_prev[i] = (uint8_t)((gpio_get(ENC_PINS[i][0]) << 1) | gpio_get(ENC_PINS[i][1]));
        for (int k = 0; k < 2; k++) {
            uint p = ENC_PINS[i][k];
            if (first) {
                gpio_set_irq_enabled_with_callback(p, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                                   true, &enc_isr);
                first = false;
            } else {
                gpio_set_irq_enabled(p, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
            }
        }
    }
}

static void encoders_reset(void) {
    uint32_t irq = save_and_disable_interrupts();
    for (int i = 0; i < 4; i++) enc_count[i] = 0;
    restore_interrupts(irq);
}

static void enc_report(void) {
    static int32_t  last_count[4] = { 0, 0, 0, 0 };
    static uint32_t last_ms = 0;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    int32_t  c[4], v[4];

    uint32_t irq = save_and_disable_interrupts();
    for (int i = 0; i < 4; i++) c[i] = enc_count[i];
    restore_interrupts(irq);

    uint32_t dt = now_ms - last_ms;
    if (dt == 0) dt = 1;
    for (int i = 0; i < 4; i++) {
        v[i] = (int32_t)(((int64_t)(c[i] - last_count[i]) * 1000) / (int64_t)dt);
        last_count[i] = c[i];
    }
    last_ms = now_ms;

    printf("ENC,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n", (unsigned long)now_ms,
           (long)c[0], (long)c[1], (long)c[2], (long)c[3],
           (long)v[0], (long)v[1], (long)v[2], (long)v[3]);
}

#endif  // ENABLE_ENCODERS

// ============================================================================
// COMMANDS
// ============================================================================

static void print_stat(void) {
    printf("STAT,%d,%d,%d,%d,%d,%d\n", stby[0], stby[1],
           pwm_cmd[0], pwm_cmd[1], pwm_cmd[2], pwm_cmd[3]);
}

static void handle_line(const char *line) {
    int a, b;
    bool ok = true;

    if (sscanf(line, "M,%d,%d", &a, &b) == 2) {
        if (a < 0 || a > 3) ok = false;
        else {
            pwm_cmd[a] = b > 255 ? 255 : (b < -255 ? -255 : b);
            motor_set(&MOTORS[a], pwm_cmd[a]);
        }
    }
    else if (sscanf(line, "ALL,%d", &a) == 1) {
        a = a > 255 ? 255 : (a < -255 ? -255 : a);
        for (int i = 0; i < 4; i++) {
            pwm_cmd[i] = a;
            motor_set(&MOTORS[i], a);
        }
    }
    else if (sscanf(line, "STBY,%d,%d", &a, &b) == 2) {
        if (a < 0 || a > 2 || (b != 0 && b != 1)) ok = false;
        else {
            if (a == 0 || a == 1) stby[0] = b;
            if (a == 0 || a == 2) stby[1] = b;
            apply_stby();
        }
    }
    else if (strncmp(line, "SV,", 3) == 0) {
        int n = sscanf(line, "SV,%d,%d", &a, &b);
        int dps = (n == 2) ? b : SERVO_DEFAULT_DPS;
        if (n < 1 || a < 0 || a > 180 || dps < 0) {
            ok = false;
        } else if (servo_move(a, dps)) {
            printf("SV,moving,%d\n", a);
        } else {
            printf("SV,done,%d\n", a);       // already there
        }
    }
    else if (strcmp(line, "STOP") == 0) {
        stop_all();
    }
    else if (strcmp(line, "COAST") == 0) {
        coast_all();
    }
#if ENABLE_ENCODERS
    else if (strcmp(line, "ENCRESET") == 0) {
        encoders_reset();
    }
#endif
    else if (sscanf(line, "WD,%d", &a) == 1) {
        wd_ms = a < 0 ? 0 : (uint32_t)a;
    }
    else if (sscanf(line, "MIN,%d,%d", &a, &b) == 2) {          // deadband compensation, id 0..3 or 4 = all
        if (a < 0 || a > 4 || b < 0 || b > 200) ok = false;
        else for (int i = 0; i < 4; i++) if (a == 4 || a == i) min_pwm[i] = b;
    }
    else if (sscanf(line, "ZERO,%d", &a) == 1) {                 // 1 = brake at PWM 0 (default), 0 = coast
        if (a != 0 && a != 1) ok = false;
        else { zero_brake = a; for (int i = 0; i < 4; i++) if (pwm_cmd[i] == 0) motor_set(&MOTORS[i], 0); }
    }
    else {
        ok = false;
    }

    if (ok) {
        last_cmd_time = get_absolute_time();
        wd_tripped = false;
        print_stat();
    } else {
        printf("ERR,%s\n", line);
    }
}

static void poll_serial(void) {
    static char   line[64];
    static size_t len = 0;
    static bool   discarding = false;   // set after an overlong line, until next newline

    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (discarding) {
                discarding = false;
            } else if (len > 0) {
                line[len] = '\0';
                handle_line(line);
            }
            len = 0;
        } else if (discarding) {
            // drop characters until newline
        } else if (len < sizeof(line) - 1) {
            line[len++] = (char)c;
        } else {
            len = 0;                    // overlong line: discard the rest of it
            discarding = true;
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    stdio_init_all();   // USB-CDC

    // Standby pins: start LOW (drivers disabled) until the PC enables them
    gpio_init(PIN_STBY1); gpio_set_dir(PIN_STBY1, GPIO_OUT);
    gpio_init(PIN_STBY2); gpio_set_dir(PIN_STBY2, GPIO_OUT);
    apply_stby();

    float motor_div = (float)clock_get_hz(clk_sys) /
                      ((float)MOTOR_PWM_FREQ_HZ * (MOTOR_PWM_WRAP + 1));
    for (int i = 0; i < 4; i++) motor_init(&MOTORS[i], motor_div);

    servo_init();
#if ENABLE_ENCODERS
    encoders_init();
    absolute_time_t next_enc = make_timeout_time_ms(ENC_PERIOD_MS);
#endif

    last_cmd_time = get_absolute_time();

    // final stack: RP2040 hardware watchdog. If this loop ever hangs, the chip reboots and every pin (motors)
    // returns to its safe low state. A watchdog reboot is reported once the USB host is listening.
    bool wd_reboot = watchdog_caused_reboot();
    watchdog_enable(HW_WATCHDOG_MS, true);
    absolute_time_t boot_msg_at = make_timeout_time_ms(1500);
    bool boot_msg_done = false;

    while (true) {
        watchdog_update();
        if (!boot_msg_done && absolute_time_diff_us(get_absolute_time(), boot_msg_at) <= 0) {
            printf("BOOT,%s\n", wd_reboot ? "watchdog" : "power");
            boot_msg_done = true;
        }
        poll_serial();
        servo_update();

        // Command watchdog: stop (brake) the motors if the PC went quiet.
        // Servos hold their last angle.
        if (wd_ms > 0 && !wd_tripped &&
            absolute_time_diff_us(last_cmd_time, get_absolute_time()) > (int64_t)wd_ms * 1000) {
            stop_all();
            wd_tripped = true;
            printf("WD,stopped\n");
            print_stat();
        }

#if ENABLE_ENCODERS
        if (absolute_time_diff_us(get_absolute_time(), next_enc) <= 0) {
            enc_report();
            next_enc = delayed_by_ms(next_enc, ENC_PERIOD_MS);
        }
#endif
    }
}
