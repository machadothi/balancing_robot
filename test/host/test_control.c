/**
 * @file test_control.c
 * @brief PID and mixer: the worked example from docs/08 and every limit
 */

#include "control/mixer.h"
#include "control/pid.h"
#include "unit.h"

#define DT 0.01f

static PID_t default_pid(void) {
    PID_t pid = {
        .kp = 25.0f, .ki = 0.5f, .kd = 0.8f,
        .integral_limit = 100.0f, .output_limit = 255.0f,
    };
    return pid;
}

/* docs/08 "A worked sample": at rest, then 2 degrees of error in one sample */
static void test_pid_worked_example(void) {
    PID_t pid = default_pid();
    float out = pid_update(&pid, -2.0f, DT);

    CHECK_NEAR(pid.p_term, -50.0, 1e-4);
    CHECK_NEAR(pid.i_term, -0.01, 1e-5);
    CHECK_NEAR(pid.d_term, -160.0, 1e-3);
    CHECK_NEAR(out, -210.01, 1e-3);
}

static void test_pid_output_clamp(void) {
    PID_t pid = default_pid();
    CHECK_NEAR(pid_update(&pid, 50.0f, DT), 255.0, 0.0);
    CHECK_NEAR(pid_update(&pid, -100.0f, DT), -255.0, 0.0);
}

static void test_pid_integral_clamp(void) {
    PID_t pid = default_pid();
    pid.kp = 0.0f;
    pid.kd = 0.0f;

    /* 1000 deg*s of accumulated error requested, 100 allowed */
    for (int i = 0; i < 10000; i++) {
        (void)pid_update(&pid, 10.0f, DT);
    }
    CHECK_NEAR(pid.integral, 100.0, 1e-3);
    CHECK_NEAR(pid.i_term, 50.0, 1e-3);

    for (int i = 0; i < 10000; i++) {
        (void)pid_update(&pid, -10.0f, DT);
    }
    CHECK_NEAR(pid.integral, -100.0, 1e-3);
}

static void test_pid_constant_error_has_no_derivative(void) {
    PID_t pid = default_pid();
    pid.kp = 0.0f;
    pid.ki = 0.0f;

    (void)pid_update(&pid, 1.0f, DT);
    CHECK_NEAR(pid_update(&pid, 1.0f, DT), 0.0, 1e-4);
}

static void test_pid_reset_keeps_gains(void) {
    PID_t pid = default_pid();
    (void)pid_update(&pid, 3.0f, DT);
    pid_reset(&pid);

    CHECK(pid.integral == 0.0f);
    CHECK(pid.prev_error == 0.0f);
    CHECK(pid.output == 0.0f);
    CHECK(pid.kp == 25.0f && pid.ki == 0.5f && pid.kd == 0.8f);
}

static void test_mixer_turn_and_saturation(void) {
    Mixer_Output_t m = mixer_mix(250.0f, 100.0f, 255, 20, 20);
    CHECK(m.left == 255);
    CHECK(m.right == 158);           /* 20 + 150 * 235/255 */

    m = mixer_mix(-250.0f, 100.0f, 255, 20, 20);
    CHECK(m.left == -158);
    CHECK(m.right == -255);
}

/* 300 once wrapped to 44 when narrowed to uint8_t: full power became low power */
static void test_mixer_saturates_before_narrowing(void) {
    Mixer_Output_t m = mixer_mix(255.0f, 45.0f, 255, 20, 20);
    CHECK(m.left == 255);
    CHECK(m.right == 213);           /* 20 + 210 * 235/255 */
}

static void test_mixer_deadband(void) {
    Mixer_Output_t m = mixer_mix(-5.0f, 0.0f, 255, 20, 20);
    CHECK(m.left == -24 && m.right == -24);         /* 20 + 5 * 235/255 */

    m = mixer_mix(0.4f, 0.0f, 255, 20, 20);         /* below one count: stays stopped */
    CHECK(m.left == 0 && m.right == 0);

    m = mixer_mix(1.0f, 0.0f, 255, 20, 20);         /* smallest command: the deadband */
    CHECK(m.left == 20 && m.right == 20);

    m = mixer_mix(255.0f, 0.0f, 255, 20, 20);       /* full command unchanged */
    CHECK(m.left == 255 && m.right == 255);

    m = mixer_mix(10.0f, 0.0f, 255, 46, 20);        /* each wheel its own deadband */
    CHECK(m.left == 54 && m.right == 29);
}

/* No jump and never decreasing: the old mapping raised 1..19 straight to 20 */
static void test_mixer_continuous(void) {
    int16_t previous = mixer_mix(1.0f, 0.0f, 255, 46, 46).left;
    for (int u = 2; u <= 255; u++) {
        int16_t now = mixer_mix((float)u, 0.0f, 255, 46, 46).left;
        CHECK(now >= previous && now - previous <= 2);
        previous = now;
    }
}

/* D from a measured rate: same P/I as pid_update(), D = kd * rate */
static void test_pid_rate_input(void) {
    PID_t a = default_pid(), b = default_pid();
    (void)pid_update(&a, 1.0f, DT);
    (void)pid_update_rate(&b, 1.0f, 50.0f, DT);
    CHECK_NEAR(b.p_term, a.p_term, 1e-5);
    CHECK_NEAR(b.i_term, a.i_term, 1e-6);
    CHECK_NEAR(b.d_term, 0.8 * 50.0, 1e-4);

    /* A setpoint jump moves the error but not the measured rate: no kick */
    PID_t c = default_pid();
    (void)pid_update_rate(&c, 0.0f, 0.0f, DT);
    (void)pid_update_rate(&c, 5.0f, 0.0f, DT);
    CHECK_NEAR(c.d_term, 0.0, 1e-6);
}

int main(void) {
    test_pid_worked_example();
    test_pid_output_clamp();
    test_pid_integral_clamp();
    test_pid_constant_error_has_no_derivative();
    test_pid_reset_keeps_gains();
    test_mixer_turn_and_saturation();
    test_mixer_saturates_before_narrowing();
    test_mixer_deadband();
    test_mixer_continuous();
    test_pid_rate_input();
    return UNIT_RESULT();
}
