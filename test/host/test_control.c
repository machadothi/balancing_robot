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
    Mixer_Output_t m = mixer_mix(250.0f, 100.0f, 255, 20);
    CHECK(m.left == 255);
    CHECK(m.right == 150);

    m = mixer_mix(-250.0f, 100.0f, 255, 20);
    CHECK(m.left == -150);
    CHECK(m.right == -255);
}

/* 300 once wrapped to 44 when narrowed to uint8_t: full power became low power */
static void test_mixer_saturates_before_narrowing(void) {
    Mixer_Output_t m = mixer_mix(255.0f, 45.0f, 255, 20);
    CHECK(m.left == 255);
    CHECK(m.right == 210);
}

static void test_mixer_deadband(void) {
    Mixer_Output_t m = mixer_mix(-5.0f, 0.0f, 255, 20);
    CHECK(m.left == -20 && m.right == -20);

    m = mixer_mix(0.4f, 0.0f, 255, 20);     /* below one count: stays stopped */
    CHECK(m.left == 0 && m.right == 0);

    m = mixer_mix(30.0f, 0.0f, 255, 20);    /* above the deadband: unchanged */
    CHECK(m.left == 30 && m.right == 30);
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
    return UNIT_RESULT();
}
