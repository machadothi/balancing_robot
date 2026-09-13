/**
 * @file test_at.c
 * @brief AT parser and robot command table, against stubbed UART, motors and lock
 *
 * The same protocol cases as test/test_console.py, without hardware.
 */

#include <stdbool.h>

#include "cmd/at_cmd.h"
#include "motor/motor.h"
#include "robot/robot_internal.h"
#include "unit.h"

/* ==========================================================================
 * Stubs
 * ========================================================================== */

Robot_t robot = {
    .pid = { .kp = 25.0f, .ki = 0.5f, .kd = 0.8f, .integral_limit = 100.0f, .output_limit = 255.0f },
    .pid_enabled = true,
};

static char out[4096];
static int16_t wheel[MOTOR_COUNT];
static bool standby = true;
static bool streaming = false;

UART_Status_t uart_puts(UART_Port_t port, const char *s) {
    (void)port;
    strncat(out, s, sizeof(out) - strlen(out) - 1);
    return UART_OK;
}

void uart_set_rx_callback(UART_RxCallback_t callback) { (void)callback; }
void motor_set(Motor_Id_t id, int16_t command) { wheel[id] = command; }
void motor_standby(bool enable) { standby = enable; }
void robot_lock(void) {}
void robot_unlock(void) {}

void robot_enable(void) {
    pid_reset(&robot.pid);
    robot.motors_enabled = true;
    motor_standby(false);
}

void robot_disable(void) {
    robot.motors_enabled = false;
    motor_set(MOTOR_LEFT, 0);
    motor_set(MOTOR_RIGHT, 0);
    motor_standby(true);
    pid_reset(&robot.pid);
}

void robot_restore_defaults(void) {
    robot.pid.kp = 25.0f;
    robot.pid.ki = 0.5f;
    robot.pid.kd = 0.8f;
    robot.target_velocity = 0.0f;
    robot.turn_rate = 0.0f;
}

/* Stands in for telemetry.c's table (integer 0..1) */
static AT_Result_t set_stream(const float *values) {
    streaming = (values[0] != 0.0f);
    return AT_OK;
}

static const AT_Command_Def_t stream_commands[] = {
    { .name = "STREAM", .set = set_stream, .params = 1, .integer = true, .min = 0.0f, .max = 1.0f },
};

/* ==========================================================================
 * Helpers
 * ========================================================================== */

static void send(const char *line) {
    out[0] = '\0';
    at_cmd_process(UART_PORT_USB, line, (uint16_t)strlen(line));
}

/** The last line before the prompt; every reply must end with one */
static const char *last_line(void) {
    static char line[256];
    size_t n = strlen(out);

    if (n < 4 || strcmp(out + n - 4, "\r\n> ") != 0) {
        return "<no prompt>";
    }
    out[n - 4] = '\0';
    const char *start = strrchr(out, '\n');
    snprintf(line, sizeof(line), "%s", start ? start + 1 : out);
    out[n - 4] = '\r';
    return line;
}

#define EXPECT(cmd, want)                                                       \
    do {                                                                        \
        send(cmd);                                                              \
        const char *got_ = last_line();                                         \
        if (strcmp(got_, want) != 0) {                                          \
            unit_failures++;                                                    \
            printf("%s:%d: %s -> \"%s\", expected \"%s\"\n",                    \
                   __FILE__, __LINE__, cmd, got_, want);                        \
        }                                                                       \
    } while (0)

/* ==========================================================================
 * Tests
 * ========================================================================== */

static void test_syntax(void) {
    EXPECT("AT", "OK");
    EXPECT("AT+VERSION?", "+VERSION:1.0.0");
    EXPECT("at+version?", "+VERSION:1.0.0");
    EXPECT("  AT+VERSION?", "+VERSION:1.0.0");
    EXPECT("HELLO", "ERROR:Invalid command (must start with AT)");
    EXPECT("AT+", "ERROR:Invalid syntax (use AT+CMD?, AT+CMD=val, or AT+CMD)");
    EXPECT("ATX", "ERROR:Invalid syntax (use AT+CMD?, AT+CMD=val, or AT+CMD)");
}

static void test_error_codes(void) {
    static const char *const cases[][2] = {
        { "AT+SAVE", "ERROR:1" },           { "AT+LOAD", "ERROR:1" },
        { "AT+NOSUCH?", "ERROR:2" },        { "AT+NOSUCH", "ERROR:2" },
        { "AT+NOSUCH=1", "ERROR:2" },       { "AT+ENABLE?", "ERROR:2" },
        { "AT+KP", "ERROR:2" },             { "AT+KP=", "ERROR:3" },
        { "AT+KP=abc", "ERROR:3" },         { "AT+KP=1.5x", "ERROR:3" },
        { "AT+KP=nan", "ERROR:3" },         { "AT+KP=inf", "ERROR:3" },
        { "AT+SPEED=10", "ERROR:3" },       { "AT+SPEED=abc,10", "ERROR:3" },
        { "AT+SPEED=10,nan", "ERROR:3" },   { "AT+SPEED=1,2,3", "ERROR:3" },
        { "AT+KP=-1", "ERROR:4" },          { "AT+TURN=100.5", "ERROR:4" },
        { "AT+VELOCITY=-101", "ERROR:4" },  { "AT+SPEED=0,101", "ERROR:4" },
        { "AT+STREAM=2", "ERROR:4" },       { "AT+STREAM=0.5", "ERROR:4" },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        EXPECT(cases[i][0], cases[i][1]);
    }
}

static void test_gains(void) {
    EXPECT("AT+KP=nan", "ERROR:3");
    EXPECT("AT+KP?", "+KP:25.0000");        /* a rejected value changes nothing */
    EXPECT("AT+KP=12.3456", "OK");
    EXPECT("AT+KP?", "+KP:12.3456");
    EXPECT("AT+KI=12.3456", "OK");
    EXPECT("AT+KI?", "+KI:12.3456");
    EXPECT("AT+DEFAULT", "OK");
    EXPECT("AT+KP?", "+KP:25.0000");
    EXPECT("AT+KI?", "+KI:0.5000");
    EXPECT("AT+KD?", "+KD:0.8000");
}

static void test_values(void) {
    EXPECT("AT+TURN=-0.004", "OK");
    EXPECT("AT+TURN?", "+TURN:0.00");
    EXPECT("AT+TURN=-99.999", "OK");
    EXPECT("AT+TURN?", "+TURN:-100.00");
    EXPECT("AT+TARGET=-42.5", "OK");
    EXPECT("AT+TARGET?", "+TARGET:-42.50");
    EXPECT("AT+VELOCITY?", "+VELOCITY:0.00");
    EXPECT("AT+ALL?", "+ALL:0.000,0.000,0.000,0.000,0.000,0.000,0.00");
    EXPECT("AT+STREAM=1", "OK");
    CHECK(streaming);
}

static void test_motor_states(void) {
    EXPECT("AT+STATUS?", "+STATUS:DISABLED,UNBALANCED");
    EXPECT("AT+PIDOFF", "OK");
    EXPECT("AT+ENABLE", "OK");
    EXPECT("AT+STATUS?", "+STATUS:ENABLED,UNBALANCED");
    CHECK(!standby);

    EXPECT("AT+SPEED=30,-30", "OK");
    EXPECT("AT+SPEED?", "+SPEED:30.0,-30.0");
    CHECK(wheel[MOTOR_LEFT] == 76 && wheel[MOTOR_RIGHT] == -76);

    EXPECT("AT+STOP", "OK");
    EXPECT("AT+SPEED?", "+SPEED:0.0,0.0");
    EXPECT("AT+STATUS?", "+STATUS:DISABLED,UNBALANCED");
    CHECK(wheel[MOTOR_LEFT] == 0 && wheel[MOTOR_RIGHT] == 0 && standby);

    EXPECT("AT+PIDON", "OK");
    CHECK(robot.pid_enabled);
    EXPECT("AT+PID", "OK");
    CHECK(!robot.pid_enabled);
}

static void test_help(void) {
    send("AT+HELP");
    int lines = 0;
    for (const char *p = out; (p = strstr(p, "  AT+")) != NULL; p++) {
        lines++;
    }
    CHECK(lines >= 25);
    CHECK(strstr(out, "AT+SPEED?/=a,b") != NULL);
    CHECK(strstr(out, "AT+KP?/=n") != NULL);
    CHECK_STR(last_line(), "OK");
}

int main(void) {
    at_cmd_init();
    robot_commands_register();
    at_cmd_register(stream_commands, sizeof(stream_commands) / sizeof(stream_commands[0]));

    test_syntax();
    test_error_codes();
    test_gains();
    test_values();
    test_motor_states();
    test_help();
    return UNIT_RESULT();
}
