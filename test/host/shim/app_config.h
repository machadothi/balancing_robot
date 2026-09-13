/* Host stand-in for the generated app_config.h: every optional command on */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define BOARD_NAME                  "host"
#define APP_BLINK_ONLY              0
#define CONSOLE_USB                 1
#define CONSOLE_BT                  0
#define CONSOLE_ANY                 1
#define CONSOLE_ECHO                0
#define TELEMETRY                   1
#define LOGGING                     0
#define AT_CMD_HELP                 1
#define AT_CMD_ALL_QUERY            1
#define AT_CMD_PID_TOGGLE           1
#define IMU_SAMPLE_RATE_MS          10
#define ATTITUDE_FILTER_COMPLEMENTARY 1
#define ATTITUDE_FILTER_KALMAN      0

#endif // APP_CONFIG_H
