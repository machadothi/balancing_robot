#ifndef __MOTOR_H__
#define __MOTOR_H__

// Chip control pins
#define TB6612_STBY     GPIO4 // B
#define TB6612_AIN1     GPIO3 // B
#define TB6612_AIN2     GPIO8 // A
#define TB6612_BIN1     GPIO5 // B
#define TB6612_BIN2     GPIO8 // B

#define TB6612_PWMA     GPIO1 // B
#define TB6612_PWMB     GPIO0 // B

// Motor encoder controls pins
#define MOTOR1          GPIO5 // A
#define MOTOR2          GPIO6 // A

void motor_init(void);
void motor1_set_direction(bool direction);
void motor2_set_direction(bool direction);
void motor1_set_speed(uint8_t speed);
void motor2_set_speed(uint8_t speed);

#endif // __MOTOR_H__
