#ifndef kriz_motor
#define kriz_motor

#include <stdio.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"

struct motor {
	float torque;
	float torque_goal;
	//uint8_t direction;
	float torque_meas;
	uint8_t state;
	};

int motor_read(struct motor* motor);
int motor_write(struct motor* motor);
void motor_init(struct motor* motor);

void motor_stop(struct motor* motor);
void motor_left(struct motor* motor);
void motor_right(struct motor* motor);
bool is_motor_stucked(struct motor* motor);

float float_decode(uint16_t aNum);
uint16_t float_code(float aNum);


#endif