#ifndef kriz_motor
#define kriz_motor

#include <stdio.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"

struct motor {
	//IN
	int8_t rps;
	//OUT
	int8_t rpsMeas;
	int8_t torqueMeas;
	uint8_t state;
	uint8_t temperature;
	uint8_t error;
	//INSIDE
	int8_t rpsGoal;
	bool stucked;
	};

int motor_read(struct motor* motor);
int motor_write(struct motor* motor);
void motor_init(struct motor* motor);

void motor_stop(struct motor* motor);
void motor_left(struct motor* motor);
void motor_right(struct motor* motor);
void motor_unblock(struct motor* motor);
bool is_motor_stucked(struct motor* motor);

#endif