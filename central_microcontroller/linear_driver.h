#ifndef kriz_linear
#define kriz_linear

#include <stdio.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"

struct linear{
    uint8_t command;
    uint8_t speed;
    uint8_t states;
    uint16_t height;
    uint16_t toGround; // implementovat !!
    uint16_t goal_height;
};

int linear_read(struct linear* linear);
int linear_write(struct linear* linear);
void linear_init(struct linear* linear);

void linear_stop(struct linear* linear);
void linear_go_down(struct linear* linear);
void linear_go_up(struct linear* linear);
bool is_linear_stucked(struct linear* linear);
bool linear_reached_goal(struct linear* linear);
bool is_linear_home(struct linear* linear);
bool is_linear_safe(struct linear* linear);








#endif