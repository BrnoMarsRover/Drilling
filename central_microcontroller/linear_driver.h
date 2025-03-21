#ifndef kriz_linear
#define kriz_linear

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define LINEAR_ADDR 0x09

#define SAFE_POS 50 // [mm]

struct linear{
    //IN
    uint8_t command;
    uint8_t speed;
    //OUT
    uint8_t state;
    uint8_t error;
    uint16_t height;
    uint16_t toGround; // implementovat !!
    //INSIDE
    uint16_t goalHeight;
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