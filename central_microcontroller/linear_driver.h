#ifndef kriz_linear
#define kriz_linear

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "math.h"

#define I2C_PORT i2c0
#define LINEAR_ADDR 0x09

#define SAFE_POS 50 // [mm]
#define LIN_Kp 5
#define LIN_Ki 0.2
#define LIN_Kd 0

struct linear{
    //IN
    uint8_t command;
    uint8_t speed;
    //OUT
    uint8_t state;
    uint8_t error;
    uint16_t height;
    uint16_t toGround;
    //INSIDE
    uint16_t goalHeight;
    float pid_prevError;
    float pid_integral;
};

int linear_read(struct linear* linear);
int linear_write(struct linear* linear);
void linear_init(struct linear* linear);

void linear_stop(struct linear* linear);
void linear_goto(struct linear* linear, float dt);
bool is_linear_stucked(struct linear* linear);
bool linear_reached_goal(struct linear* linear);
bool is_linear_home(struct linear* linear);
bool is_linear_safe(struct linear* linear);
float linear_step(struct linear* linear, float dt);









#endif