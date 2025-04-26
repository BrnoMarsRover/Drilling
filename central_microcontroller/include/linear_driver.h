/******************************************************************************
 * @file    linear_driver.h
 * @author  Martin Kriz
 * @brief   Definitions and functions for controlling a linear actuator over I2C.
 * @date    2025-04-26
 ******************************************************************************/

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

#define SAFE_POS 58 // [mm]
#define STORE_POS 92 // [mm]
#define LIN_Kp 15

#define MAX_OUTPUT  255.0f
#define MIN_OUTPUT -255.0f
#define MIN_SPEED 40.0f

struct linear{
    //In
    uint8_t command;
    uint8_t speed;
    //Output
    uint8_t state;
    uint8_t error;
    uint16_t height;
    uint16_t toGround;
    //Internal
    uint8_t i2cStatus;
    uint16_t goalHeight;
};

/**
 * @brief   Reads linear actuator sensor data over I2C.
 * @param   linear Pointer to the linear actuator structure.
 * @return  0 on success, negative value on failure.
 */
int linear_read(struct linear* linear);

/**
 * @brief   Sends control commands to the linear actuator via I2C.
 * @param   linear Pointer to the linear actuator structure.
 * @return  0 on success, negative value on failure.
 */
int linear_write(struct linear* linear);

/**
 * @brief   Initializes the linear actuator structure and sets default values.
 * @param   linear Pointer to the linear actuator structure.
 */
void linear_init(struct linear* linear);

/**
 * @brief   Stops linear actuator movement.
 * @param   linear Pointer to the linear actuator structure.
 */
void linear_stop(struct linear* linear);

/**
 * @brief   Moves the linear actuator towards the goal height.
 * @param   linear Pointer to the linear actuator structure.
 * @param   dt Time delta for control calculation.
 */
void linear_goto(struct linear* linear, float dt);

/**
 * @brief   Checks if the linear actuator is stuck based on error codes.
 * @param   linear Pointer to the linear actuator structure.
 * @return  true if stuck, false otherwise.
 */
bool is_linear_stucked(struct linear* linear);

/**
 * @brief   Checks if the linear actuator has reached the goal height.
 * @param   linear Pointer to the linear actuator structure.
 * @return  true if the goal is reached, false otherwise.
 */
bool linear_reached_goal(struct linear* linear);

/**
 * @brief   Checks if the linear actuator is at its home (bottom) position.
 * @param   linear Pointer to the linear actuator structure.
 * @return  true if at home position, false otherwise.
 */
bool is_linear_home(struct linear* linear);

/**
 * @brief   Checks if it is safe to move the storage system.
 * @param   linear Pointer to the linear actuator structure.
 * @return  true if safe to move, false otherwise.
 */
bool can_storage_move(struct linear* linear);

/**
 * @brief   Checks if the linear actuator can continue moving downwards.
 * @param   linear Pointer to the linear actuator structure.
 * @return  true if movement down is allowed, false otherwise.
 */
bool can_linear_goDown(struct linear* linear);

/**
 * @brief   Computes the control output for actuator movement based on goal error.
 * @param   linear Pointer to the linear actuator structure.
 * @param   dt Time delta for control update.
 * @return  Control output (speed command).
 */
float linear_step(struct linear* linear, float dt);

#endif