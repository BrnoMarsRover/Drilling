/******************************************************************************
 * @file    motor_driver.h
 * @author  Martin Kriz
 * @brief   Header file for the DC motor subsystem functions and definitions.
 * @date    2025-04-26
 ******************************************************************************/

#ifndef kriz_motor
#define kriz_motor

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

// I2C adress
#define I2C_PORT i2c0
#define MOTOR_ADDR 0x0A

// Hardcoded deadtime based on measurement
#define MAX_DEAD_TICKS 100000 // max dead time in ms/loop time ms e.g. 3000/100

/**
 * @brief   Structure representing the DC motor subsystem.
 */
struct motor {
	//PICO -> MOTOR
	int8_t rps;
	//PICO <- MOTOR
	int8_t rpsMeas;
	int8_t torqueMeas;
	uint8_t state;
	uint8_t temperature;
	uint8_t error;
	//PICO
	uint8_t i2cStatus;
	int8_t rpsGoal;
	uint8_t deadTicks;
	bool running;
	bool stucked;
	};

/**
 * @brief   Reads motor subsystem data over I2C.
 * @param   motor Pointer to the motor structure.
 * @return  0 on success, -1 on I2C error, -2 if motor pointer is NULL.
 */
int motor_read(struct motor* motor);

/**
 * @brief   Writes motor command over I2C.
 * @param   motor Pointer to the motor structure.
 * @return  0 on success, -1 on I2C error, -2 if motor pointer is NULL.
 */
int motor_write(struct motor* motor);

/**
 * @brief   Initializes the motor structure.
 * @param   motor Pointer to the motor structure.
 */
void motor_init(struct motor* motor);

/**
 * @brief   Stops the motor by setting speed to zero.
 * @param   motor Pointer to the motor structure.
 */
void motor_stop(struct motor* motor);

/**
 * @brief   Handles motor rotation to the left while checking overload.
 * @param   motor Pointer to the motor structure.
 */
void motor_left(struct motor* motor);

/**
 * @brief   Handles motor rotation to the right.
 * @param   motor Pointer to the motor structure.
 */
void motor_right(struct motor* motor);

/**
 * @brief   Clears the motor's stucked state and resets deadTicks.
 * @param   motor Pointer to the motor structure.
 */
void motor_unblock(struct motor* motor);

/**
 * @brief   Checks if the motor is stuck based on state and deadTicks counter.
 * @param   motor Pointer to the motor structure.
 * @return  true if the motor is stuck, false otherwise.
 */
bool is_motor_stucked(struct motor* motor);

/**
 * @brief   Calculate the rps error, primary for adjustable speed of linear actuator.
 * @param   motor Pointer to the motor structure.
 * @return  Error (goal - meas).
 */
float get_rps_error(struct motor* motor);

#endif