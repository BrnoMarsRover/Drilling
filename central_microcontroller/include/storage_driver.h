/******************************************************************************
 * @file    storage_driver.h
 * @author  Martin Kriz
 * @brief   Definitions and functions for controlling a storage unit over I2C.
 * @date    2025-04-26
 ******************************************************************************/

#ifndef kriz_246877
#define kriz_246877

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define STORAGE_ADDR 0x08

#define STORE_SLOTS 4
#define DEF_POS 0


struct storage{
    //Input
    uint8_t command;
    //Output
    uint16_t weight;
    uint8_t active_slot;
    uint8_t error;
    bool scaleTared;
    bool active;
    //Internal
    bool i2cStatus;
    uint8_t demand_pos;
    bool weight_recieved;
    bool weighting;
    uint16_t samples[STORE_SLOTS];
};

/**
 * @brief   Reads the status and data from the storage unit over I2C.
 * @param   storage Pointer to the storage structure.
 * @return  0 on success, negative value on failure.
 */
int storage_read(struct storage* storage);

/**
 * @brief   Sends a control command to the storage unit over I2C.
 * @param   storage Pointer to the storage structure.
 * @return  0 on success, negative value on failure.
 */
int storage_write(struct storage* storage);

/**
 * @brief   Initializes the storage structure and resets its internal state.
 * @param   storage Pointer to the storage structure.
 */
void storage_init(struct storage* storage);

/**
 * @brief   Commands the storage to move to a specified position.
 * @param   storage Pointer to the storage structure.
 * @param   pos Desired storage slot position.
 */
void storage_goto(struct storage *storage, uint8_t pos);

/**
 * @brief   Initiates or completes the weight measurement process.
 * @param   storage Pointer to the storage structure.
 */
void storage_get_weight(struct storage *storage);

/**
 * @brief   Commands the storage unit to hold its current position.
 * @param   storage Pointer to the storage structure.
 */
void storage_hold(struct storage *storage);

/**
 * @brief   Resets all stored weight samples to zero.
 * @param   storage Pointer to the storage structure.
 */
void storage_wreset(struct storage *storage);

/**
 * @brief   Checks if the storage unit is in a default, safe position.
 * @param   storage Pointer to the storage structure.
 * @return  true if storage is OK, false otherwise.
 */
bool is_storage_ok(struct storage *storage);

/**
 * @brief   Commands a full reset of the storage unit.
 * @param   storage Pointer to the storage structure.
 */
void storage_reset(struct storage *storage);

/**
 * @brief   Commands the storage unit to tare (zero) its scale.
 * @param   storage Pointer to the storage structure.
 */
void storage_get_tared(struct storage *storage);

#endif
