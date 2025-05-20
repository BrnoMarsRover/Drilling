/******************************************************************************
* @file     drill_status.h
 * @author  Martin Kriz
 * @brief   Header for drill status class
 * @date    2025-04-28
 *****************************************************************************/

#ifndef DRILL_STATUS_H
#define DRILL_STATUS_H

#include <cstdint>
#include <string>
#include <atomic>


class DrillStatus {
public:
    DrillStatus();
    ~DrillStatus();

    // Setter
    void setStatus(uint16_t status);

    // Motor getter
    std::string getMotorI2CStatus() const;
    std::string getMotorStucked() const;
    std::string getMotorError() const;
    bool isMotorStucked() const;

    // Linear getter
    std::string getLinearI2CStatus() const;
    std::string getLinearError() const;

    // Storage getter
    std::string getStorageI2CStatus() const;
    std::string getStorageScaleTared() const;
    std::string getStorageError() const;
    bool isStorageTared() const;


private:
    std::atomic<bool> motor_i2cStatus;
    std::atomic<bool> motor_stucked;
    std::atomic<int> motor_error;

    std::atomic<bool> linear_i2cStatus;
    std::atomic<int> linear_error;

    std::atomic<bool> storage_i2cStatus;
    std::atomic<bool> storage_scaleTared;
    std::atomic<int> storage_error;
};

#endif // DRILL_STATUS_H
