/******************************************************************************
* @file     drill_status.cpp
 * @author  Martin Kriz
 * @brief   Class for handling drill status register
 * @date    2025-04-28
 *****************************************************************************/

#include "drill/drill_status.h"

DrillStatus::DrillStatus()
    : last_received_time_(std::chrono::steady_clock::now()),
      motor_i2cStatus(false),
      motor_stucked(false),
      motor_error(0),
      linear_i2cStatus(false),
      linear_error(0),
      storage_i2cStatus(false),
      storage_scaleTared(false),
      storage_error(0) {}

DrillStatus::~DrillStatus() = default;

void DrillStatus::setStatus(const uint16_t status) {
    last_received_time_ = std::chrono::steady_clock::now();

    motor_error      = status & 0b0000000000000011;
    motor_stucked    = (status >> 2) & 0b1;
    motor_i2cStatus  = (status >> 3) & 0b1;

    linear_error     = (status >> 4) & 0b00000111;
    linear_i2cStatus = (status >> 7) & 0b1;

    storage_error      = (status >> 8) & 0b00000011;
    storage_scaleTared = (status >> 10) & 0b1;
    storage_i2cStatus  = (status >> 11) & 0b1;
}

bool DrillStatus::is_drill_connected() const
{
    constexpr std::chrono::seconds kTimeout(2);
    if (const auto now = std::chrono::steady_clock::now(); (now - last_received_time_) < kTimeout)
        return true;
    return false;
}

std::string DrillStatus::getMotorI2CStatus() const {
    return motor_i2cStatus ? "CONNECTED" : "DISCONNECTED";
}

std::string DrillStatus::getMotorStucked() const {
    if (!motor_i2cStatus) return "-";
    return motor_stucked ? "BLOCKED" : "FREE";
}

std::string DrillStatus::getMotorError() const {
    if (!motor_i2cStatus) return "-";
    switch (motor_error) {
        case 0: return "NO ERROR";
        case 1: return "H-BRIDGE FAILED";
        default: return "UNKNOWN";
    }
}

std::string DrillStatus::getLinearI2CStatus() const {
    return linear_i2cStatus ? "CONNECTED" : "DISCONNECTED";
}

std::string DrillStatus::getLinearError() const {
    if (!linear_i2cStatus) return "-";
    switch (linear_error) {
        case 0: return "NO ERROR";
        case 1: return "COMMUNICATION FAILED";
        case 2: return "UNKNOWN COMMAND";
        case 3: return "MOTOR DE-SYNCHRONIZATION";
        case 4: return "MOTOR OVERWEIGHT";
        case 5: return "MOTOR FAILFED";
        default: return "UNKNOWN";
    }
}

std::string DrillStatus::getStorageI2CStatus() const {
    return storage_i2cStatus ? "CONNECTED" : "DISCONNECTED";
}

std::string DrillStatus::getStorageScaleTared() const {
    if (!storage_i2cStatus) return "-";
    return storage_scaleTared ? "TARED" : "NOT TARED";
}

std::string DrillStatus::getStorageError() const {
    if (!storage_i2cStatus) return "-";
    switch (storage_error) {
        case 0: return "NO ERROR";
        case 1: return "MAX WEIGHT EXCEEDED";
        case 2: return "UNKNOWN COMMAND";
        case 3: return "SLOT NOT FOUND";
        default: return "UNKNOWN";
    }
}

bool DrillStatus::isMotorStucked() const {
    return motor_stucked;
}

bool DrillStatus::isStorageTared() const {
    return storage_scaleTared;
}

