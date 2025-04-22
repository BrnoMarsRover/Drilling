//
// Created by martin on 22.04.25.
//

#include "drill/drill_status.h"

DrillStatus::DrillStatus()
    : motor_i2cStatus(false),
      motor_stucked(false),
      motor_error(0),
      linear_i2cStatus(false),
      linear_error(0),
      storage_i2cStatus(false),
      storage_scaleTared(false),
      storage_error(0) {}

DrillStatus::~DrillStatus() = default;

void DrillStatus::setStatus(const uint16_t status) {
    motor_error      = status & 0b0000000000000011;
    motor_stucked    = (status >> 2) & 0b1;
    motor_i2cStatus  = (status >> 3) & 0b1;

    linear_error     = (status >> 4) & 0b00000111;
    linear_i2cStatus = (status >> 7) & 0b1;

    storage_error      = (status >> 8) & 0b00000011;
    storage_scaleTared = (status >> 10) & 0b1;
    storage_i2cStatus  = (status >> 11) & 0b1;
}

std::string DrillStatus::getMotorI2CStatus() const {
    return motor_i2cStatus ? "OK" : "ERROR";
}

std::string DrillStatus::getMotorStucked() const {
    return motor_stucked ? "BLOCKED" : "FREE";
}

std::string DrillStatus::getMotorError() const {
    switch (motor_error) {
        case 0: return "NO ERROR";
        case 1: return "H-BRIDGE FAILED";
        default: return "UNKNOWN";
    }
}

std::string DrillStatus::getLinearI2CStatus() const {
    return linear_i2cStatus ? "OK" : "ERROR";
}

std::string DrillStatus::getLinearError() const {
    switch (linear_error) {
        case 0: return "NO ERROR";
        case 1: return "SLIMA1";
        case 2: return "SLIMA2";
        case 3: return "SLIMA3";
        default: return "UNKNOWN";
    }
}

std::string DrillStatus::getStorageI2CStatus() const {
    return storage_i2cStatus ? "OK" : "ERROR";
}

std::string DrillStatus::getStorageScaleTared() const {
    return storage_scaleTared ? "TARED" : "NOT TARED";
}

std::string DrillStatus::getStorageError() const {
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

