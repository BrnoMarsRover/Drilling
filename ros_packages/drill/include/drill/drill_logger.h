/******************************************************************************
* @file     drill_logger.h
 * @author  Martin Kriz
 * @brief   Header for drill logger class
 * @date    2025-04-28
 *****************************************************************************/

#ifndef DRILL_LOGGER_H
#define DRILL_LOGGER_H

#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>

class DrillLogger
{
public:
    DrillLogger();
    ~DrillLogger();

    // Log stamp
    void logActionStamp(int option);

    // Log data
    void logDrillSampleData(float actualTorque, float actualRPS, int motorTemperature, int actualHeight);
    void logDrillSampleResult(int depth);
    void logStoreSampleResult(int slot, float weight);
    void logDrillCalibration(bool weightReset);

private:
    // File
    std::string file_name_;
    std::ofstream file_;

    // Function to get file name
    void generateFilename();

    // Function to generate time stamp
    static std::string generateTimestamp();
};


#endif //DRILL_LOGGER_H
