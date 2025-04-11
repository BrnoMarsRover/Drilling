//
// Created by martin on 14.03.25.
//

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

    // Tisk razitka
    void logActionStamp(int option);

    // Tisk dat z vrtani
    void logDrillSampleData(float actualTorque, float actualRPS, int motorTemperature, int actualHeight);
    void logDrillSampleResult(int depth);

    void logStoreSampleResult(int slot, float weight);

    void logDrillCalibration(bool weightReset);

private:
    std::string file_name_;
    std::ofstream file_;
    // Pomocná metoda pro získání formátovaného názvu souboru
    void generateFilename();
    static std::string generateTimestamp();
};


#endif //DRILL_LOGGER_H
