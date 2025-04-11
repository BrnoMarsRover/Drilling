//
// Created by martin on 14.03.25.
//

#include <drill/drill_logger.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>

DrillLogger::DrillLogger()
{
    generateFilename();
    file_.open(file_name_, std::ios::app); // Otevření v režimu přidávání dat

    if (!file_.is_open()) {
        std::cerr << "Chyba: Nelze otevřít soubor: " << file_name_ << std::endl;
    } else {
        file_ << "Drilling record" << std::endl << std::endl;
        std::cout << "Soubor otevřen: " << file_name_ << std::endl;
    }
}

DrillLogger::~DrillLogger()
{
    if (file_.is_open()) {
        file_.close();
        std::cout << "Soubor uzavřen: " << file_name_ << std::endl;
    }
}


void DrillLogger::logActionStamp(const int option)
{
    // Výběr zprávy podle volby option
    std::string message;
    switch (option)
    {
    case 1:
        message = "Action DrillSample";
        break;
    case 2:
        message = "Action StoreSample";
        break;
    default:
        message = "DrillCalibration";
        break;
    }

    // Zápis do souboru s razítkem
    file_ << generateTimestamp() << message << std::endl;

    std::cout << "Akce zapsána: " << message << " do souboru: " << file_name_ << std::endl;

}

void DrillLogger::logDrillSampleData(const float actualTorque, const float actualRPS, const int motorTemperature, const int actualHeight)
{
    file_ << actualTorque << "," << actualRPS << "," << motorTemperature << "," << actualHeight << ";";
}

void DrillLogger::logDrillSampleResult(const int depth)
{
    file_ << std::endl;
    file_ << "Reached depth: " << depth << std::endl;
    file_ << std::endl;
}

void DrillLogger::logStoreSampleResult(const int slot, const float weight)
{
    file_ << "Slot: " << slot << ", weight: " << weight << std::endl;
    file_ << std::endl;
}

void DrillLogger::logDrillCalibration(const bool weightReset)
{
    file_ << "Weights have been reset: " << weightReset << std::endl;
    file_ << std::endl;
}

void DrillLogger::generateFilename()
{
    // Získání aktuálního času a formátování názvu souboru
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_time);

    // Formátování názvu souboru
    std::ostringstream filename;
    filename << "drillData_"
             << (now_tm->tm_year + 1900) << "-"
             << (now_tm->tm_mon + 1) << "-"
             << now_tm->tm_mday << "_"
             << now_tm->tm_hour << "-"
             << now_tm->tm_min << "-"
             << now_tm->tm_sec << ".txt";

    file_name_ = filename.str();
}

std::string DrillLogger::generateTimestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_time);

    std::ostringstream timestamp;
    timestamp << "["
              << (now_tm->tm_year + 1900) << "-"
              << (now_tm->tm_mon + 1) << "-"
              << now_tm->tm_mday << " "
              << now_tm->tm_hour << ":"
              << now_tm->tm_min << ":"
              << now_tm->tm_sec << "] ";

    return timestamp.str();
}
