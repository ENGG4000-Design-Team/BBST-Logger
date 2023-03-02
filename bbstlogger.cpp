/**
 * bbstlogger.cpp
 * Author: Ethan Garnier
 * Data logging application for the
 * Balloon Borne Solar Telescope (BBST)
 */
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <ctime>

#include <opencv2/opencv.hpp>
#include "cmps14.hpp"
#include <wiringPi.h>
#include <wiringSerial.h>

const std::string LOGFILE = "log.csv";
std::mutex m_logfile;

// Write a vector of data to logfile as a comma separated line
// with timestamp. vector holds strings, this might suck so review this.
void logfileWrite(const std::vector<std::string> &data)
{
    // Lock the logfile mutex to ensure only one thread is
    // writing to the logfile at a time. lock is automatically
    // released when we leave the scope of function
    const std::lock_guard<std::mutex> lock(m_logfile);

    // Open logfile for append operation
    std::ofstream logfile(LOGFILE, std::ios::app);

    // Get current timestamp from system clock
    std::time_t timestamp = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());

    std::string timestampStr = std::ctime(&timestamp);

    // Start writing to logfile

    // Goofy substring to remove newline at end of logfile
    logfile << timestampStr.substr(0, timestampStr.length() - 1) << ", ";

    for (const auto &x : data)
        logfile << x << ",";

    logfile << "\n";

    logfile.close();
}

void IMUThread()
{
    std::vector<std::string> data(3);

    // Initiate IMU over serial connection
    auto imu = new cmps14(false);
    if (imu->begin() == -1)
        return;

    // Read IMU data and write to log file
    float heading = 0.0f, pitch = 0.0f, roll = 0.0f;
    while (1)
    {
        data[0] = "Heading: " + std::to_string(imu->getHeading());
        data[1] = "Pitch: " + std::to_string(imu->getPitch());
        data[2] = "Roll: " + std::to_string(imu->getRoll());
        
        logfileWrite(data);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void photodiodeThread()
{
    std::vector<std::string> data{"d7", "d4", "d1"};
    logfileWrite(data);
}

void imgProcThread()
{
    std::vector<std::string> data{"to the right", "below", "up"};
    logfileWrite(data);
}

int main()
{
    // Launch threads
    std::thread t_imu(IMUThread);
    std::thread t_photodiode(photodiodeThread);
    std::thread t_imgProc(imgProcThread);

    // Join threads
    t_imu.join();
    t_photodiode.join();
    t_imgProc.join();

    return 1;
}
