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
#include <cmps14.hpp>
#include <wiringPi.h>
#include <wiringSerial.h>

#include "SolarPosition.h"

const std::string LOGFILE = "log.csv";
std::mutex m_logfile;

// Structure that represents the bytes being sent to the
// motor controllers from IMU data. This data is the heading correction
// angle, the pitch correction angle, and the roll angle.
struct
{
    union
    {
        float headingCorr;
        uint8_t headingCorrBuff[sizeof(float)];
    };
    union
    {
        float pitchCorr;
        uint8_t pitchCorrBuff[sizeof(float)];
    };
    union
    {
        float roll;
        uint8_t rollBuff[sizeof(float)];
    };
} IMUComm;

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
    // Initiate IMU over serial connection
    auto imu = new cmps14(false);
    if (imu->begin() == -1)
    {
        std::cerr << "Unable to initialize CMPS14 IMU over serial communication..." << std::endl;
        return;
    }

    // Initialize serial communication to the motor controllers
    int controllerFd = serialOpen("/dev/ttyACM0", 115200);
    if (controllerFd == -1)
    {
        std::cerr << "Unable to initialize serial communication with gimbal controller. Please check serial port being used..." << std::endl;
        return;
    }

    // Initialize WiringPi
    if (wiringPiSetup() == -1)
    {
        serialClose(controllerFd);
        std::cerr << "Unable to start WiringPi" << std::endl;
        return;
    }

    // Main IMU loop
    int i;
    float azimuth = 0.0f, elevation = 0.0f;
    float latitude = 45.944962483507844;
    float longitude = -66.64841312244609;
    std::vector<std::string> data(3);
    while (1)
    {
        // Calculate location of Sun at current point in time
        calcSunPos(elevation, azimuth, longitude, latitude);

        // Read data from IMU and store in IMUComm structure
        IMUComm.headingCorr = sunPos.azimuth - imu->getHeading();
        IMUComm.pitchCorr = sunPos.elevation - imu->getPitch();
        IMUComm.roll = imu->getRoll();

        // Send data to motor controller by simpling sending the heading,
        // pitch, and roll byte by byte. Each float is separated by a comma,
        // and data transmission ends with a new line character.
        for (i = 0; i < sizeof(float); i++)
            serialPutchar(controllerFd, IMUComm.headingCorrBuff[i]);

        serialPutchar(controllerFd, ',');

        for (i = 0; i < sizeof(float); i++)
            serialPutchar(controllerFd, IMUComm.pitchCorrBuff[i]);

        serialPutchar(controllerFd, ',');

        for (i = 0; i < sizeof(float); i++)
            serialPutchar(controllerFd, IMUComm.rollBuff[i]);

        serialPutchar(controllerFd, ',');

        // Generate the data array to send to log file
        data[0] = "Heading Correction: " + std::to_string(IMUComm.headingCorr);
        data[1] = "Pitch Correction: " + std::to_string(IMUComm.pitchCorr);
        data[2] = "Roll: " + std::to_string(IMUComm.roll);

        // Log data to logfile
        logfileWrite(data);

        // TODO: Experimentally determine lower limit on delay between loops
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // TODO: Need to close this in a signal handler since
    // this is never reached...
    serialClose(controllerFd);
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
    //std::thread t_photodiode(photodiodeThread);
    //std::thread t_imgProc(imgProcThread);

    // Join threads
    t_imu.join();
    //t_photodiode.join();
    //t_imgProc.join();

    return 1;
}
