/**
 * bbstlogger.cpp
 * Author: Ethan Garnier
 * Data logging application for the
 * Balloon Borne Solar Telescope (BBST)
 */
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <map>
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

#include "Adafruit_ADS1015.h"

#include "sun_pos.h"

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
        IMUComm.headingCorr = azimuth - imu->getHeading();
        IMUComm.pitchCorr = elevation - imu->getPitch();
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
        // logfileWrite(data);

        // TODO: Experimentally determine lower limit on delay between loops
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // TODO: Need to close this in a signal handler since
    // this is never reached...
    serialClose(controllerFd);
}

void photodiodeThread()
{
    const int PHOTODIODE_ARRAY_X = 5;
    const int PHOTODIODE_ARRAY_Y = 5;

    // GPIO pins we are using to control multiplexer channels
    int pins[4] = {1, 2, 3, 4};

    // Map the multiplexer channel bits to the corresponding
    // index on the photodiodes array below.
    std::map<uint8_t, std::vector<int>> photodiodeIdx{
        {0x00, {0, 2}}, // 0b0000 corresponds to photodiode D1
        {0x01, {1, 3}}, // 0b0001 corresponds to photodiode D2
        {0x02, {2, 4}}, // 0b0010 corresponds to photodiode D3
        {0x03, {3, 3}}, // 0b0011 corresponds to photodiode D4
        {0x04, {4, 2}}, // 0b0100 corresponds to photodiode D5
        {0x05, {3, 1}}, // 0b0101 corresponds to photodiode D6
        {0x0A, {2, 0}}, // 0b1010 corresponds to photodiode D7
        {0x0B, {1, 1}}, // 0b1011 corresponds to photodiode D8
    };

    // Store photodiode readings in this array such that their
    // index mimics their physical position on the photodiode board.
    // Below is the exact indexing:
    //      D1 => photodiodes[0][2]
    //      D2 => photodiodes[1][3]
    //      D3 => photodiodes[2][4]
    //      D4 => photodiodes[3][3]
    //      D5 => photodiodes[4][2]
    //      D6 => photodiodes[3][1]
    //      D7 => photodiodes[2][0]
    //      D8 => photodiodes[1][1]
    // All other values in between are zeros and disgarded in processing.
    uint16_t photodiodes[PHOTODIODE_ARRAY_Y][PHOTODIODE_ARRAY_X] = {
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000}};

    // Setup GPIO pins used for selecting mux channels
    for (const auto &pin : pins)
    {
        pinMode(pin, OUTPUT);
    }

    // Setup ADC
    Adafruit_ADS1015 ads;
    ads.setGain(GAIN_ONE);
    ads.begin();

    while (1)
    {
        for (const auto &[key, val] : photodiodeIdx)
        {
            // Write key to GPIO pins to select particular photodiode
            for (int i = 0; i < 4; i++)
            {
                digitalWrite(pins[i], (key >> i) & 0x01);
            }

            // TODO: Is this needed?
            // Sleep for 1ms for good measuer
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // Read value from adc and store in photodiodes matrix
            // TODO: Double check all values are coming from A0,
            // or else we need to keep track of that too
            photodiodes[val[0]][val[1]] = ads.readADC_SingleEnded(0);
        }

        // print matrix
        std::cout << std::showbase
                  << std::internal
                  << std::setfill('0');

        std::cout << "-----------------------------------------------------------" << std::endl;

        for (const auto &row : photodiodes)
        {
            for (const auto &col : row)
            {
                std::cout << std::hex << std::setw(6) << col << " ";
            }
            std::cout << std::endl
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void imgProcThread()
{
    std::vector<std::string> data{"to the right", "below", "up"};
    logfileWrite(data);
}

int main()
{
    // Initialize WiringPi
    if (wiringPiSetup() == -1)
    {
        std::cerr << "Unable to start WiringPi" << std::endl;
        return 1;
    }

    // Launch threads
    std::thread t_imu(IMUThread);
    // std::thread t_photodiode(photodiodeThread);
    // std::thread t_imgProc(imgProcThread);

    // Join threads
    t_imu.join();
    // t_photodiode.join();
    // t_imgProc.join();

    return 0;
}
