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
// motor controllers from IMU data.
struct
{
    union
    {
        float azimuth;
        uint8_t azimuthBuff[sizeof(float)];
    };
    union
    {
        float elevation;
        uint8_t elevationBuff[sizeof(float)];
    };
    union
    {
        float heading;
        uint8_t headingBuff[sizeof(float)];
    };
    union
    {
        float pitch;
        uint8_t pitchBuff[sizeof(float)];
    };
    union
    {
        float roll;
        uint8_t rollBuff[sizeof(float)];
    };
} IMUComm;

// Send data stored in IMUComm to motor controller by
// by sending each float byte by byte. Each float will be
// separated by a comma in the communication stream.
void sendIMUComm(int controllerFd)
{
    int i;
    for (i = 0; i < sizeof(float); i++)
        serialPutchar(controllerFd, IMUComm.azimuthBuff[i]);

    serialPutchar(controllerFd, ',');

    for (i = 0; i < sizeof(float); i++)
        serialPutchar(controllerFd, IMUComm.elevationBuff[i]);

    serialPutchar(controllerFd, ',');

    for (i = 0; i < sizeof(float); i++)
        serialPutchar(controllerFd, IMUComm.headingBuff[i]);

    serialPutchar(controllerFd, ',');

    for (i = 0; i < sizeof(float); i++)
        serialPutchar(controllerFd, IMUComm.pitchBuff[i]);

    serialPutchar(controllerFd, ',');

    for (i = 0; i < sizeof(float); i++)
        serialPutchar(controllerFd, IMUComm.rollBuff[i]);

    serialPutchar(controllerFd, ',');
}

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

    std::cout << "IMU Software Version: " << imu->getSoftwareVersion() << std::endl;
    std::vector<int> calStatus = imu->getCalibrationStatus();
    std::cout << "IMU Calibration State: " << std::endl
              << "\tSystem: " << calStatus[3] << std::endl
              << "\tGyroscope: " << calStatus[2] << std::endl
              << "\tAccelerometer: " << calStatus[1] << std::endl
              << "\tMagnotometer: " << calStatus[0] << std::endl;

    // Initialize serial communication to the motor controllers
    int controllerFd = serialOpen("/dev/ttyACM0", 115200);
    if (controllerFd == -1)
    {
        std::cerr << "Unable to initialize serial communication with gimbal controller. Please check serial port being used..." << std::endl;
        return;
    }

    // Main IMU loop
    // TODO: During flight these position values will change on each iteration
    float latitude = 45.944962483507844;
    float longitude = -66.64841312244609;
    float prevHeading = 0.0f, prevPitch = 0.0f, prevRoll = 0.0f;
    std::vector<std::string> data(5);
    while (1)
    {
        // Calculate location of Sun at current point in time
        calcSunPos(IMUComm.elevation, IMUComm.azimuth, longitude, latitude);

        // Read data from IMU and store in IMUComm structure
        // Experimenting with a weighted reading based off previous value
        // to provide better smoothing.
        IMUComm.heading = imu->getHeading();
        IMUComm.heading = (IMUComm.heading != 0.0f) ? IMUComm.heading : 0.1f;
        IMUComm.heading = (prevHeading != 0.0f) ? 0.8 * IMUComm.heading + 0.2 * prevHeading : IMUComm.heading;

        IMUComm.pitch = imu->getPitch();
        IMUComm.pitch = (IMUComm.pitch != 0.0f) ? IMUComm.pitch : 0.1f;
        IMUComm.pitch = (prevPitch != 0.0f) ? 0.8 * IMUComm.pitch + 0.2 * prevPitch : IMUComm.pitch;

        IMUComm.roll = imu->getRoll();
        IMUComm.roll = (IMUComm.roll != 0.0f) ? IMUComm.roll : 0.1f;
        IMUComm.roll = (prevRoll != 0.0f) ? 0.8 * IMUComm.roll + 0.2 * prevRoll : IMUComm.roll;

        sendIMUComm(controllerFd);

        prevHeading = IMUComm.heading;
        prevPitch = IMUComm.pitch;
        prevRoll = IMUComm.roll;

        // Generate the data array to send to log file
        // data[0] = "Sun Azimuth: " + std::to_string(IMUComm.azimuth);
        // data[1] = "Sun Elevation: " + std::to_string(IMUComm.elevation);
        // data[2] = "IMU Heading: " + std::to_string(IMUComm.heading);
        // data[3] = "IMU Pitch: " + std::to_string(IMUComm.pitch);
        // data[4] = "IMU Roll: " + std::to_string(IMUComm.roll);

        std::cout << "Sent: " << IMUComm.azimuth << ", " << IMUComm.elevation << ", " << IMUComm.heading << ", " << IMUComm.pitch << ", " << IMUComm.roll << std::endl;

        // Log data to logfile
        // logfileWrite(data);

        // TODO: Experimentally determine lower limit on delay between loops
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // TODO: Need to close this in a signal handler since
    // this is never reached...
    serialClose(controllerFd);
}

void photodiodeThread()
{
    const int PHOTODIODE_ARRAY_X = 5;
    const int PHOTODIODE_ARRAY_Y = 5;
    const int NPHOTODIODES = 8;

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

    // Store photodiode values for finding max and min
    uint16_t photodiodeValues[NPHOTODIODES] = {
        0x0000,
        0x0000,
        0x0000,
        0x0000,
        0x0000,
        0x0000,
        0x0000,
        0x0000,
    };

    // Setup GPIO pins used for selecting mux channels
    for (const auto &pin : pins)
    {
        pinMode(pin, OUTPUT);
    }

    // Setup ADC
    Adafruit_ADS1015 ads;
    ads.setGain(GAIN_ONE);
    ads.begin();

    int i = 0, norm = 0;
    int centerX = ceil(PHOTODIODE_ARRAY_X / 2), centerY = ceil(PHOTODIODE_ARRAY_Y / 2);
    uint16_t maxVal1 = 0x0000, maxVal2 = 0x0000, value = 0x0000;
    std::vector<int> maxPos1{0, 0}, maxPos2{0, 0};
    std::vector<int> moveVect{0, 0};
    while (1)
    {
        i = 0;
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

            // Read value from adc
            // TODO: Double check all values are coming from A0,
            // or else we need to keep track of that too
            value = ads.readADC_SingleEnded(0);

            // Store value in photodiodes matrix
            photodiodes[val[0]][val[1]] = value;

            // Store value in photodiode value array
            photodiodeValues[i++] = value;
        }

        // Normalize by taking the average of weakest half photodiodes
        // and subtracting that value from each photodiode intensity
        norm = 0;
        std::sort(photodiodeValues, photodiodeValues + NPHOTODIODES);
        for (i = 0; i < NPHOTODIODES / 2; i++)
        {
            norm += photodiodeValues[i];
        }

        norm /= (NPHOTODIODES / 2);

        for (auto &[key, val] : photodiodeIdx)
        {
            photodiodes[val[0]][val[1]] -= norm;
        }

        // TODO: We might be able to rewrite the below function
        // now that we have the sorted array of intensities

        // Find two maximum photodiode values and their indicies in array
        maxVal1 = 0x0000;
        maxVal2 = 0x0000;
        for (i = 0; i < PHOTODIODE_ARRAY_Y; i++)
        {
            auto rowMax = std::max_element(photodiodes[i], photodiodes[i] + PHOTODIODE_ARRAY_X);
            if (*rowMax > maxVal1)
            {
                maxVal2 = maxVal1;
                maxPos2[0] = maxPos1[0];
                maxPos2[1] = maxPos1[1];

                maxPos1[0] = std::distance(photodiodes[i], rowMax);
                maxPos1[1] = i;
                maxVal1 = *rowMax;
            }
            else if (*rowMax > maxVal2)
            {
                maxPos2[0] = std::distance(photodiodes[i], rowMax);
                maxPos2[1] = i;
                maxVal2 = *rowMax;
            }
        }

        moveVect[0] = (maxPos1[0] - centerX) * maxVal1 + (maxPos2[0] - centerX) * maxVal2;
        moveVect[1] = (-1 * maxPos1[1] - centerY) * maxVal1 + (-1 * maxPos2[1] - centerY) * maxVal2;

        double temp = tan(moveVect[1]/moveVect[0]);

        std::cout << "Max1 " << maxPos1[0] << "," << maxPos1[1] << ": " << maxVal1 << std::endl;
        std::cout << "Max2 " << maxPos2[0] << "," << maxPos2[1] << ": " << maxVal2 << std::endl;
        std::cout << "Move Vect: " << moveVect[0] << ", " << moveVect[1] << std::endl;
        std::cout << "Heading correction: " << temp << std::endl;
        // std::cout << "Max Photodiode at " << maxPos[0] << ", " << maxPos[1] << " intensity: " << maxVal << std::endl;
        // std::cout << "Vector to center: " << moveVect[0] << ", " << moveVect[1] << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    // std::thread t_imu(IMUThread);
    std::thread t_photodiode(photodiodeThread);
    // std::thread t_imgProc(imgProcThread);

    // Join threads
    // t_imu.join();
    t_photodiode.join();
    // t_imgProc.join();

    return 0;
}
