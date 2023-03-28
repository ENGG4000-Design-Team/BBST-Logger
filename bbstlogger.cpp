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

#define PI 3.14159265358979323846

const std::string LOGFILE = "log.csv";
std::mutex m_logfile;
int motorControllerFd;

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
void sendIMUComm()
{
    int i;
    for (i = 0; i < sizeof(float); i++)
        serialPutchar(motorControllerFd, IMUComm.azimuthBuff[i]);

    serialPutchar(motorControllerFd, ',');

    for (i = 0; i < sizeof(float); i++)
        serialPutchar(motorControllerFd, IMUComm.elevationBuff[i]);

    serialPutchar(motorControllerFd, ',');

    for (i = 0; i < sizeof(float); i++)
        serialPutchar(motorControllerFd, IMUComm.headingBuff[i]);

    serialPutchar(motorControllerFd, ',');

    for (i = 0; i < sizeof(float); i++)
        serialPutchar(motorControllerFd, IMUComm.pitchBuff[i]);

    serialPutchar(motorControllerFd, ',');

    for (i = 0; i < sizeof(float); i++)
        serialPutchar(motorControllerFd, IMUComm.rollBuff[i]);

    serialPutchar(motorControllerFd, ',');

    std::cout << "Sent: " << IMUComm.azimuth << "," << IMUComm.elevation << "," << IMUComm.heading << "," << IMUComm.pitch << "," << IMUComm.roll << "," << std::endl;
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

        sendIMUComm();

        prevHeading = IMUComm.heading;
        prevPitch = IMUComm.pitch;
        prevRoll = IMUComm.roll;

        // Generate the data array to send to log file
        // data[0] = "Sun Azimuth: " + std::to_string(IMUComm.azimuth);
        // data[1] = "Sun Elevation: " + std::to_string(IMUComm.elevation);
        // data[2] = "IMU Heading: " + std::to_string(IMUComm.heading);
        // data[3] = "IMU Pitch: " + std::to_string(IMUComm.pitch);
        // data[4] = "IMU Roll: " + std::to_string(IMUComm.roll);

        // Log data to logfile
        // logfileWrite(data);

        // TODO: Experimentally determine lower limit on delay between loops
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void photodiodeThread()
{
    const int PHOTODIODE_ARRAY_X = 5;
    const int PHOTODIODE_ARRAY_Y = 5;
    const int NPHOTODIODES = 8;

    // GPIO pins we are using to control multiplexer channels
    int pins[4] = {1, 2, 3, 4};

    // Map multiplexer channel bits to photodiode data array where
    // index 0 is its x distance from center, index 1 is its y distance
    // from center, and index 2 is its intensity value
    std::map<uint8_t, std::vector<int>> photodiodes{
        {0x00, {0, 2, 0}},   // 0b0000 corresponds to photodiode D1
        {0x01, {1, 1, 0}},   // 0b0001 corresponds to photodiode D2
        {0x02, {2, 0, 0}},   // 0b0010 corresponds to photodiode D3
        {0x03, {1, -1, 0}},  // 0b0011 corresponds to photodiode D4
        {0x04, {0, -2, 0}},  // 0b0100 corresponds to photodiode D5
        {0x05, {-1, -1, 0}}, // 0b0101 corresponds to photodiode D6
        {0x0A, {-2, 0, 0}},  // 0b1010 corresponds to photodiode D7
        {0x0B, {-1, 1, 0}},  // 0b1011 corresponds to photodiode D8
    };

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
    /*uint16_t photodiodes[PHOTODIODE_ARRAY_Y][PHOTODIODE_ARRAY_X] = {
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
        {0x0000, 0x0000, 0x0000, 0x0000, 0x0000}};*/

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

    int i = 0;
    float norm = 0.0f, magnitude = 0.0f, xCorrection = 0.0f, yCorrection = 0.0f;
    uint16_t value = 0x0000;
    std::vector<int> maxVal1{0, 0, 0}, maxVal2{0, 0, 0};
    std::vector<float> moveVect{0, 0};
    while (1)
    {
        auto start = std::chrono::high_resolution_clock::now();

        i = 0;
        for (auto &[key, val] : photodiodes)
        {
            // Write key to GPIO pins to select particular photodiode
            for (int i = 0; i < 4; i++)
            {
                digitalWrite(pins[i], (key >> i) & 0x01);
            }

            // Sleep for 1ms for good luck
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // Read value from adc and store
            // TODO: Double check all values are coming from A0,
            // or else we need to keep track of that too
            value = ads.readADC_SingleEnded(0);
            val[2] = value;
            photodiodeValues[i++] = value;
        }

        // Normalize by taking the average of weakest half photodiodes
        // and subtracting that value from each photodiode intensity
        norm = 0.0;
        std::sort(photodiodeValues, photodiodeValues + NPHOTODIODES);
        for (i = 0; i < NPHOTODIODES / 2; i++)
        {
            norm += photodiodeValues[i];
        }

        norm /= (NPHOTODIODES / 2);

        // Find two maximum photodiode values and their indicies in array
        maxVal1[2] = 0;
        maxVal2[2] = 0;
        for (auto &[key, val] : photodiodes)
        {
            val[2] -= norm;
            if (val[2] > maxVal1[2])
            {
                maxVal2 = maxVal1;
                maxVal1 = val;
            }
            else if (val[2] > maxVal2[2])
            {
                maxVal2 = val;
            }
        }

        // TODO: Store prev location and if it is the same
        // do not send data.

        xCorrection = 0.0f;
        yCorrection = 0.0f;
        magnitude = sqrt(maxVal1[2] * maxVal1[2] + maxVal2[2] * maxVal2[2]);
        if (magnitude > 2.5f)
        {
            moveVect[0] = static_cast<float>(maxVal1[0] * maxVal1[2] + maxVal2[0] * maxVal2[2]);
            moveVect[1] = static_cast<float>(maxVal1[1] * maxVal1[2] + maxVal2[1] * maxVal2[2]);

            float theta = atan2(moveVect[1], moveVect[0]);

            std::cout << theta * (180 / PI) << std::endl;

            moveVect[0] = 0.89 * cos(theta);
            moveVect[1] = 0.89 * sin(theta);

            xCorrection = 1.5 * atan(moveVect[0] / 21.4f) * 180 / PI;
            yCorrection = 1.5 * atan(moveVect[1] / 21.4f) * 180 / PI;

            std::cout << xCorrection << ", " << yCorrection << std::endl;

            IMUComm.azimuth = 181.0f - xCorrection;
            IMUComm.elevation = 41.0f - yCorrection;
            IMUComm.heading = 0.1f;
            IMUComm.pitch = 0.1f;
            IMUComm.roll = 0.1f;

            sendIMUComm();
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Execution time: " << duration.count() << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(40));
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

    // Initialize serial communication to the motor controllers
    motorControllerFd = serialOpen("/dev/ttyACM0", 115200);
    if (motorControllerFd == -1)
    {
        std::cerr << "Unable to initialize serial communication with gimbal controller. Please check serial port being used..." << std::endl;
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

    serialClose(motorControllerFd);

    return 0;
}
