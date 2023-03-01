/**
 * bbstlogger.cpp
 * Author: Ethan Garnier
 * Data logging application for the 
 * Balloon Borne Solar Telescope (BBST)
*/
#include <iostream>
#include <thread>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
//#include "cmps14.hpp"
//#include <wiringPi.h>
//#include <wiringSerial.h>

void IMUThread(std::string logFile)
{
    std::cout << "IMU THREAD" << std::endl;
}

void photodiodeThread(std::string logFile)
{
    std::cout << "PHOTODIODE THREAD" << std::endl;
}

void imgProcThread()
{
    std::cout << "IMAGE PROCESSING THREAD" << std::endl;
}

int main()
{
    // File that data will be logged to
    std::string logFile = "log.csv";

    // Launch threads
    std::thread t_imu(IMUThread, logFile);
    std::thread t_photodiode(photodiodeThread, logFile);
    std::thread t_imgProc(imgProcThread);

    // Join threads
    t_imu.join();
    t_photodiode.join();
    t_imgProc.join();

    return 1;
}
