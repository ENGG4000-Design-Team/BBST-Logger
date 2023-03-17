// SolarPosition.h

// 2019 Ken Willmott
// Arduino library based on the program "Arduino Uno and Solar Position Calculations"
// (c) David R. Brooks, which can be found at http://www.instesre.org/ArduinoDocuments.htm
// and issued under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License:
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// 
// Modifications made by Ethan Garnier 2023

#include <chrono>
#include <time.h>

const float KM_PER_AU = 149597870.7;  //kilometers per astronomical unit

const float RAD_TO_DEG = 57.29577951;
const float DEG_TO_RAD = 0.01745329;
const float PI = 3.141592654;
const float TWO_PI = 6.28318531;

typedef time_t(*getExternalTime)();

// data structure to store solar position results
struct SolarPosition_t
{
  float elevation = 0;
  float azimuth = 0;
  float distance = 0;
  time_t time = 0;
};

// utility functions
long JulianDate(int year, int month, int day);
SolarPosition_t calculateSolarPosition(time_t tParam, float Latitude, float Longitude);

// class interface
class SolarPosition
{
  private:

    static getExternalTime getExtTimePtr;  // shared pointer to external sync function

    // current values:
    float Latitude;
    float Longitude;

    // results:
    SolarPosition_t result;

  public:

    SolarPosition(float Latitude, float Longitude);

    static void setTimeProvider(getExternalTime getTimeFunction);

    SolarPosition_t getSolarPosition();
    SolarPosition_t getSolarPosition(time_t t);

    float getSolarElevation();
    float getSolarElevation(time_t t);

    float getSolarAzimuth();
    float getSolarAzimuth(time_t t);

    float getSolarDistance();
    float getSolarDistance(time_t t);
};
