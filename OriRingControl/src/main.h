/**********************************************************************************************
 * Haptic OriRing
 * by Mustafa Mete <mustafa.mete@epfl.ch>
 **********************************************************************************************/
#include <Arduino.h>
#include "Timing.cpp"           // Custom class to implement accurate timing functions
#include "pouch.h"
#include "pyramidFSR.h"
#include <Keyboard.h>           // include the keyboard library

double minPressure = MIN_PRESSURE, maxPressure = MAX_PRESSURE; // [kPa]
unsigned long now, period;

// bending sensor parameters
int bendingSensorPin1 = A8, bendingSensorPin2 = A9;

int bendingAngle1_0 = 310, bendingAngle2_0 = 460; // get them form calibration
int bendingAngle1_90 = 680, bendingAngle2_90 = 252 ;  // get them form calibration

float bendingAngleSlope; // formula



float ringLegAngleBS = 0; // [rad]
float ringHeightBS = 0; //[mm]
float maxRingHeight = 30;

float bendingSensorVsupp = 3.0; // [v]
float bdVolDivRes = 10; // [ohm]

// Set the window size/period as needed
const int windowSize = 400; // 400 around 1 s
const float changeThreshold = 0.8; // 1.5; //4;
int swipeRisingEdgeMaxLimit = 400; // 400;

int swipeDirection = 0, swipeDirectionPrevious = 0, swipeRisingEdgeCounter = 0;
float sectionValues[4][windowSize];
float sectionSum[4], sectionAverage[4], sectionThreshold[4];

int valueIndex = 0;

int calculateSwipeDirection_PFSR(int changeArray[4]);
int sectionChange[4] = {0,0,0,0}; // -1 falling, 0 staying still, 1 rising

// 0 no swipe, 1 +x direction, 2 -x direction, 3 +y direction, 4 -y direction
int swipeWasHigh = 0;
int pinPotentiometer = A3;

// badwidth test
unsigned long startTime, initialTime;
double currentTime;
float t, t_all, sinPressureTrajectory;
float amplitude = 30, offsetPressure = 20, frequency = 0.25; //0.1; // [kPa, kPa, Hz]
int cycleCount = 0, cyclesToDouble = 3, maxFrequency = 100;


