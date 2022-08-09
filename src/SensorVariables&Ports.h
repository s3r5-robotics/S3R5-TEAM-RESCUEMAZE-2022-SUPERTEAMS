#pragma once
// TOF Sensors
#define ServoPin 26
#define TOFPinCenter 25
#define TOFPinRight 32
#define TOFPinLeft 33
#define PhotoResistor 13 // photo resistor
#define MaxTOFDistance 200
#define TOFForwardCalibrationValue 0
#define TOFRightCalibrationValue 0
#define TOFLeftCalibrationValue 0
#define TOFForwardDistanceFromCenter 0
#define TOFRightDistanceFromCenter 0
#define TOFLeftDistanceFromCenter 0
#define DistanceFromRobotToWall 110 // distance from wall to robot to be centered

// Buttons
#define StartButton 35 // start/stop button
#define OtherButton 32 // other button

// Temperature
#define TempLeftMLXPort 1
#define TempRightMLXPort 0

// Laser sensorts
#define LaserForwardMLXPort 7
#define LaserRightMLXPort 0
#define LaserLeftMLXPort 1

// Motors
#ifndef SpeedConfig
#define SpeedConfig
#define MotorDrivingSpeed 250
#define MotorDrivingSpeedFast 400
#define MotorTurnSpeedFast 250
#define MotorTurnSpeedSlow 150
#endif

#ifndef DXL_pin
#define DXL_pin 23
#endif

#define MotorID_FL 4
#define MotorID_FR 5
#define MotorID_BL 9
#define MotorID_BR 2
#define MotorBaseCCWValue 0
#define MotorBaseCWValue 1024 // if value is over 1024 wheel starts turning the other direction

// multiplexer address
#define MultyplexerAddress 0x70

// movement dirrections
#define STOP 0b000
#define FORWARD 0b001
#define BACKWARD 0b010
#define LEFT 0b011
#define RIGHT 0b100

// led states
#define ON 1
#define OFF 0

// turn directions
#define TURN_LEFT -90
#define TURN_RIGHT 90

// camera port
#define OpenMVCameraLeft 0
#define OpenMVCameraRight 1

// color ranges for photo resistor
#define BlackLow 1100
#define BlackHigh 1400
#define WhiteLow 2730
#define WhiteHigh 2770
#define SilverLow 2350
#define SilverHigh 2450

// temperature range
#define TempLow 27 // 28
#define TempHigh 42 // 40

// reporting LED
#define ReportLED 33

// color sensor 
#define ColorSensorPin 25 // GPIO 25

// camera pins
#ifndef OpenMVPins
#define OpenMVPins
#define OpenMV1p1 5
#define OpenMV1p2 4
#define OpenMV1p3 15

#define OpenMV2p1 33
#define OpenMV2p2 32
#define OpenMV2p3 35
#endif

// maximum amount of victims

#ifndef MaxVictimNum
#define MaxVictimNum 6
#endif

// WiFi
const char* ssid = "LER";
const char* password = "doberdan";