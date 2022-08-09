#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>
#include <DynamixelShield.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "DFRobot_I2CMultiplexer.h"
#include "Adafruit_MLX90614.h"
#include "Adafruit_I2CDevice.h"
#include "Adafruit_SPIDevice.h"
#include "Adafruit_VL53L1X.h"
#include <SensorVariables&Ports.h>
#include <Servo.h>

//Multiplexer
DFRobot_I2CMultiplexer I2CMulti(MultyplexerAddress);              
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//Motors
DynamixelShield dxl(Serial2, DXL_pin);

//Gyro
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t bno_event;

//Laser TOFs
//NewPing TOFForward = NewPing(TOFPinCenter, TOFPinCenter, MaxTOFDistance);
//NewPing TOFRight = NewPing(TOFPinRight, TOFPinRight, MaxTOFDistance);
//NewPing TOFLeft = NewPing(TOFPinLeft, TOFPinLeft, MaxTOFDistance);

Adafruit_VL53L1X VL53 = Adafruit_VL53L1X();

Servo servo;

int currentDirection = 0;
int currentTurnSpeed = 0;

int tempDistance = 0;

uint8_t lineNum = 0;

float offset = 0;

void resumeMotors();

int GetDistance(int sensor);

//Initialization
/**
 * @brief prepares the dxl motors
 * 
 * @param id id of mottor (written on the dxl motor)
*/
void prepareMotor(int id)
{
  if(!dxl.ping(id)){
      Serial.println("Can't ping motor!");
  }
  else
  {
      Serial.println("Motors are pinged");
  }
  dxl.torqueOff(id);
  dxl.setWheelMode(id);
  dxl.torqueOn(id);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(1.0);
}

/**
 * @brief initializes the sensors
*/
void initialazeSensors() 
{
    //Gyro
    if(!bno.begin())
    {
        Serial.println("Bno not detected");
        //exit(0);
    }
    Serial.println("BNO begin");

    //Motors
    Serial2.begin(115200);
    prepareMotor(MotorID_FL); // front left 
    prepareMotor(MotorID_FR); // front right
    prepareMotor(MotorID_BL); // back left
    prepareMotor(MotorID_BR); // back right
    Serial.println("Motor begin");

    delay(2000); // Pause for 2 seconds

    // Clear the buffer

    // setup pins for output / intput
    pinMode(OpenMV1p1, INPUT);
    pinMode(OpenMV1p2, INPUT);
    pinMode(OpenMV1p3, INPUT);
    pinMode(OpenMV2p1, INPUT);
    pinMode(OpenMV2p2, INPUT);
    pinMode(OpenMV2p3, INPUT);
    Serial.println("Pins set");

    pinMode(StartButton, INPUT);
    pinMode(OtherButton, INPUT);
    pinMode(PhotoResistor, ANALOG);
    pinMode(ReportLED, OUTPUT);
    pinMode(ColorSensorPin, OUTPUT);
    digitalWrite(ColorSensorPin, HIGH);
    digitalWrite(ReportLED, OFF);
    Serial.println("Pins set 2");

    servo.attach(ServoPin);
    servo.write(90);
    Serial.println("Servo attach");
    
    //Temperature
    I2CMulti.selectPort(TempLeftMLXPort);   
    mlx.begin();
    I2CMulti.selectPort(TempRightMLXPort);
    mlx.begin();
    Serial.println("mlx begin");

    //Begin wire
    Wire.begin();
    Serial.println("Wire begin");

    //Laser sensors
    I2CMulti.selectPort(LaserForwardMLXPort);
    if(!VL53.begin(0x29))
    {
        Serial.printf("VL531 %i\n", VL53.vl_status);
    }else
        Serial.println("VL531 setup");
    VL53.startRanging();
    delay(200);
    I2CMulti.selectPort(LaserRightMLXPort);
    if(!VL53.begin(0x29)){
        Serial.printf("VL531 %i\n", VL53.vl_status);
    }else
        Serial.println("VL532 setup");
    VL53.startRanging();
    delay(200);
    I2CMulti.selectPort(LaserLeftMLXPort);
    if(!VL53.begin(0x29)){
        Serial.printf("VL531 %i\n", VL53.vl_status);
    }else
        Serial.println("VL533 setup");
    VL53.startRanging();
    delay(200);

    // INITAL DISTANCE
    tempDistance = GetDistance(LaserForwardMLXPort);

    Serial.println("Init");
}

/**
 * @brief setup after button press
*/
void setupAfterStart()
{
    bno.getEvent(&bno_event, Adafruit_BNO055::VECTOR_EULER);
    offset = bno_event.orientation.x;
}

//Functions
/**
 * @brief turns on the motors so the robot spins/moves in the correct direction with provided turn speed
 * 
 * @param direction direction the robot to go in (LEFT, RIGHT, FORWARD, BACK, STOP)
 * 
 * @param turnSpeed how fast the robot should turn (MotorTurnSpeedFast, MotorTurnSpeedSlow)
*/
void engageMotors(int direction, int turnSpeed = MotorTurnSpeedFast, bool fast = false)
{
    int rightMotorsSpeed, leftMotorsSpeed;
    int forwardSpeed = fast ? MotorDrivingSpeedFast : MotorDrivingSpeed;
    currentDirection = direction == STOP ? currentDirection : direction;
    currentTurnSpeed = turnSpeed == STOP ? currentTurnSpeed : turnSpeed;
    
    if (direction == FORWARD)
    {
        leftMotorsSpeed = MotorBaseCCWValue + forwardSpeed;
        rightMotorsSpeed = MotorBaseCWValue + forwardSpeed;
        //Serial.println("FORWARD");
    }
    else if (direction == BACKWARD)
    {
        leftMotorsSpeed = MotorBaseCWValue + forwardSpeed;
        rightMotorsSpeed = MotorBaseCCWValue + forwardSpeed;
        //Serial.println("BACKWARD");
    }
    else if (direction == LEFT)
    {
        leftMotorsSpeed = MotorBaseCWValue + turnSpeed;
        rightMotorsSpeed = MotorBaseCWValue + turnSpeed;
        //Serial.println("LEFT");
    }
    else if (direction == RIGHT)
    {
        leftMotorsSpeed = MotorBaseCCWValue + turnSpeed;
        rightMotorsSpeed = MotorBaseCCWValue + turnSpeed;
        //Serial.println("RIGHT");
    }
    else if (direction == STOP)
    {
        leftMotorsSpeed = 0;
        rightMotorsSpeed = 0;
        //Serial.println("STOP");
    }
    else return;
       
    dxl.setGoalVelocity(MotorID_FL, leftMotorsSpeed, UNIT_RAW);
    dxl.setGoalVelocity(MotorID_BL, leftMotorsSpeed, UNIT_RAW);
    dxl.setGoalVelocity(MotorID_FR, rightMotorsSpeed, UNIT_RAW);
    dxl.setGoalVelocity(MotorID_BR, rightMotorsSpeed, UNIT_RAW);
}

// puts the motors back to the state before they stoped
/**
 * @brief resumes the motor movement before the engage motors function got STOP direction
*/
void resumeMotors()
{
    engageMotors(currentDirection, currentTurnSpeed);
}

/**
 * @brief get current robot orientation
*/
float GetCurrentHeading()
{
  bno.getEvent(&bno_event, Adafruit_BNO055::VECTOR_EULER);
  return ((int)(bno_event.orientation.x - offset + (float)360) % 360);
}

/**
 * @brief gets current acceleration vector for x axis
*/
float GetCurrentAccel()
{
    bno.getEvent(&bno_event, Adafruit_BNO055::VECTOR_LINEARACCEL);
    return bno_event.acceleration.x;
}

/**
 * @brief corrects the input angle (that must be eather 0, 90, 270, 360, -90 or 450) that snaps it from 0 to 360 deg
*/
int correctDegree(int currentDeg)
{
    Serial.println(currentDeg);
    if(currentDeg == -90)
        return 270;
    else if(currentDeg == 450)
        return 90;
    else if(currentDeg == 360)
        return 0;
    return currentDeg;
}

//default return form camera output should be N other possible values could denote color or a character ranging from (R G Y to H S U)
/**
 * @brief get camera output (R, G, Y, H, S, U)
 * 
 * @param port i2c port camera is connected to
*/
int getCameraOutput(int port)
{
    //Serial.printf("P1: %i, P2: %i, P3: %i, ", digitalRead(OpenMV1p1), digitalRead(OpenMV1p2), digitalRead(OpenMV1p3));
    if(port == true)
        return ((uint8_t)(digitalRead(OpenMV1p1)) | (uint8_t)(digitalRead(OpenMV1p2)<<1) | (uint8_t)(digitalRead(OpenMV1p3)<<2));
    else
        return ((uint8_t)(digitalRead(OpenMV2p1)) | (uint8_t)(digitalRead(OpenMV2p2)<<1) | (uint8_t)(digitalRead(OpenMV2p3)<<2));
    /*I2CMulti.selectPort(port);
    Wire.requestFrom(0x54, 2);
    int32_t data = Wire.read();
    //Serial.println(data);
    return data;*/
}

/**
 * @brief get temperature readout on provided side
 * 
 * @param direction side that the sensor is on (LEFT, RIGHT)
*/
float getTemperature(int direction = LEFT)
{
    if (direction == LEFT)
    {
        I2CMulti.selectPort(TempLeftMLXPort);
    }
    else if (direction == RIGHT)
    {
        I2CMulti.selectPort(TempRightMLXPort);
    }
    return mlx.readObjectTempC();
}

/**
 * @brief read button state true or flase
*/
bool readButton()
{
    return digitalRead(StartButton);
}

/**
 * @brief read analog value of photo resistors boundries in "SensorVariables&Ports.h"
*/
uint16_t readPhotoResistor()
{
    return analogRead(PhotoResistor);
}

/**
 * @brief set servo turn angle 0 - 180 deg
*/
void turnServo(int deg)
{
    servo.write(deg);
}

/**
 * @brief turn LED on or off
 * 
 * @param toggle state the led should be set to (ON, OFF)
*/
void toggleLED(bool toggle = 1)
{
    digitalWrite(ReportLED, toggle);
}

bool checkForBlack()
{
    uint16_t reading = readPhotoResistor();
    return (reading > BlackLow && reading < BlackHigh);
}

bool checkForSilver()
{
    uint16_t reading = readPhotoResistor();
    return (reading > SilverLow && reading < SilverHigh);
}

/**
 * @brief get distance on sensor in param
 * 
 * @param direction direction the sensor to be read of should be (LeserLeft, LeserRight, LaserForward)
*/
int GetDistance(int direction)
{
    I2CMulti.selectPort(direction);
    return VL53.distance();
}

/**
 * @brief checks if robot is on ramp
 * 
 * @returns boolean
*/
bool checkRamp()
{
    bno.getEvent(&bno_event, Adafruit_BNO055::VECTOR_EULER);
    return !(bno_event.orientation.y < -3);
}

/**
 * @brief dispenses a pellet this takes 1s
*/
void dispensePallet(uint8_t side = LEFT)
{
    delay(500);
    servo.write(side == LEFT ? 180 : 0);
    delay(1500);
    servo.write(90);
}

/**
 * @brief blinks LED for 5 seconds
*/
void blinkLED()
{
    toggleLED(ON);
    delay(500);
    toggleLED(OFF);
    delay(500);
    toggleLED(ON);
    delay(500);
    toggleLED(OFF);
    delay(500);
    toggleLED(ON);
    delay(500);
    toggleLED(OFF);
    delay(500);
    toggleLED(ON);
    delay(500);
    toggleLED(OFF);
    delay(500);
    toggleLED(ON);
    delay(500);
    toggleLED(OFF);
    delay(500);
}