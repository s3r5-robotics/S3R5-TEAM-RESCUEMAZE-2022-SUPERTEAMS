#pragma once
#include <Arduino.h>
#include <ProgramVariables.h>
#include <SensorObjects&Functions.h>

int absoluteHeading = 0;
uint16_t avgDist = 0;
uint8_t count = 0;
bool detecting = false; // zato ker je detection v drugem loopu da pausa robota
bool turning = false;
bool paused = false;

// kit despensing in black tile
uint8_t numberOfBlackTiles = 0;
bool haltTile = false;

int shouldTurn(int targetRot);

void TurnRobot90(int direction);

void checkHole();

void movementLoop();

void goForward();

void goForwardR();

void GoForwardOneTile();

void pauseRobot()
{
    if(!readButton())
    {
        Serial.println("Pause");
        paused = true;
        engageMotors(STOP);
        delay(3000);
        while(readButton()){ ; };
        ESP.restart();
    }
}

void updateDetecting(bool val);

/**
 * @brief returns true if robot should keep turning to target angle
 * 
 * @param targetRot angle to check
*/
int shouldTurn(int targetRot)
{ //*Preveri ce se robot more obrnit na kot
    float curRot = GetCurrentHeading();
    //Serial.printf("%f ", curRot);
    //Serial.printf("Heading deg: %f, target: %i\n", curRot, targetRot);
    int p0 = targetRot;
    int p3 = (targetRot + 5) % 360;
    int p15 = (targetRot + 15) % 360;
    int n3 = (targetRot - 5 + 360) % 360;
    int n15 = (targetRot - 15 + 360) % 360;
    int p180 = (targetRot + 180) % 360;
    int l = 0, r = 0;

    //Serial.printf("Targer: %f, Current: %f\n", targetRot, curRot);
    //Within(-3,+3)deg, stop turing.
    l = n3; r = p3;
    if ((l < r && curRot > l && curRot < r) ||
    (l > r && (curRot > l || curRot < r)))
    {
        //Serial.print("In first if");
        return false;
    }
    //Within[3,15]deg,Turn Slowly
    l = p3; r = p15;
    if ((l < r && curRot >= l && curRot <= r) ||
        (l > r && (curRot >= l || curRot <= r)))
    {
        //Serial.print("In second if");
        return true;
    }
    //Within[15,180]deg,Turn Faast
    l = p15; r = p180;
    if ((l < r && curRot >= l && curRot <= r) ||
       (l > r && (curRot >= l || curRot <= r)))
    {
        //Serial.print("In third if");
        return true;
    }
    //Within[-15,-3]deg,Turn Slowly
    l = n15; r = n3;
    if ((l < r && curRot >= l && curRot <= r) ||
    (l > r && (curRot >= l || curRot <= r)))
    {
        //Serial.print("In forth if");
        return true;
    }
    //Within[-180,-15]deg,Turn Fast
    l = p180; r = n15;
    if ((l < r && curRot >= l && curRot <= r) ||
    (l > r && (curRot >= l || curRot <= r)))
    {
        //Serial.print("In fifth if");
        return true;
    }

    //Serial.print("default return");
    return true;
}

/**
 * @brief angle the robot should turn to (preferably 90 deg)
 * 
 * @param direction direction the robot should turn to in degres
*/
void TurnRobot90(int direction)
{
    float curRot = GetCurrentHeading();
    //Serial.printf("Heading deg: %f, target: %i\n", curRot);
    int p0 = direction;
    //Serial.printf("Targer: %f, Current: %f\n", direction, curRot);
    int p3 = (p0 + 5) % 360;
    int p15 = (p0 + 15) % 360;
    int n3 = (p0 - 5 + 360) % 360;
    int n15 = (p0 - 15 + 360) % 360;
    int p180 = (p0 + 180) % 360;
    int l = 0, r = 0;
    //Within(-3,+3)deg, stop turing.
    l = n3; r = p3;
    if ((l < r && curRot > l && curRot < r) ||
    (l > r && (curRot > l || curRot < r)))
    {
        Serial.println("STOP");
        engageMotors(STOP);
        return;
    }
    //Within[3,15]deg,Turn Slowly
    l = p3; r = p15;
    if ((l < r && curRot >= l && curRot <= r) ||
        (l > r && (curRot >= l || curRot <= r)))
    {
        engageMotors(LEFT, MotorTurnSpeedSlow);
        //move(1, -1);
        return;
    }
    //Within[15,180]deg,Turn Faast
    l = p15; r = p180;
    if ((l < r && curRot >= l && curRot <= r) ||
       (l > r && (curRot >= l || curRot <= r)))
    {
        //move(2, -2);
        engageMotors(LEFT, MotorTurnSpeedFast);
        // WheelLeft = 30;
        // WheelRight = -30;
        return;
    }
    //Within[-15,-3]deg,Turn Slowly
    l = n15; r = n3;
    if ((l < r && curRot >= l && curRot <= r) ||
    (l > r && (curRot >= l || curRot <= r)))
    {
        //move(-1, 1);
        engageMotors(RIGHT, MotorTurnSpeedSlow);
        //WheelLeft = -10;
        //WheelRight = 10;
        return;
    }
    //Within[-180,-15]deg,Turn Fast
    l = p180; r = n15;
    if ((l < r && curRot >= l && curRot <= r) ||
    (l > r && (curRot >= l || curRot <= r)))
    {
        //move(-2, 2);
        engageMotors(RIGHT, MotorTurnSpeedFast);
        // WheelLeft = -30;
        // WheelRight = 30;
        return;
    }
}

/**
 * @brief does a 90 deg turn
*/
void doTrun(int deg, uint8_t dir = RIGHT)
{
    int disCount = 0;
    int dis = 0;
    while(shouldTurn(deg))
    {   
        pauseRobot();
        TurnRobot90(deg);
    }
    engageMotors(STOP);
    delay(3000);
}

/**
 * @brief checks what color is under robot (for checking checkpoints, holes, etc...)
*/
void checkHole()
{
    if(checkForBlack() && numberOfBlackTiles == 3)
    { // if black turn 180 degres and invert heuristic (changes what side it prefers to turn to) to prevent going into loops
      Serial.println("Black");
      absoluteHeading = correctDegree(absoluteHeading + 180);
      doTrun(absoluteHeading);
      engageMotors(BACKWARD);
      delay(500);
      engageMotors(STOP);
      delay(750);
      dispensePallet();
      delay(750);
      resumeMotors();
    }
}

/**
 * @brief main movement loop for robot pref: right
*/
uint8_t timesTurned = 0;
void movementLoopR()
{
    if(detecting) return;

    if((GetDistance(LaserRightMLXPort) > TileWidth) && timesTurned == 0)
    {
        Serial.println("RIGHT");
        turning = true;
        timesTurned++;
        absoluteHeading = correctDegree(absoluteHeading+(TURN_RIGHT));
        doTrun(absoluteHeading, RIGHT);
        turning = false;
        delay(1000);
        return;
    }
    else
    {
        if(GetDistance(LaserForwardMLXPort) > TileWidth)
        {
            timesTurned = 0;
            GoForwardOneTile();
            return;
        }
        else
        {
            timesTurned = 0;
            Serial.println("LEFT");
            turning = true;
            absoluteHeading = correctDegree(absoluteHeading+(TURN_LEFT));
            doTrun(absoluteHeading, LEFT);
            turning = false;
            delay(1000);
            return;
        }
    }
}

/**
 * @brief main movement loop for robot pref: forward
*/
void movementLoopF()
{
    if(detecting) return;
    if(GetDistance(LaserForwardMLXPort) > 110)
    {
        goForward();
    }
    if(GetDistance(LaserRightMLXPort) < TileWidth) //wall right
    {
        while(true)
        {
            pauseRobot();
            if(detecting) break;
            
            if(GetDistance(LaserForwardMLXPort) < TileWidth) //wall forward
            {
                turning = true;
                Serial.println("LEFT");
                absoluteHeading = correctDegree(absoluteHeading+TURN_LEFT);
                doTrun(absoluteHeading);
                turning = false;
                engageMotors(STOP);
            }
            else
            {
                break;
            }
        }
    }
    else
    {
        Serial.println("RIGHT");
        
        turning = true;
        absoluteHeading = correctDegree(absoluteHeading+(TURN_RIGHT));
        doTrun(absoluteHeading);
        turning = false;
        engageMotors(STOP);
    }

    goForward();
}

/**
 * @brief drives forward till front wall
*/
void goForward()
{
    if(detecting) return;
    if(GetDistance(LaserForwardMLXPort) < TileWidth) 
    {
        Serial.println("Error: forward wall too close or too far away");
        return;
    }
    bool pastCurrentTileCenter = false;
    if(checkRamp())
        engageMotors(FORWARD);
    else
        engageMotors(FORWARD, 0, true);

    while(true)
    {
        if(detecting) break;

        Serial.println("F Loop");

        checkHole();
        pauseRobot();

        if(checkRamp())
            engageMotors(FORWARD);
        else
            engageMotors(FORWARD, 0, true);

        if(shouldTurn(absoluteHeading) && !checkRamp())
        {
            doTrun(absoluteHeading);
        }

        int distanceFromTileCenter = (GetDistance(LaserForwardMLXPort) + TOFForwardDistanceFromCenter - TileWidth / 2) % TileWidth;
        if(!pastCurrentTileCenter)
        {
            if(distanceFromTileCenter < (TileWidth / 2))
            {
                pastCurrentTileCenter = true;
            }
            Serial.println("First if");
        }
        if(distanceFromTileCenter <= 1 && pastCurrentTileCenter)
        {
            engageMotors(STOP);
            break;
            Serial.println("Second if");
        }
    }
    return;
}

/**
 * @brief goes forward 30cm
*/
void GoForwardOneTile()
{
    uint32_t currDistance = GetDistance(LaserForwardMLXPort);
    uint32_t target = 110;
    checkHole();
    if(currDistance >= TileWidth + target)
        target = currDistance - TileWidthOneTileDrive + WallThickness;

    if(currDistance < TileWidth) goForward();
    
    currDistance = GetDistance(LaserForwardMLXPort);
    while(currDistance > target)
    {
        if(checkRamp())
            engageMotors(FORWARD);
        else
            engageMotors(FORWARD, 0, true);

        pauseRobot();
        checkHole();

        if(GetDistance(LaserForwardMLXPort) < 120) break;
        currDistance = GetDistance(LaserForwardMLXPort);
    }
    engageMotors(STOP);
    delay(1000);
}

/**
 * @brief function that takes robot back to start
*/

void updateDetecting(bool val){
    detecting = val;
}