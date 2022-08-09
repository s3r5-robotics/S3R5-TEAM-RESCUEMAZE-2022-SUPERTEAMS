#pragma once
#include <SensorObjects&Functions.h>
#include <MovementFunctions.h>

// pallets must be dropped for RED 1, YELLOW 1, H 3, S 2
uint8_t victimsFound = 0;

bool prevBlackReading = true;

bool victimHeat(int side = LEFT);

void haltVictim(int palletNum = 0, uint8_t side = LEFT);

void finish();

/**
 * @brief brains of the sensors of the robot; Victim detection, pallet dispension...
*/
void TakeMesurements()
{
    //tickGyro();
    if(checkForBlack() && prevBlackReading == false)
    {
        numberOfBlackTiles++;
    }

    int CameraOutputLeft = -1;// getCameraOutput(OpenMVCameraLeft);
    int CameraOutputRight = -1;// getCameraOutput(OpenMVCameraRight);

    if(GetDistance(LaserLeftMLXPort) < 110){
        CameraOutputLeft = getCameraOutput(OpenMVCameraLeft);
        if(victimHeat(TempLeftMLXPort))
        {
            haltVictim(1, LEFT);
        }
    }
    if(GetDistance(LaserRightMLXPort) < 110){
        CameraOutputRight = getCameraOutput(OpenMVCameraRight);
        if(victimHeat(TempRightMLXPort))
        {
            haltVictim(1, RIGHT);
        }
    }

    
    switch(CameraOutputLeft)
    {
        case 4: // R
        haltVictim(1);
        break;
        case 5: // G
        haltVictim(0);
        break;
        case 6: // Y
        haltVictim(1);
        break;
    }

    switch(CameraOutputRight)
    {
        case 4: // R
        haltVictim(1, RIGHT);
        break;
        case 5: // G
        haltVictim(0, RIGHT);
        break;
        case 6: // Y
        haltVictim(1, RIGHT);
        break;
    }

    if(victimsFound >= MaxVictimNum)
    {
        detecting = true;
        while(true) engageMotors(STOP);
    }

    prevBlackReading = checkForBlack();
}

/**
 * @brief stops for 5 seconds turn on led and dispenses number of pellets equal to palletNum
 * 
 * @param palletNum number of pallets to dispense
*/
void haltVictim(int palletNum, uint8_t side)
{
    Serial.printf("Detected %i \n", side);
    detecting = true; // stop movement
    delay(100);
    engageMotors(STOP);
    blinkLED();
    detecting = false; // start movement again
    resumeMotors();
    delay(2000);
}

/**
 * @brief check if there is a heated victim
*/
bool victimHeat(int side)
{
    float temp = getTemperature(side);
    return ((float)TempLow < temp && temp < (float)TempHigh);
}

/**
 * @brief signal that the robot has reached the finish
*/
void finish()
{
    toggleLED();
    delay(500);
    toggleLED(OFF);
    delay(500);
    toggleLED();
    delay(500);
    toggleLED(OFF);
}


/**
 * @brief function that runs on second core used for victim detection
*/
void detectionLoop(void * parameters){
    delay(500);
    for(;;){
        if(paused == true) continue;
        TakeMesurements();
        delay(100);
        //Serial.printf("Okay detecting \n");
    }
}