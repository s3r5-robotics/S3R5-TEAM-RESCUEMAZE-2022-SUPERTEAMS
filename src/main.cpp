#include <MovementFunctions.h>
#include <SensorLogic.h>

TaskHandle_t detectionTask;

void setup()
{
    Serial.begin(9600);
    delay(500);
    initialazeSensors();
    while(digitalRead(OtherButton));
    xTaskCreatePinnedToCore(
        detectionLoop,
        "MesurmentTask",
        10000,
        NULL,
        1,
        &detectionTask,
        0
    );
}

void loop()
{
    pauseRobot();

    movementLoopF();
}