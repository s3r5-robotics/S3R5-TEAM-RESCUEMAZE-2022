#include <MovementFunctions.h>
#include <SensorLogic.h>

TaskHandle_t detectionTask;

void setup()
{
  Serial.begin(9600);
  delay(500);
  initHeap();
  initialazeSensors();
  engageMotors(STOP);
  while(digitalRead(BlackButton));
  xTaskCreatePinnedToCore(
    detectionLoop,
    "MeasurmentTask",
    10000,
    NULL,
    1,
    &detectionTask,
    0
  );
  attachInterrupt(WhiteButton, buttonInterrupt, CHANGE);
}

void loop()
{
  movementLoopF();
}