#pragma once
#include <Arduino.h>
#include <ProgramVariables.h>
#include <SensorObjects&Functions.h>
#include <dataHeap.h>
#include <Mapping.h>

int absoluteHeading = 0;
uint16_t avgDist = 0;
uint8_t count = 0;
int heu = 1; // heu -> heuristic is what side it prefers so if heuristic is 1 it prefers right if its -1 it prefers left
Node_heap_s positionHeap;
bool detecting = false; // zato ker je detection v drugem loopu da pausa robota
bool turning = false;
bool paused = false;

int shouldTurn(int targetRot);

void TurnRobot90(int direction);

void avoidHole();

void movementLoop();

void initHeap();

void goForward();

void goForwardR();

void GoForwardOneTile();

void realign();

void CheckAlignment();

void returnToStart();

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
    checkOrientation(deg);
    while(shouldTurn(deg))
    {
        TurnRobot90(deg);
    }
    engageMotors(STOP);
}

/**
 * @brief checks what color is under robot (for checking checkpoints, holes, etc...)
*/
void avoidHole()
{
  uint16_t photoReadout = readPhotoResistor();
  if((BlackLow < photoReadout && photoReadout < BlackHigh))
  { // if black turn 180 degres and invert heuristic (changes what side it prefers to turn to) to prevent going into loops
    Serial.println("Black");
    absoluteHeading = correctDegree(absoluteHeading + 180);
    PositionData position = { absoluteHeading, 0 };
    push(&positionHeap, position);
    engageMotors(BACKWARD);
    delay(200);
    engageMotors(STOP);
    doTrun(absoluteHeading);
  }
}

/**
 * @brief main movement loop for robot pref: right
*/
uint8_t timesTurned = 0;
void movementLoopR()
{
    PositionData position;
    if(detecting) return;

    if((GetDistance(LaserRightMLXPort) > TileWidth) && timesTurned == 0)
    {
        Serial.println("RIGHT");
        turning = true;
        timesTurned++;
        absoluteHeading = correctDegree(absoluteHeading+(TURN_RIGHT*heu));
        position.heading = absoluteHeading;
        position.stepForward = 0;
        push(&positionHeap, position);
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
            position.heading = absoluteHeading;
            position.stepForward = 1;
            push(&positionHeap, position);
            GoForwardOneTile();
            return;
        }
        else
        {
            timesTurned = 0;
            Serial.println("LEFT");
            turning = true;
            absoluteHeading = correctDegree(absoluteHeading+(TURN_LEFT*heu));
            position.heading = absoluteHeading;
            position.stepForward = 0;
            push(&positionHeap, position);
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
    PositionData position;
    if(detecting) return;
    int rand = GetDistance(LaserForwardMLXPort)%2;
    if(rand == 1) heu = 1; else heu = -1;
    if(GetDistance(heu == 1 ? LaserRightMLXPort : LaserLeftMLXPort) < TileWidth) //wall right
    {
        while(true)
        {
            if(detecting) break;
            
            if(GetDistance(LaserForwardMLXPort) < TileWidth) //wall forward
            {
                turning = true;
                Serial.println("LEFT");
                absoluteHeading = correctDegree(absoluteHeading+(TURN_LEFT*heu));
                position.heading = absoluteHeading;
                position.stepForward = 0;
                push(&positionHeap, position);
                while(shouldTurn(absoluteHeading))
                {
                    
                    TurnRobot90(absoluteHeading);
                }
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
        absoluteHeading = correctDegree(absoluteHeading+(TURN_RIGHT*heu));
        position.heading = absoluteHeading;
        position.stepForward = 0;
        push(&positionHeap, position);
        while(shouldTurn(absoluteHeading))
        {
            TurnRobot90(absoluteHeading);
        }
        turning = false;
        engageMotors(STOP);
    }
    position.heading = absoluteHeading;
    position.stepForward = 1;
    if(GetDistance(LaserForwardMLXPort) > TileWidth) 
        push(&positionHeap, position);
    goForward();
    //realign();
}

/**
 * @brief initializes the position heap
*/
void initHeap()
{
    positionHeap.first = NULL;
    positionHeap.last = NULL;
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
        }
        if(distanceFromTileCenter <= 1 && pastCurrentTileCenter)
        {
            if(checkRamp())
                engageMotors(FORWARD);
            else
                engageMotors(FORWARD, 0, true);
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
    avoidHole();
    if(currDistance >= TileWidth + target)
        target = currDistance - TileWidthOneTileDrive + WallThickness;

    if(currDistance < TileWidth) goForward();
    
    currDistance = GetDistance(LaserForwardMLXPort);
    updatePosition();
    while(currDistance > target)
    {
        if(checkRamp())
            engageMotors(FORWARD);
        else
            engageMotors(FORWARD, 0, true);
        avoidHole();
        if(GetDistance(LaserForwardMLXPort) < 120) break;
        currDistance = GetDistance(LaserForwardMLXPort);
    }
    engageMotors(STOP);
    delay(1000);
}

/**
 * @brief realigns the robot
*/
void realign()
{
    while(shouldTurn(absoluteHeading))
    {
        TurnRobot90(absoluteHeading);
    }
}

/**
 * @brief function that takes robot back to start
*/
void returnToStart()
{
    PositionData data = pop(&positionHeap);
    while(!(data.heading == 1000 && data.stepForward == 1000))
    {

        int angle = correctDegree((int)(180-(int)data.heading));
        while(shouldTurn(angle))
        {
                
            TurnRobot90(angle);
        }

        for(int i = 0; i < data.stepForward; i++)
        {
            GoForwardOneTile();
        }

        data = pop(&positionHeap);
    }
}

void updateDetecting(bool val){
    detecting = val;
}