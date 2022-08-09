#pragma once

#define NONE 0
#define HEAT 1
#define RED 2
#define YELLOW 3
#define GREEN 4
#define VICTIM_H 5
#define VICTIM_S 6
#define VICTIM_U 7
#define HOLE 8

#define FACING_Y_POS 0 // 0
#define FACING_X_NEG 1 // 90 / 90 = 1
#define FACING_Y_NEG 2 // 180 / 90 = 2
#define FACING_X_POS 3 // 270 / 90 = 3

struct PositionInfo {
    uint8_t info;
    int16_t x;
    int16_t y;
} ;

int16_t x = 0, y = 0;
uint16_t facing = 0;
uint16_t orientation = 0;
uint8_t positionInfoCount = 0;

PositionInfo positionInfo[20];

void checkOrientation(uint16_t direction);

void updatePosition();

PositionInfo checkPositionInfo();

bool addNewPositionInfo(PositionInfo info);

/**
 * @brief updates the cooridnate system based on witch direction the robot is facing
*/
void updatePosition()
{
    switch(facing)
    {
        case FACING_Y_POS:
        y += 1;
        break;
        case FACING_Y_NEG:
        y -= 1;
        break;
        case FACING_X_POS:
        x += 1;
        break;
        case FACING_X_NEG:
        x -= 1;
        break;
    }
    Serial.printf("P(%i, %i)\n", x, y);
}

/**
 * @brief adds new position info to the table of position info (victim, hole, etc...)
 * 
 * @param info position inf (must provide x, y, and type of info (check definitions))
 * 
 * @returns boolean (true if added false if not)
*/

bool addNewPositionInfo(PositionInfo info)
{
    if(positionInfoCount >= 20) return false;
    positionInfo[positionInfoCount].info = info.info;
    positionInfo[positionInfoCount].x = info.x;
    positionInfo[positionInfoCount].y = info.y;
    positionInfoCount++;
    return true;
}

/**
 * @brief checks the orientation and writes witch direction the robot is facing
 * 
 * @param direction the direction the robot is facing (must be in 90 degree increments form 0 - 270) default is 0
*/
void checkOrientation(uint16_t direction = 0)
{
    orientation = direction;
    facing = direction/90;
}

/**
 * @brief gets the info if any at all at robots position
 * 
 * @returns info and cooridnates of the requested info or 0 if nothing is sotred there
*/
PositionInfo checkPositionInfo()
{
    for(uint8_t i = 0; i < positionInfoCount; i++)
    {
        if(positionInfo[i].x == x && positionInfo[i].y == y)
            return positionInfo[i];
    }
    return {NONE, x, y};
}