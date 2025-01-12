#ifndef PROJECT_UNIFICATION_CONSTANTS_H
#define PROJECT_UNIFICATION_CONSTANTS_H

#include "soccer-common/Extends/QPoint/ExtendsQPoint.h"
const float WORLD_WIDTH = 9000.0;  // FIELD_LENGTH
const float WORLD_HEIGHT = 6000.0; // FIELD_WIDTH
const float BOT_RADIUS = 90.0;
const float START_POS_X = 0.0; // 1497.57;
const float START_ORIENT = 0.785;
const float START_POS_Y = 0.0; // 1120;
const float END_POS_X = 3500.0;
const float END_POS_Y = 0.0;
const float NODE_RADIUS = 1.0;
const float END_DIST_THRESHOLD = 100.0;
const float BOT_CLEARANCE = 1 * BOT_RADIUS;
const float BOT_TURN_RADIUS = 7;
const float RRTSTAR_NEIGHBOR_FACTOR = 6;
const bool BOT_FOLLOW_DUBIN = false;
const double GOAL_SAMPLING_PROB = 0.05;
const int RADIUS = 5;

void setInitialPos(Point initialPos);

#endif // PROJECT_UNIFICATION_CONSTANTS_H
