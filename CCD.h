#pragma once

#include "Ball.h"
#include "Wall.h"
#include <vector>

class CCD
{
public:
    static bool checkBallWallCCD(
        const Ball& ball,
        const Wall& wall,
        float dt,
        float& collisionTime
    );

    static void updateBallWithCCD(
        Ball& ball,
        const std::vector<Wall>& walls,
        float dt,
        float restitution
    );
    static bool checkBallBallCCD(const Ball& a, const Ball& b, float dt, float& collisionTime);

    static void updateBallPairWithCCD(Ball& a, Ball& b, float dt, float restitution);

    static void updateBallsWithCCD(
        std::vector<Ball>& balls,
        const std::vector<Wall>& walls,
        float dt,
        float restitution
    );
    static void solveInitialOverlaps(std::vector<Ball>& balls, const std::vector<Wall>& walls);
};