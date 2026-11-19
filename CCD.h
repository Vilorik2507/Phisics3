#pragma once

#include "Ball.h"
#include "Box.h"
#include "Wall.h"
#include "PhysicsGroup.h"

#include <vector>

namespace CCD
{
    void updateBallWithCCD(
        Ball& ball,
        const std::vector<Wall>& walls,
        float dt,
        float restitution
    );

    void updateBallsWithCCD(
        std::vector<Ball>& balls,
        const std::vector<Wall>& walls,
        float dt,
        float restitution,
        PhysicsGroup group
    );

    void updateBoxesWithCCD(
        std::vector<Box>& boxes,
        const std::vector<Wall>& walls,
        float dt,
        float restitution,
        PhysicsGroup group
    );

    // Общий CCD: учитывает Normal и Fast одновременно.
    void updateAllBallsWithCCD(
        std::vector<Ball>& balls,
        const std::vector<Wall>& walls,
        float dt,
        float restitution
    );

    void updateAllBoxesWithCCD(
        std::vector<Box>& boxes,
        const std::vector<Wall>& walls,
        float dt,
        float restitution
    );
}