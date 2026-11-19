#pragma once

#include "PhysicsGroup.h"

class MultiRate
{
private:
    float normalFrequency = 60.f;
    float fastFrequency = 240.f;

    float fastAccumulator = 0.f;

    int fastStepsSinceNormal = 0;

public:
    float getNormalDt() const;
    float getFastDt() const;

    void addFrameTime(float frameDt);

    bool shouldUpdateFast();
    bool shouldUpdateNormal();
};
