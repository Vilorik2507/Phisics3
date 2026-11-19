#include "MultiRate.h"

float MultiRate::getNormalDt() const
{
    return 1.f / normalFrequency;
}

float MultiRate::getFastDt() const
{
    return 1.f / fastFrequency;
}

void MultiRate::addFrameTime(float frameDt)
{
    fastAccumulator += frameDt;
}

bool MultiRate::shouldUpdateFast()
{
    if (fastAccumulator >= getFastDt())
    {
        fastAccumulator -= getFastDt();
        return true;
    }

    return false;
}

bool MultiRate::shouldUpdateNormal()
{
    fastStepsSinceNormal++;

    if (fastStepsSinceNormal >= 4)
    {
        fastStepsSinceNormal = 0;
        return true;
    }

    return false;
}
