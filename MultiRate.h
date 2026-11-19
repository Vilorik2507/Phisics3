
#pragma once

class MultiRate
{
private:
    static constexpr double NORMAL_FREQUENCY = 60.0;
    static constexpr double FAST_FREQUENCY = 240.0;

    static constexpr double NORMAL_DT =
        1.0 / NORMAL_FREQUENCY;

    static constexpr double FAST_DT =
        1.0 / FAST_FREQUENCY;

    static constexpr int FAST_STEPS_PER_NORMAL = 4;

    double accumulator = 0.0;

    int fastStepsPerFrame = 0;
    int fastStepsTotal = 0;
    int normalStepsTotal = 0;

    int fastStepsSinceNormal = 0;

public:
    void beginFrame(float frameDt);

    bool hasFastStep() const;

    void consumeFastStep();

    bool isNormalUpdateDue() const;

    void consumeNormalStep();

    float getFastDt() const;
    float getNormalDt() const;

    int getFastStepsThisFrame() const;
    int getFastStepsTotal() const;
    int getNormalStepsTotal() const;

    float getRemainingTime() const;

    void reset();
};
