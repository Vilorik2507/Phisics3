
#include "MultiRate.h"

#include <cmath>

// =====================================================
// ДОБАВЛЕНИЕ ВРЕМЕНИ КАДРА
// =====================================================

void MultiRate::beginFrame(float frameDt)
{
    fastStepsPerFrame = 0;

    if (!std::isfinite(frameDt) || frameDt <= 0.f)
        return;

    accumulator += static_cast<double>(frameDt);
}

// =====================================================
// ПРОВЕРКА НАЛИЧИЯ FAST-ШАГА
// =====================================================

bool MultiRate::hasFastStep() const
{
    constexpr double tolerance = 1e-9;

    return accumulator + tolerance >= FAST_DT;
}

// =====================================================
// ВЫПОЛНЕНИЕ FAST-ШАГА
// =====================================================

void MultiRate::consumeFastStep()
{
    if (!hasFastStep())
        return;

    accumulator -= FAST_DT;

    if (accumulator < 0.0)
        accumulator = 0.0;

    ++fastStepsPerFrame;
    ++fastStepsTotal;
    ++fastStepsSinceNormal;
}

// =====================================================
// ПРОВЕРКА НЕОБХОДИМОСТИ NORMAL-ШАГА
// =====================================================

bool MultiRate::isNormalUpdateDue() const
{
    return fastStepsSinceNormal >= FAST_STEPS_PER_NORMAL;
}

// =====================================================
// ВЫПОЛНЕНИЕ NORMAL-ШАГА
// =====================================================

void MultiRate::consumeNormalStep()
{
    if (!isNormalUpdateDue())
        return;

    fastStepsSinceNormal -= FAST_STEPS_PER_NORMAL;

    ++normalStepsTotal;
}

// =====================================================
// ИНТЕРВАЛЫ
// =====================================================

float MultiRate::getFastDt() const
{
    return static_cast<float>(FAST_DT);
}

float MultiRate::getNormalDt() const
{
    return static_cast<float>(NORMAL_DT);
}

// =====================================================
// СТАТИСТИКА
// =====================================================

int MultiRate::getFastStepsThisFrame() const
{
    return fastStepsPerFrame;
}

int MultiRate::getFastStepsTotal() const
{
    return fastStepsTotal;
}

int MultiRate::getNormalStepsTotal() const
{
    return normalStepsTotal;
}

// =====================================================
// ОСТАТОК НАКОПЛЕННОГО ВРЕМЕНИ
// =====================================================

float MultiRate::getRemainingTime() const
{
    return static_cast<float>(accumulator);
}

// =====================================================
// СБРОС
// =====================================================

void MultiRate::reset()
{
    accumulator = 0.0;

    fastStepsPerFrame = 0;
    fastStepsTotal = 0;
    normalStepsTotal = 0;

    fastStepsSinceNormal = 0;
}
