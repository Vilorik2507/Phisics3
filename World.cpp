
#include "World.h"

#include "Collision.h"
#include "CCD.h"

#include <cstddef>

// =====================================================
// ДОБАВЛЕНИЕ ОБЪЕКТОВ
// =====================================================

void World::addBall(const Ball& ball)
{
    balls.push_back(ball);
    shadowInitialized = false;
}

void World::addWall(const Wall& wall)
{
    walls.push_back(wall);
    shadowInitialized = false;
}

void World::addBox(const Box& box)
{
    boxes.push_back(box);
    shadowInitialized = false;
}

// =====================================================
// ГРАВИТАЦИЯ
// =====================================================

void World::applyGravity(PhysicsGroup group, float dt)
{
    if (group == PhysicsGroup::Static)
        return;

    for (Ball& ball : balls)
    {
        if (ball.getGroup() == group)
        {
            ball.setVelocity(
                ball.getVelocity() + gravity * dt
            );
        }
    }

    for (Box& box : boxes)
    {
        if (box.getGroup() == group)
        {
            box.setVelocity(
                box.getVelocity() + gravity * dt
            );
        }
    }
}

// =====================================================
// ОБНОВЛЕНИЕ ВЫБРАННОЙ ГРУППЫ
// =====================================================

void World::updateGroup(PhysicsGroup group, float dt)
{
    if (group == PhysicsGroup::Static)
        return;

    for (Ball& ball : balls)
    {
        if (ball.getGroup() == group)
            ball.update(dt);
    }

    for (Box& box : boxes)
    {
        if (box.getGroup() == group)
            box.update(dt);
    }
}

// =====================================================
// СТОЛКНОВЕНИЯ ШАРОВ
// =====================================================

void World::solveBallCollisions(
    float restitution,
    int iterations
)
{
    for (int iteration = 0; iteration < iterations; ++iteration)
    {
        for (std::size_t i = 0; i < balls.size(); ++i)
        {
            for (std::size_t j = i + 1; j < balls.size(); ++j)
            {
                resolveCollision(
                    balls[i],
                    balls[j],
                    restitution
                );
            }
        }

        for (Ball& ball : balls)
        {
            for (const Wall& wall : walls)
            {
                resolveBallWallCollision(
                    ball,
                    wall,
                    restitution
                );
            }
        }
    }
}

// =====================================================
// СТОЛКНОВЕНИЯ ПРЯМОУГОЛЬНИКОВ
// =====================================================

void World::solveBoxCollisions(
    float restitution,
    int iterations
)
{
    ::solveBoxCollisions(
        boxes,
        walls,
        restitution,
        iterations
    );
}

// =====================================================
// СИНХРОНИЗАЦИЯ КОНТАКТОВ
// =====================================================

void World::synchronizeBodies(
    float /*dt*/,
    float restitution
)
{
    solveBallCollisions(restitution, 2);
    solveBoxCollisions(restitution, 2);
}

// =====================================================
// ОБЫЧНАЯ СИМУЛЯЦИЯ
// =====================================================

void World::update(float dt, float restitution)
{
    applyGravity(PhysicsGroup::Normal, dt);
    applyGravity(PhysicsGroup::Fast, dt);

    updateGroup(PhysicsGroup::Normal, dt);
    updateGroup(PhysicsGroup::Fast, dt);

    synchronizeBodies(dt, restitution);

    // При переходе на другой режим создаём
    // теневое состояние заново.
    shadowInitialized = false;
}

// =====================================================
// CCD ДЛЯ ШАРОВ
// =====================================================

void World::updateWithCCD(float dt, float restitution)
{
    applyGravity(PhysicsGroup::Normal, dt);
    applyGravity(PhysicsGroup::Fast, dt);

    CCD::updateBallsWithCCD(
        balls,
        walls,
        dt,
        restitution,
        PhysicsGroup::Normal
    );

    CCD::updateBallsWithCCD(
        balls,
        walls,
        dt,
        restitution,
        PhysicsGroup::Fast
    );

    updateGroup(PhysicsGroup::Normal, dt);
    updateGroup(PhysicsGroup::Fast, dt);

    solveBoxCollisions(restitution, 6);
    solveBallCollisions(restitution, 6);

    shadowInitialized = false;
}

// =====================================================
// ПОЛНАЯ CCD-СИМУЛЯЦИЯ
// =====================================================

void World::updateWithFullCCD(
    float dt,
    float restitution
)
{
    applyGravity(PhysicsGroup::Normal, dt);
    applyGravity(PhysicsGroup::Fast, dt);

    CCD::updateBallsWithCCD(
        balls,
        walls,
        dt,
        restitution,
        PhysicsGroup::Normal
    );

    CCD::updateBallsWithCCD(
        balls,
        walls,
        dt,
        restitution,
        PhysicsGroup::Fast
    );

    CCD::updateBoxesWithCCD(
        boxes,
        walls,
        dt,
        restitution,
        PhysicsGroup::Normal
    );

    CCD::updateBoxesWithCCD(
        boxes,
        walls,
        dt,
        restitution,
        PhysicsGroup::Fast
    );

    synchronizeBodies(dt, restitution);

    shadowInitialized = false;
}

// =====================================================
// ИНИЦИАЛИЗАЦИЯ ТЕНЕВОГО СОСТОЯНИЯ
// =====================================================

void World::initializeShadowState()
{
    shadowBalls = balls;
    shadowBoxes = boxes;

    shadowInitialized = true;
}

// =====================================================
// ОБЩЕЕ ФИЗИЧЕСКОЕ ОБНОВЛЕНИЕ ТЕНЕВОГО СОСТОЯНИЯ
// =====================================================

void World::updateShadowState(
    float fastDt,
    float restitution
)
{
    if (fastDt <= 0.f)
        return;

    // Гравитация применяется к обеим динамическим группам.
    // Статические объекты не изменяются.

    for (Ball& ball : shadowBalls)
    {
        if (ball.getGroup() != PhysicsGroup::Static)
        {
            ball.setVelocity(
                ball.getVelocity() + gravity * fastDt
            );
        }
    }

    for (Box& box : shadowBoxes)
    {
        if (box.getGroup() != PhysicsGroup::Static)
        {
            box.setVelocity(
                box.getVelocity() + gravity * fastDt
            );
        }
    }

    // Глобальный CCD:
    // Normal-Normal, Fast-Fast и Normal-Fast.
    // Все тела одной категории синхронизируются
    // на общем временном интервале.

    CCD::updateAllBallsWithCCD(
        shadowBalls,
        walls,
        fastDt,
        restitution
    );

    CCD::updateAllBoxesWithCCD(
        shadowBoxes,
        walls,
        fastDt,
        restitution
    );
}

// =====================================================
// ПЕРЕНОС FAST В ОСНОВНОЕ СОСТОЯНИЕ
// =====================================================

void World::copyFastBodiesFromShadow()
{
    for (std::size_t i = 0; i < balls.size(); ++i)
    {
        if (balls[i].getGroup() != PhysicsGroup::Fast)
            continue;

        balls[i].setPosition(
            shadowBalls[i].getPosition()
        );

        balls[i].setVelocity(
            shadowBalls[i].getVelocity()
        );
    }

    for (std::size_t i = 0; i < boxes.size(); ++i)
    {
        if (boxes[i].getGroup() != PhysicsGroup::Fast)
            continue;

        boxes[i].setPosition(
            shadowBoxes[i].getPosition()
        );

        boxes[i].setVelocity(
            shadowBoxes[i].getVelocity()
        );
    }
}

// =====================================================
// ПЕРЕНОС NORMAL В ОСНОВНОЕ СОСТОЯНИЕ
// =====================================================

void World::copyNormalBodiesFromShadow()
{
    for (std::size_t i = 0; i < balls.size(); ++i)
    {
        if (balls[i].getGroup() != PhysicsGroup::Normal)
            continue;

        balls[i].setPosition(
            shadowBalls[i].getPosition()
        );

        balls[i].setVelocity(
            shadowBalls[i].getVelocity()
        );
    }

    for (std::size_t i = 0; i < boxes.size(); ++i)
    {
        if (boxes[i].getGroup() != PhysicsGroup::Normal)
            continue;

        boxes[i].setPosition(
            shadowBoxes[i].getPosition()
        );

        boxes[i].setVelocity(
            shadowBoxes[i].getVelocity()
        );
    }
}

// =====================================================
// MULTI-RATE
// NORMAL: 60 ГЦ
// FAST:   240 ГЦ
// =====================================================

void World::updateMultiRate(
    float frameDt,
    float restitution
)
{
    if (!shadowInitialized)
        initializeShadowState();

    multiRate.beginFrame(frameDt);

    while (multiRate.hasFastStep())
    {
        const float fastDt = multiRate.getFastDt();

        multiRate.consumeFastStep();

        // 1. Общая физика и поиск TOI.
        updateShadowState(
            fastDt,
            restitution
        );

        // 2. Fast переносится после каждого шага 240 Гц.
        copyFastBodiesFromShadow();

        // 3. Normal переносится только после четырёх
        //    шагов Fast, то есть с частотой 60 Гц.
        if (multiRate.isNormalUpdateDue())
        {
            copyNormalBodiesFromShadow();

            multiRate.consumeNormalStep();
        }
    }
}

// =====================================================
// ОТРИСОВКА
// =====================================================

void World::draw(sf::RenderWindow& window) const
{
    for (const Wall& wall : walls)
        wall.draw(window);

    for (const Box& box : boxes)
        box.draw(window);

    for (const Ball& ball : balls)
        ball.draw(window);
}

// =====================================================
// СТАТИСТИКА
// =====================================================

int World::getFastStepsThisFrame() const
{
    return multiRate.getFastStepsThisFrame();
}

int World::getFastStepsTotal() const
{
    return multiRate.getFastStepsTotal();
}

int World::getNormalStepsTotal() const
{
    return multiRate.getNormalStepsTotal();
}
