
#include "World.h"
#include "Collision.h"
#include "CCD.h"
#include "MultiRate.h"

void World::addBall(const Ball& ball)
{
    balls.push_back(ball);
}
void World::addWall(const Wall& wall) {
    walls.push_back(wall);
}
void World::addBox(const Box& box)
{
    boxes.push_back(box);
}
void World::update(float dt, float restitution)
{
    // Обновляем положение шариков
    for (Ball& ball : balls)
    {
        ball.update(dt);
    }
    for (Box& box : boxes)
    {
        box.update(dt);
    }
    // Проверяем столкновения шариков друг с другом
    for (size_t i = 0; i < balls.size(); i++)
    {
        for (size_t j = i + 1; j < balls.size(); j++)
        {
            resolveCollision(
                balls[i],
                balls[j],
                restitution
            );
        }
    }

    // Проверяем столкновения шариков со стенами
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

void World::updateWithCCD(float dt, float restitution)
{
    for (Ball& ball : balls)
    {
        if (walls.empty())
        {
            ball.update(dt);
            continue;
        }

        // Пока проверяем первую стену
        CCD::updateBallWithCCD(
            ball,
            walls,
            dt,
            restitution
        );
    }

    // Проверяем столкновения между шариками
    for (size_t i = 0; i < balls.size(); i++)
    {
        for (size_t j = i + 1; j < balls.size(); j++)
        {
            resolveCollision(balls[i], balls[j], restitution);
        }
    }
}


void World::draw(sf::RenderWindow& window) const
{
    for (const Ball& ball : balls)
    {
        ball.draw(window);
    }
    for (const Wall& wall : walls) {
        wall.draw(window);
    }
    for (const Box& box : boxes) {
        box.draw(window);
    }
}


void World::updateWithFullCCD(float dt, float restitution)
{
    CCD::updateBallsWithCCD(
        balls,
        walls,
        dt,
        restitution
    );

    const float gravity = 980.f;

    const int subSteps = 8;
    const float subDt = dt / subSteps;

    const int solverIterations = 10;

    for (int step = 0; step < subSteps; step++)
    {
        // Гравитация и CCD для каждой коробки
        for (Box& box : boxes)
        {
            sf::Vector2f velocity = box.getVelocity();
            velocity.y += gravity * subDt;

            box.setVelocity(velocity);

            updateBoxWithWallCCD(
                box,
                walls,
                subDt,
                restitution
            );
        }

        for (int iteration = 0; iteration < solverIterations; iteration++)
        {
            for (Box& box : boxes)
            {
                for (const Wall& wall : walls)
                {
                    resolveBoxWallCollision(box, wall, restitution);
                }
            }

            for (size_t i = 0; i < boxes.size(); i++)
            {
                for (size_t j = i + 1; j < boxes.size(); j++)
                {
                    resolveBoxBoxCollision(
                        boxes[i],
                        boxes[j],
                        restitution
                    );
                }
            }
        }
    }
}


void World::updatePhysicsStep(
    float dt,
    float restitution,
    PhysicsGroup group,
    bool updateVelocity,
    bool updatePosition)
{
    const float gravity = 980.f;

    for (Box& box : boxes)
    {
        if (box.getGroup() != group)
            continue;

        if (updateVelocity)
        {
            sf::Vector2f velocity = box.getVelocity();
            velocity.y += gravity * dt;
            box.setVelocity(velocity);
        }

        if (updatePosition)
        {
            sf::Vector2f position = box.getPosition();
            position += box.getVelocity() * dt;
            box.setPosition(position);
        }
    }

    for (Ball& ball : balls)
    {
        if (ball.getGroup() != group)
            continue;

        if (updateVelocity)
        {
            sf::Vector2f velocity = ball.getVelocity();
            velocity.y += gravity * dt;
            ball.setVelocity(velocity);
        }

        if (updatePosition)
        {
            sf::Vector2f position = ball.getPosition();
            position += ball.getVelocity() * dt;
            ball.setPosition(position);
        }
    }
}


void World::updateMultiRate(float frameDt, float restitution)
{
    multiRate.addFrameTime(frameDt);

    while (multiRate.shouldUpdateFast())
    {
        const float fastDt = multiRate.getFastDt();

        // Fast: скорость и положение обновляются на каждом шаге.
        updatePhysicsStep(
            fastDt,
            restitution,
            PhysicsGroup::Fast,
            true,
            true
        );

        // Normal: скорость обновляется раз в четыре шага.
        if (multiRate.shouldUpdateNormal())
        {
            updatePhysicsStep(
                multiRate.getNormalDt(),
                restitution,
                PhysicsGroup::Normal,
                true,
                false
            );
        }

        // Положение Normal продвигается на общей сетке 240 Гц.
        updatePhysicsStep(
            fastDt,
            restitution,
            PhysicsGroup::Normal,
            false,
            true
        );

        // Разрешаем столкновения коробок.
        const int solverIterations = 4;

        for (int iteration = 0; iteration < solverIterations; iteration++)
        {
            for (size_t i = 0; i < boxes.size(); i++)
            {
                for (size_t j = i + 1; j < boxes.size(); j++)
                {
                    resolveBoxBoxCollision(
                        boxes[i],
                        boxes[j],
                        restitution
                    );
                }
            }

            for (Box& box : boxes)
            {
                for (const Wall& wall : walls)
                {
                    resolveBoxWallCollision(
                        box,
                        wall,
                        restitution
                    );
                }
            }
        }
    }
}
