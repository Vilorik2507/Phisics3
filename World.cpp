
#include "World.h"
#include "Collision.h"
#include "CCD.h"

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
    for (Box& box : boxes) {
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
    for (Box& box : boxes)
    {
        box.update(dt);
    }
}
