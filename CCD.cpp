
#include "CCD.h"
#include "Collision.h"
#include <cmath>
#include <algorithm>
#include <iostream>


bool CCD::checkBallWallCCD(
    const Ball& ball,
    const Wall& wall,
    float dt,
    float& collisionTime
)
{
    sf::Vector2f position = ball.getPosition();
    sf::Vector2f velocity = ball.getVelocity();

    float radius = ball.getRadius();

    float left = wall.getPosition().x;
    float right = left + wall.getSize().x;

    float top = wall.getPosition().y;
    float bottom = top + wall.getSize().y;

    float earliestTime = dt + 1.f;
    bool hit = false;

    // Проверка левой и правой граней
    if (velocity.x > 0.f)
    {
        float time = (left - radius - position.x) / velocity.x;

        if (time >= 0.f && time <= dt)
        {
            float y = position.y + velocity.y * time;

            if (y >= top - radius && y <= bottom + radius)
            {
                earliestTime = time;
                hit = true;
            }
        }
    }

    if (velocity.x < 0.f)
    {
        float time = (right + radius - position.x) / velocity.x;

        if (time >= 0.f && time <= dt && time < earliestTime)
        {
            float y = position.y + velocity.y * time;

            if (y >= top - radius && y <= bottom + radius)
            {
                earliestTime = time;
                hit = true;
            }
        }
    }

    // Проверка верхней и нижней граней
    if (velocity.y > 0.f)
    {
        float time = (top - radius - position.y) / velocity.y;

        if (time >= 0.f && time <= dt && time < earliestTime)
        {
            float x = position.x + velocity.x * time;

            if (x >= left - radius && x <= right + radius)
            {
                earliestTime = time;
                hit = true;
            }
        }
    }

    if (velocity.y < 0.f)
    {
        float time = (bottom + radius - position.y) / velocity.y;

        if (time >= 0.f && time <= dt && time < earliestTime)
        {
            float x = position.x + velocity.x * time;

            if (x >= left - radius && x <= right + radius)
            {
                earliestTime = time;
                hit = true;
            }
        }
    }

    if (hit)
    {
        collisionTime = earliestTime;
        return true;
    }

    return false;
}



void CCD::updateBallWithCCD(
    Ball& ball,
    const std::vector<Wall>& walls,
    float dt,
    float restitution
)
{
    float remainingTime = dt;

    const int maxCollisions = 8;

    for (int iteration = 0; iteration < maxCollisions; iteration++)
    {
        if (remainingTime <= 0.000001f)
            return;

        float earliestTime = remainingTime;
        int hitWallIndex = -1;

        for (size_t i = 0; i < walls.size(); i++)
        {
            float collisionTime = 0.f;

            bool hit = checkBallWallCCD(
                ball,
                walls[i],
                remainingTime,
                collisionTime
            );

            if (hit && (hitWallIndex == -1 || collisionTime < earliestTime))
            {
                earliestTime = collisionTime;
                hitWallIndex = static_cast<int>(i);
            }
        }

        if (hitWallIndex == -1)
        {
            ball.update(remainingTime);
            return;
        }
        ball.update(earliestTime);

        sf::Vector2f velocity = ball.getVelocity();

        sf::Vector2f position = ball.getPosition();
        const Wall& wall = walls[hitWallIndex];

        float left = wall.getPosition().x;
        float right = left + wall.getSize().x;

        float top = wall.getPosition().y;
        float bottom = top + wall.getSize().y;

        float distanceLeft = std::abs(position.x + ball.getRadius() - left);
        float distanceRight = std::abs(position.x - ball.getRadius() - right);

        float distanceTop = std::abs(position.y + ball.getRadius() - top);
        float distanceBottom = std::abs(position.y - ball.getRadius() - bottom);

        float minimumDistance = std::min(
            std::min(distanceLeft, distanceRight),
            std::min(distanceTop, distanceBottom)
        );

        if (minimumDistance == distanceLeft || minimumDistance == distanceRight)
        {
            velocity.x = -velocity.x * restitution;
        }
        else
        {
            velocity.y = -velocity.y * restitution;
        }

        ball.setVelocity(velocity);

        ball.setVelocity(velocity);
        remainingTime -= earliestTime;
    }

    if (remainingTime > 0.f)
        ball.update(remainingTime);
}

bool CCD::checkBallBallCCD(
    const Ball& a,
    const Ball& b,
    float dt,
    float& collisionTime
)
{
    sf::Vector2f p = b.getPosition() - a.getPosition();
    sf::Vector2f v = b.getVelocity() - a.getVelocity();

    float radiusSum = a.getRadius() + b.getRadius();

    float A = v.x * v.x + v.y * v.y;
    float B = 2.f * (p.x * v.x + p.y * v.y);
    float C = p.x * p.x + p.y * p.y - radiusSum * radiusSum;

    // Шарики уже пересекаются
    if (C <= 0.f)
    {
        float approach = p.x * v.x + p.y * v.y;

        // Если шарики расходятся или стоят,
        // повторное столкновение не требуется
        if (approach >= 0.f)
            return false;

        collisionTime = 0.f;
        return true;
    }

    // Нет относительного движения
    if (A < 0.000001f)
        return false;

    float discriminant = B * B - 4.f * A * C;

    // Действительных корней нет
    if (discriminant < 0.f)
        return false;

    float time = (-B - std::sqrt(discriminant)) / (2.f * A);

    if (time < 0.f || time > dt)
        return false;

    // Проверяем, что шарики сближаются
    float approach = p.x * v.x + p.y * v.y;

    if (approach >= 0.f)
        return false;

    collisionTime = time;
    return true;
}

void CCD::updateBallPairWithCCD(
    Ball& a,
    Ball& b,
    float dt,
    float restitution
)
{
    float collisionTime = 0.f;

    bool hit = checkBallBallCCD(
        a, b, dt, collisionTime
    );

    if (!hit)
    {
        a.update(dt);
        b.update(dt);
        return;
    }

    // Двигаем оба шарика до контакта
    a.update(collisionTime);
    b.update(collisionTime);

    // Рассчитываем нормаль столкновения
    sf::Vector2f difference = b.getPosition() - a.getPosition();

    float distance = std::sqrt(
        difference.x * difference.x +
        difference.y * difference.y
    );

    if (distance < 0.0001f)
        return;

    sf::Vector2f normal = difference / distance;

    sf::Vector2f relativeVelocity =
        b.getVelocity() - a.getVelocity();

    float velocityAlongNormal =
        relativeVelocity.x * normal.x +
        relativeVelocity.y * normal.y;

    if (velocityAlongNormal < 0.f)
    {
        float inverseMassA = 1.f / a.getMass();
        float inverseMassB = 1.f / b.getMass();

        float impulseMagnitude =
            -(1.f + restitution) * velocityAlongNormal /
            (inverseMassA + inverseMassB);

        sf::Vector2f impulse = normal * impulseMagnitude;

        a.setVelocity(
            a.getVelocity() - impulse * inverseMassA
        );

        b.setVelocity(
            b.getVelocity() + impulse * inverseMassB
        );
    }

    // Двигаем шарики в течение оставшегося времени
    float remainingTime = dt - collisionTime;

    a.update(remainingTime);
    b.update(remainingTime);
}


void CCD::updateBallsWithCCD(
    std::vector<Ball>& balls,
    const std::vector<Wall>& walls,
    float dt,
    float restitution
)
{
    float remainingTime = dt;
    const int maxCollisions = 16;

    solveInitialOverlaps(balls, walls);
    
    for (int iteration = 0; iteration < maxCollisions; iteration++)
    {
        if (remainingTime <= 0.000001f)
            return;

        float earliestTime = remainingTime;
        int hitBallA = -1;
        int hitBallB = -1;
        int hitWall = -1;

        // Ищем столкновения шариков со стенами
        for (size_t i = 0; i < balls.size(); i++)
        {
            for (size_t w = 0; w < walls.size(); w++)
            {
                float time = 0.f;

                if (checkBallWallCCD(
                    balls[i], walls[w], remainingTime, time))
                {
                    if (time < earliestTime ||
                        (hitBallA == -1 && hitBallB == -1 && hitWall == -1))
                    {
                        earliestTime = time;
                        hitBallA = static_cast<int>(i);
                        hitBallB = -1;
                        hitWall = static_cast<int>(w);
                    }
                }
            }
        }

        // Ищем столкновения между всеми парами шариков
        for (size_t i = 0; i < balls.size(); i++)
        {
            for (size_t j = i + 1; j < balls.size(); j++)
            {
                float time = 0.f;

                if (checkBallBallCCD(
                    balls[i], balls[j], remainingTime, time))
                {
                    if (time < earliestTime ||
                        (hitBallA == -1 && hitBallB == -1 && hitWall == -1))
                    {
                        earliestTime = time;
                        hitBallA = static_cast<int>(i);
                        hitBallB = static_cast<int>(j);
                        hitWall = -1;
                    }
                }
            }
        }

        // Если столкновений нет — перемещаем все шарики
        if (hitBallA == -1)
        {
            for (Ball& ball : balls)
                ball.update(remainingTime);

            return;
        }

        // Перемещаем все шарики до найденного столкновения
        for (Ball& ball : balls)
            ball.update(earliestTime);

        remainingTime -= earliestTime;

        // Столкновение со стеной
        if (hitWall != -1)
        {
            Ball& ball = balls[hitBallA];
            const Wall& wall = walls[hitWall];

            sf::Vector2f p = ball.getPosition();
            sf::Vector2f v = ball.getVelocity();

            float left = wall.getPosition().x;
            float right = left + wall.getSize().x;
            float top = wall.getPosition().y;
            float bottom = top + wall.getSize().y;
            float r = ball.getRadius();

            float dl = std::abs(p.x + r - left);
            float dr = std::abs(p.x - r - right);
            float dtp = std::abs(p.y + r - top);
            float db = std::abs(p.y - r - bottom);

            float minDistance = std::min(
                std::min(dl, dr),
                std::min(dtp, db)
            );

            if (minDistance == dl || minDistance == dr)
                v.x = -v.x * restitution;
            else
                v.y = -v.y * restitution;

            ball.setVelocity(v);
            float speedLength = std::sqrt(v.x * v.x + v.y * v.y);

            if (speedLength > 0.000001f)
            {
                ball.setPosition(
                    ball.getPosition() +
                    v / speedLength * 0.001f
                );
            }
        }
        // Столкновение двух шариков
        else
        {
            Ball& a = balls[hitBallA];
            Ball& b = balls[hitBallB];

            sf::Vector2f difference =
                b.getPosition() - a.getPosition();

            float distance = std::sqrt(
                difference.x * difference.x +
                difference.y * difference.y
            );
            std::cout << "Distance: " << distance
                << " | Relative speed: "
                << (b.getVelocity().x - a.getVelocity().x)
                << std::endl;
            if (distance > 0.000001f)
            {
                sf::Vector2f normal = difference / distance;

                sf::Vector2f relativeVelocity =
                    b.getVelocity() - a.getVelocity();

                float speed =
                    relativeVelocity.x * normal.x +
                    relativeVelocity.y * normal.y;

                // Импульс применяем только при сближении
                if (speed < -0.000001f)
                {
                    float invA = 1.f / a.getMass();
                    float invB = 1.f / b.getMass();

                    float impulseValue =
                        -(1.f + restitution) * speed /
                        (invA + invB);

                    sf::Vector2f impulse =
                        normal * impulseValue;

                    a.setVelocity(
                        a.getVelocity() - impulse * invA
                    );

                    b.setVelocity(
                        b.getVelocity() + impulse * invB
                    );
                }

                // Небольшая коррекция позиции против повторного контакта
                const float separation = 0.001f;

                a.setPosition(
                    a.getPosition() - normal * separation * 0.5f
                );

                b.setPosition(
                    b.getPosition() + normal * separation * 0.5f
                );
            }
        }
    }

    // Если достигнут лимит столкновений, завершаем оставшийся шаг
    for (Ball& ball : balls)
        ball.update(remainingTime);
}


void CCD::solveInitialOverlaps(
    std::vector<Ball>& balls,
    const std::vector<Wall>& walls
)
{
    // Исправляем пересечение шариков друг с другом
    for (size_t i = 0; i < balls.size(); i++)
    {
        for (size_t j = i + 1; j < balls.size(); j++)
        {
            Collision collision = checkCollision(
                balls[i],
                balls[j]
            );

            if (!collision.isColliding)
                continue;

            float inverseMassA = 1.f / balls[i].getMass();
            float inverseMassB = 1.f / balls[j].getMass();

            float totalInverseMass = inverseMassA + inverseMassB;

            if (totalInverseMass <= 0.f)
                continue;

            float correctionMagnitude =
                collision.penetration / totalInverseMass;

            sf::Vector2f correction =
                collision.normal * correctionMagnitude;

            balls[i].setPosition(
                balls[i].getPosition() -
                correction * inverseMassA
            );

            balls[j].setPosition(
                balls[j].getPosition() +
                correction * inverseMassB
            );
        }
    }

    // Исправляем пересечение шариков со стенами
    // Только координаты, скорости не меняем
    for (Ball& ball : balls)
    {
        for (const Wall& wall : walls)
        {
            sf::Vector2f pos = ball.getPosition();
            float radius = ball.getRadius();

            sf::Vector2f wallPos = wall.getPosition();
            sf::Vector2f wallSize = wall.getSize();

            float closestX = std::clamp(
                pos.x,
                wallPos.x,
                wallPos.x + wallSize.x
            );

            float closestY = std::clamp(
                pos.y,
                wallPos.y,
                wallPos.y + wallSize.y
            );

            sf::Vector2f difference(
                pos.x - closestX,
                pos.y - closestY
            );

            float distanceSquared =
                difference.x * difference.x +
                difference.y * difference.y;

            if (distanceSquared >= radius * radius)
                continue;

            float distance = std::sqrt(distanceSquared);

            if (distance > 0.0001f)
            {
                sf::Vector2f normal(
                    difference.x / distance,
                    difference.y / distance
                );

                ball.setPosition(
                    pos + normal * (radius - distance)
                );
            }
        }
    }
}