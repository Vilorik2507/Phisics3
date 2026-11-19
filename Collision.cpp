#include "Collision.h"


#include <cmath>
#include <algorithm>
#include <limits>

Collision checkCollision(const Ball& a, const Ball& b) {
	Collision result{ false, sf::Vector2f(0.f,0.f), 0.f };
	sf::Vector2f difference = b.getPosition() - a.getPosition();
	float distanceSquared = difference.x * difference.x + difference.y * difference.y;
	float radiusSum = a.getRadius() + b.getRadius();
	if (distanceSquared >= radiusSum * radiusSum) {//^2
		return result;
	}
	float distance = std::sqrt(distanceSquared);
	result.isColliding = true;
	if (distance > 0.0010f) {
		result.normal = difference / distance;
	}
	else {
		result.normal = sf::Vector2f(1.f, 0.f);
	}
	result.penetration = radiusSum - distance;

	return result;
}
void resolveCollision(Ball& a,Ball& b,float restitution)
{
    Collision collision = checkCollision(a, b);

    if (!collision.isColliding)
    {
        return;
    }

    sf::Vector2f normal = collision.normal;

    sf::Vector2f relativeVelocity =
        b.getVelocity() - a.getVelocity();

    float velocityAlongNormal =
        relativeVelocity.x * normal.x +
        relativeVelocity.y * normal.y;

    if (velocityAlongNormal > 0.f)
    {
        return;
    }

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

    float correctionPercent = 0.8f;
    float correctionSlop = 0.01f;

    float correctionMagnitude =
        std::max(
            collision.penetration - correctionSlop,
            0.f
        ) /
        (inverseMassA + inverseMassB) *
        correctionPercent;

    sf::Vector2f correction =
        normal * correctionMagnitude;

    a.setPosition(
        a.getPosition() - correction * inverseMassA
    );

    b.setPosition(
        b.getPosition() + correction * inverseMassB
    );
}

void resolveBallWallCollision(
    Ball& ball,
    const Wall& wall,
    float restitution
)
{
    sf::Vector2f center = ball.getPosition();
    sf::Vector2f wallPosition = wall.getPosition();
    sf::Vector2f wallSize = wall.getSize();

    float radius = ball.getRadius();

    // Ближайшая к центру шарика точка стены
    float closestX = std::clamp(
        center.x,
        wallPosition.x,
        wallPosition.x + wallSize.x
    );

    float closestY = std::clamp(
        center.y,
        wallPosition.y,
        wallPosition.y + wallSize.y
    );

    sf::Vector2f difference(
        center.x - closestX,
        center.y - closestY
    );

    float distanceSquared =
        difference.x * difference.x +
        difference.y * difference.y;

    if (distanceSquared >= radius * radius)
    {
        return;
    }

    float distance = std::sqrt(distanceSquared);

    sf::Vector2f normal;
    float penetration;

    if (distance > 0.0001f)
    {
        normal = difference / distance;
        penetration = radius - distance;
    }
    else
    {
        // Центр шарика находится внутри прямоугольника.
        // Выбираем ближайшую сторону стены.

        float left = center.x - wallPosition.x;
        float right = wallPosition.x + wallSize.x - center.x;
        float top = center.y - wallPosition.y;
        float bottom = wallPosition.y + wallSize.y - center.y;

        float minimum = std::min(
            std::min(left, right),
            std::min(top, bottom)
        );

        if (minimum == left)
            normal = sf::Vector2f(-1.f, 0.f);
        else if (minimum == right)
            normal = sf::Vector2f(1.f, 0.f);
        else if (minimum == top)
            normal = sf::Vector2f(0.f, -1.f);
        else
            normal = sf::Vector2f(0.f, 1.f);

        penetration = radius + minimum;
    }

    // Исправляем положение шарика
    ball.setPosition(
        ball.getPosition() + normal * penetration
    );

    // Рассчитываем изменение скорости
    sf::Vector2f velocity = ball.getVelocity();

    float velocityAlongNormal =
        velocity.x * normal.x +
        velocity.y * normal.y;

    // Если шарик уже движется от стены,
    // отражать его скорость не нужно.
    if (velocityAlongNormal < 0.f)
    {
        velocity = velocity -
            normal * ((1.f + restitution) * velocityAlongNormal);

        ball.setVelocity(velocity);
    }
}

void resolveBoxWallCollision(
    Box& box,
    const Wall& wall,
    float restitution
)
{
    sf::Vector2f boxPos = box.getPosition();
    sf::Vector2f boxSize = box.getSize();

    sf::Vector2f wallPos = wall.getPosition();
    sf::Vector2f wallSize = wall.getSize();

    float boxLeft = boxPos.x - boxSize.x / 2.f;
    float boxRight = boxPos.x + boxSize.x / 2.f;
    float boxTop = boxPos.y - boxSize.y / 2.f;
    float boxBottom = boxPos.y + boxSize.y / 2.f;

    float wallLeft = wallPos.x;
    float wallRight = wallPos.x + wallSize.x;
    float wallTop = wallPos.y;
    float wallBottom = wallPos.y + wallSize.y;

    if (boxRight <= wallLeft ||
        boxLeft >= wallRight ||
        boxBottom <= wallTop ||
        boxTop >= wallBottom)
    {
        return;
    }

    float overlapX =
        std::min(boxRight, wallRight) -
        std::max(boxLeft, wallLeft);

    float overlapY =
        std::min(boxBottom, wallBottom) -
        std::max(boxTop, wallTop);

    sf::Vector2f velocity = box.getVelocity();

    if (overlapX < overlapY)
    {
        if (boxPos.x < wallPos.x + wallSize.x / 2.f)
        {
            boxPos.x -= overlapX;
            if (velocity.x > 0.f)
                velocity.x = -velocity.x * restitution;
        }
        else
        {
            boxPos.x += overlapX;
            if (velocity.x < 0.f)
                velocity.x = -velocity.x * restitution;
        }
    }
    else
    {
        if (boxPos.y < wallPos.y + wallSize.y / 2.f)
        {
            boxPos.y -= overlapY;
            if (velocity.y > 0.f)
                velocity.y = -velocity.y * restitution;
        }
        else
        {
            boxPos.y += overlapY;
            if (velocity.y < 0.f)
                velocity.y = -velocity.y * restitution;
        }
    }

    box.setPosition(boxPos);
    box.setVelocity(velocity);
}


void resolveBoxBoxCollision(Box& a, Box& b, float restitution)
{
    sf::Vector2f posA = a.getPosition();
    sf::Vector2f posB = b.getPosition();

    sf::Vector2f sizeA = a.getSize();
    sf::Vector2f sizeB = b.getSize();

    sf::Vector2f velA = a.getVelocity();
    sf::Vector2f velB = b.getVelocity();

    float overlapX =
        (sizeA.x + sizeB.x) * 0.5f -
        std::abs(posA.x - posB.x);

    float overlapY =
        (sizeA.y + sizeB.y) * 0.5f -
        std::abs(posA.y - posB.y);

    if (overlapX <= 0.f || overlapY <= 0.f)
        return;

    // Нормаль направлена от A к B
    sf::Vector2f normal;

    if (overlapX < overlapY)
    {
        normal = {
            posA.x < posB.x ? 1.f : -1.f,
            0.f
        };
    }
    else
    {
        normal = {
            0.f,
            posA.y < posB.y ? 1.f : -1.f
        };
    }

    float invMassA = 1.f / a.getMass();
    float invMassB = 1.f / b.getMass();

    float totalInvMass = invMassA + invMassB;

    if (totalInvMass <= 0.f)
        return;

    // Коррекция проникновения
    float penetration =
        overlapX < overlapY ? overlapX : overlapY;

    const float slop = 0.01f;
    const float percent = 0.8f;

    float correction =
        std::max(penetration - slop, 0.f) *
        percent / totalInvMass;

    posA -= normal * correction * invMassA;
    posB += normal * correction * invMassB;

    // Расчёт импульса
    sf::Vector2f relativeVelocity = velB - velA;

    float velocityAlongNormal =
        relativeVelocity.x * normal.x +
        relativeVelocity.y * normal.y;

    if (velocityAlongNormal < 0.f)
    {
        float e = restitution;

        // Убираем отскок при медленном контакте
        if (std::abs(velocityAlongNormal) < 20.f)
            e = 0.f;

        float impulseMagnitude =
            -(1.f + e) * velocityAlongNormal /
            totalInvMass;

        sf::Vector2f impulse =
            normal * impulseMagnitude;

        velA -= impulse * invMassA;
        velB += impulse * invMassB;
    }

    a.setPosition(posA);
    b.setPosition(posB);

    a.setVelocity(velA);
    b.setVelocity(velB);
}

void updateBoxWithWallCCD(
    Box& box,
    const std::vector<Wall>& walls,
    float dt,
    float restitution
)
{
    float remainingTime = dt;
    const int maxCollisions = 8;

    // Сначала исправляем уже существующие пересечения
    for (const Wall& wall : walls)
    {
        resolveBoxWallCollision(box, wall, restitution);
    }

    for (int iteration = 0;
        iteration < maxCollisions && remainingTime > 0.000001f;
        iteration++)
    {
        sf::Vector2f pos = box.getPosition();
        sf::Vector2f size = box.getSize();
        sf::Vector2f velocity = box.getVelocity();

        float halfX = size.x / 2.f;
        float halfY = size.y / 2.f;

        float earliestTime = remainingTime;
        sf::Vector2f hitNormal(0.f, 0.f);
        bool foundCollision = false;

        for (const Wall& wall : walls)
        {
            sf::Vector2f wallPos = wall.getPosition();
            sf::Vector2f wallSize = wall.getSize();

            // Расширяем стену на половину размера коробки.
            // После этого проверяем движение центра коробки.
            float minX = wallPos.x - halfX;
            float maxX = wallPos.x + wallSize.x + halfX;

            float minY = wallPos.y - halfY;
            float maxY = wallPos.y + wallSize.y + halfY;

            float txEntry, txExit;
            float tyEntry, tyExit;

            const float infinity =
                std::numeric_limits<float>::infinity();

            if (std::abs(velocity.x) < 0.000001f)
            {
                if (pos.x < minX || pos.x > maxX)
                    continue;

                txEntry = -infinity;
                txExit = infinity;
            }
            else
            {
                float t1 = (minX - pos.x) / velocity.x;
                float t2 = (maxX - pos.x) / velocity.x;

                txEntry = std::min(t1, t2);
                txExit = std::max(t1, t2);
            }

            if (std::abs(velocity.y) < 0.000001f)
            {
                if (pos.y < minY || pos.y > maxY)
                    continue;

                tyEntry = -infinity;
                tyExit = infinity;
            }
            else
            {
                float t1 = (minY - pos.y) / velocity.y;
                float t2 = (maxY - pos.y) / velocity.y;

                tyEntry = std::min(t1, t2);
                tyExit = std::max(t1, t2);
            }

            float entryTime = std::max(txEntry, tyEntry);
            float exitTime = std::min(txExit, tyExit);

            if (entryTime > exitTime ||
                exitTime < 0.f ||
                entryTime < 0.f ||
                entryTime > remainingTime)
            {
                continue;
            }

            if (entryTime < earliestTime || !foundCollision)
            {
                earliestTime = entryTime;
                foundCollision = true;

                if (txEntry > tyEntry)
                {
                    hitNormal = sf::Vector2f(
                        velocity.x > 0.f ? -1.f : 1.f,
                        0.f
                    );
                }
                else
                {
                    hitNormal = sf::Vector2f(
                        0.f,
                        velocity.y > 0.f ? -1.f : 1.f
                    );
                }
            }
        }

        // Столкновений больше нет — перемещаем коробку до конца шага
        if (!foundCollision)
        {
            box.update(remainingTime);
            remainingTime = 0.f;
            break;
        }

        // Перемещаем коробку точно до момента столкновения
        box.update(earliestTime);
        remainingTime -= earliestTime;

        velocity = box.getVelocity();

        // Отражаем только компоненту скорости по нормали
        float velocityAlongNormal =
            velocity.x * hitNormal.x +
            velocity.y * hitNormal.y;

        if (velocityAlongNormal < -0.5f)
        {
            velocity.x -=
                (1.f + restitution) *
                velocityAlongNormal * hitNormal.x;

            velocity.y -=
                (1.f + restitution) *
                velocityAlongNormal * hitNormal.y;
        }
        else
        {
            // Убираем очень маленькую скорость вдоль нормали.
            velocity.x -= velocityAlongNormal * hitNormal.x;
            velocity.y -= velocityAlongNormal * hitNormal.y;
        }

        box.setVelocity(velocity);

        // Небольшое смещение от поверхности для защиты
        // от повторного обнаружения того же контакта
        sf::Vector2f newPos = box.getPosition();

        newPos.x += hitNormal.x * 0.001f;
        newPos.y += hitNormal.y * 0.001f;

        box.setPosition(newPos);
    }

    // Если достигли лимита столкновений, обрабатываем остаток времени
    if (remainingTime > 0.f)
    {
        box.update(remainingTime);
    }
}
