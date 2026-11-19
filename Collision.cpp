
#include "Collision.h"

#include "PhysicsGroup.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    constexpr float EPSILON = 0.0001f;

    constexpr float POSITION_SLOP = 0.01f;
    constexpr float POSITION_CORRECTION = 0.8f;

    constexpr float RESTITUTION_THRESHOLD = 20.f;

    // -------------------------------------------------
    // VECTOR HELPERS
    // -------------------------------------------------

    float dot(sf::Vector2f a, sf::Vector2f b)
    {
        return a.x * b.x + a.y * b.y;
    }

    float lengthSquared(sf::Vector2f value)
    {
        return dot(value, value);
    }

    float length(sf::Vector2f value)
    {
        return std::sqrt(lengthSquared(value));
    }

    sf::Vector2f normalize(sf::Vector2f value)
    {
        float valueLength = length(value);

        if (valueLength <= EPSILON)
            return { 0.f, 0.f };

        return value / valueLength;
    }

    bool isValidVector(sf::Vector2f value)
    {
        return std::isfinite(value.x) &&
            std::isfinite(value.y);
    }

    float safeRestitution(float restitution)
    {
        if (!std::isfinite(restitution))
            return 0.f;

        return std::clamp(restitution, 0.f, 1.f);
    }

    // -------------------------------------------------
    // MASS HELPERS
    // -------------------------------------------------

    float inverseMass(const Ball& ball)
    {
        if (ball.getGroup() == PhysicsGroup::Static)
            return 0.f;

        float mass = ball.getMass();

        if (!std::isfinite(mass) || mass <= EPSILON)
            return 0.f;

        return 1.f / mass;
    }

    float inverseMass(const Box& box)
    {
        if (box.getGroup() == PhysicsGroup::Static)
            return 0.f;

        float mass = box.getMass();

        if (!std::isfinite(mass) || mass <= EPSILON)
            return 0.f;

        return 1.f / mass;
    }

    // -------------------------------------------------
    // IMPULSE HELPERS
    // -------------------------------------------------

    void applyBallImpulse(
        Ball& a,
        Ball& b,
        sf::Vector2f normal,
        float restitution
    )
    {
        float invMassA = inverseMass(a);
        float invMassB = inverseMass(b);

        float totalInverseMass = invMassA + invMassB;

        if (totalInverseMass <= EPSILON)
            return;

        sf::Vector2f relativeVelocity =
            b.getVelocity() - a.getVelocity();

        float velocityAlongNormal =
            dot(relativeVelocity, normal);

        // Bodies are already separating.
        if (velocityAlongNormal >= 0.f)
            return;

        float e = safeRestitution(restitution);

        if (std::abs(velocityAlongNormal) < RESTITUTION_THRESHOLD)
            e = 0.f;

        float impulseMagnitude =
            -(1.f + e) * velocityAlongNormal /
            totalInverseMass;

        sf::Vector2f impulse =
            normal * impulseMagnitude;

        if (invMassA > 0.f)
        {
            a.setVelocity(
                a.getVelocity() - impulse * invMassA
            );
        }

        if (invMassB > 0.f)
        {
            b.setVelocity(
                b.getVelocity() + impulse * invMassB
            );
        }
    }

    void applyBoxImpulse(
        Box& a,
        Box& b,
        sf::Vector2f normal,
        float restitution
    )
    {
        float invMassA = inverseMass(a);
        float invMassB = inverseMass(b);

        float totalInverseMass = invMassA + invMassB;

        if (totalInverseMass <= EPSILON)
            return;

        sf::Vector2f relativeVelocity =
            b.getVelocity() - a.getVelocity();

        float velocityAlongNormal =
            dot(relativeVelocity, normal);

        if (velocityAlongNormal >= 0.f)
            return;

        float e = safeRestitution(restitution);

        if (std::abs(velocityAlongNormal) < RESTITUTION_THRESHOLD)
            e = 0.f;

        float impulseMagnitude =
            -(1.f + e) * velocityAlongNormal /
            totalInverseMass;

        sf::Vector2f impulse =
            normal * impulseMagnitude;

        if (invMassA > 0.f)
        {
            a.setVelocity(
                a.getVelocity() - impulse * invMassA
            );
        }

        if (invMassB > 0.f)
        {
            b.setVelocity(
                b.getVelocity() + impulse * invMassB
            );
        }
    }

    void applyBoxWallImpulse(
        Box& box,
        sf::Vector2f normal,
        float restitution
    )
    {
        if (inverseMass(box) <= EPSILON)
            return;

        float velocityAlongNormal =
            dot(box.getVelocity(), normal);

        if (velocityAlongNormal >= 0.f)
            return;

        float e = safeRestitution(restitution);

        if (std::abs(velocityAlongNormal) < RESTITUTION_THRESHOLD)
            e = 0.f;

        box.setVelocity(
            box.getVelocity() -
            normal * ((1.f + e) * velocityAlongNormal)
        );
    }

    // -------------------------------------------------
    // POSITION CORRECTION
    // -------------------------------------------------

    void correctBallBallPenetration(
        Ball& a,
        Ball& b,
        const Collision& collision
    )
    {
        float invMassA = inverseMass(a);
        float invMassB = inverseMass(b);

        float totalInverseMass = invMassA + invMassB;

        if (totalInverseMass <= EPSILON)
            return;

        float correctionMagnitude =
            std::max(
                collision.penetration - POSITION_SLOP,
                0.f
            );

        correctionMagnitude *= POSITION_CORRECTION;
        correctionMagnitude /= totalInverseMass;

        sf::Vector2f correction =
            collision.normal * correctionMagnitude;

        if (invMassA > 0.f)
        {
            a.setPosition(
                a.getPosition() - correction * invMassA
            );
        }

        if (invMassB > 0.f)
        {
            b.setPosition(
                b.getPosition() + correction * invMassB
            );
        }
    }
}

// =====================================================
// BALL - BALL
// =====================================================

Collision checkCollision(
    const Ball& a,
    const Ball& b
)
{
    Collision result;

    sf::Vector2f positionA = a.getPosition();
    sf::Vector2f positionB = b.getPosition();

    float radiusA = a.getRadius();
    float radiusB = b.getRadius();

    if (!isValidVector(positionA) ||
        !isValidVector(positionB) ||
        !std::isfinite(radiusA) ||
        !std::isfinite(radiusB) ||
        radiusA <= 0.f ||
        radiusB <= 0.f)
    {
        return result;
    }

    sf::Vector2f difference =
        positionB - positionA;

    float radiusSum = radiusA + radiusB;

    float distanceSquared =
        lengthSquared(difference);

    if (distanceSquared >= radiusSum * radiusSum)
        return result;

    float distance =
        std::sqrt(distanceSquared);

    result.isColliding = true;

    if (distance > EPSILON)
    {
        result.normal = difference / distance;
    }
    else
    {
        // Deterministic fallback for coincident centers.
        result.normal = { 1.f, 0.f };
    }

    result.penetration =
        radiusSum - distance;

    return result;
}

void resolveCollision(
    Ball& a,
    Ball& b,
    float restitution
)
{
    Collision collision =
        checkCollision(a, b);

    if (!collision.isColliding)
        return;

    applyBallImpulse(
        a,
        b,
        collision.normal,
        restitution
    );

    correctBallBallPenetration(
        a,
        b,
        collision
    );
}

BallSweepResult checkBallBallCCD(
    const Ball& a,
    const Ball& b,
    float dt
)
{
    BallSweepResult result;

    if (!std::isfinite(dt) || dt <= 0.f)
        return result;

    if (a.getGroup() == PhysicsGroup::Static &&
        b.getGroup() == PhysicsGroup::Static)
    {
        return result;
    }

    sf::Vector2f positionA = a.getPosition();
    sf::Vector2f positionB = b.getPosition();

    sf::Vector2f velocityA = a.getVelocity();
    sf::Vector2f velocityB = b.getVelocity();

    float radiusA = a.getRadius();
    float radiusB = b.getRadius();

    if (!isValidVector(positionA) ||
        !isValidVector(positionB) ||
        !isValidVector(velocityA) ||
        !isValidVector(velocityB) ||
        !std::isfinite(radiusA) ||
        !std::isfinite(radiusB) ||
        radiusA <= 0.f ||
        radiusB <= 0.f)
    {
        return result;
    }

    // Относительное положение и скорость.
    sf::Vector2f relativePosition =
        positionB - positionA;

    sf::Vector2f relativeVelocity =
        velocityB - velocityA;

    float radiusSum = radiusA + radiusB;

    // Уравнение:
    // |relativePosition + relativeVelocity * t|^2
    // = radiusSum^2

    float c =
        dot(relativePosition, relativePosition) -
        radiusSum * radiusSum;

    // Уже пересекаются: это обрабатывает
    // дискретный решатель.
    if (c <= 0.f)
        return result;

    float aCoefficient =
        dot(relativeVelocity, relativeVelocity);

    if (aCoefficient <= EPSILON * EPSILON)
        return result;

    float bCoefficient =
        2.f * dot(relativePosition, relativeVelocity);

    // Шары удаляются друг от друга.
    if (bCoefficient >= 0.f)
        return result;

    float discriminant =
        bCoefficient * bCoefficient -
        4.f * aCoefficient * c;

    if (discriminant < 0.f)
        return result;

    float sqrtDiscriminant =
        std::sqrt(discriminant);

    float collisionTime =
        (-bCoefficient - sqrtDiscriminant) /
        (2.f * aCoefficient);

    if (!std::isfinite(collisionTime) ||
        collisionTime < 0.f ||
        collisionTime > dt)
    {
        return result;
    }

    sf::Vector2f contactPositionA =
        positionA + velocityA * collisionTime;

    sf::Vector2f contactPositionB =
        positionB + velocityB * collisionTime;

    sf::Vector2f difference =
        contactPositionB - contactPositionA;

    float distance = length(difference);

    if (distance <= EPSILON)
        return result;

    result.hit = true;
    result.time = collisionTime;
    result.normal = difference / distance;

    return result;
}

void resolveBallBallImpact(
    Ball& a,
    Ball& b,
    sf::Vector2f normal,
    float restitution
)
{
    normal = normalize(normal);

    if (lengthSquared(normal) <= EPSILON)
        return;

    applyBallImpulse(
        a,
        b,
        normal,
        restitution
    );
}

// =====================================================
// BALL - WALL
// =====================================================

void resolveBallWallCollision(
    Ball& ball,
    const Wall& wall,
    float restitution
)
{
    if (inverseMass(ball) <= EPSILON)
        return;

    sf::Vector2f circlePosition =
        ball.getPosition();

    sf::Vector2f wallPosition =
        wall.getPosition();

    sf::Vector2f wallSize =
        wall.getSize();

    float radius = ball.getRadius();

    if (!isValidVector(circlePosition) ||
        !isValidVector(wallPosition) ||
        !isValidVector(wallSize) ||
        !std::isfinite(radius) ||
        radius <= 0.f ||
        wallSize.x <= 0.f ||
        wallSize.y <= 0.f)
    {
        return;
    }

    sf::Vector2f closestPoint(
        std::clamp(
            circlePosition.x,
            wallPosition.x,
            wallPosition.x + wallSize.x
        ),
        std::clamp(
            circlePosition.y,
            wallPosition.y,
            wallPosition.y + wallSize.y
        )
    );

    sf::Vector2f difference =
        circlePosition - closestPoint;

    float distanceSquared =
        lengthSquared(difference);

    if (distanceSquared >= radius * radius)
        return;

    float distance =
        std::sqrt(distanceSquared);

    sf::Vector2f normal;

    if (distance > EPSILON)
    {
        normal = difference / distance;
    }
    else
    {
        // Circle center is inside the wall.
        // Find the nearest wall side.

        float left =
            std::abs(circlePosition.x - wallPosition.x);

        float right =
            std::abs(
                wallPosition.x + wallSize.x -
                circlePosition.x
            );

        float top =
            std::abs(circlePosition.y - wallPosition.y);

        float bottom =
            std::abs(
                wallPosition.y + wallSize.y -
                circlePosition.y
            );

        float minimum =
            std::min(
                std::min(left, right),
                std::min(top, bottom)
            );

        if (minimum == left)
            normal = { -1.f, 0.f };
        else if (minimum == right)
            normal = { 1.f, 0.f };
        else if (minimum == top)
            normal = { 0.f, -1.f };
        else
            normal = { 0.f, 1.f };

        distance = -minimum;
    }

    float penetration =
        radius - distance;

    ball.setPosition(
        ball.getPosition() +
        normal * penetration
    );

    float velocityAlongNormal =
        dot(ball.getVelocity(), normal);

    if (velocityAlongNormal < 0.f)
    {
        float e = safeRestitution(restitution);

        if (std::abs(velocityAlongNormal) < RESTITUTION_THRESHOLD)
            e = 0.f;

        ball.setVelocity(
            ball.getVelocity() -
            normal * ((1.f + e) * velocityAlongNormal)
        );
    }
}

// =====================================================
// BOX - WALL: COLLISION DETECTION
// =====================================================

Collision checkBoxWallCollision(
    const Box& box,
    const Wall& wall
)
{
    Collision result;

    sf::Vector2f boxPosition =
        box.getPosition();

    sf::Vector2f boxSize =
        box.getSize();

    sf::Vector2f wallPosition =
        wall.getPosition();

    sf::Vector2f wallSize =
        wall.getSize();

    if (!isValidVector(boxPosition) ||
        !isValidVector(boxSize) ||
        !isValidVector(wallPosition) ||
        !isValidVector(wallSize) ||
        boxSize.x <= 0.f ||
        boxSize.y <= 0.f ||
        wallSize.x <= 0.f ||
        wallSize.y <= 0.f)
    {
        return result;
    }

    sf::Vector2f halfSize =
        boxSize / 2.f;

    float boxLeft =
        boxPosition.x - halfSize.x;

    float boxRight =
        boxPosition.x + halfSize.x;

    float boxTop =
        boxPosition.y - halfSize.y;

    float boxBottom =
        boxPosition.y + halfSize.y;

    float wallLeft =
        wallPosition.x;

    float wallRight =
        wallPosition.x + wallSize.x;

    float wallTop =
        wallPosition.y;

    float wallBottom =
        wallPosition.y + wallSize.y;

    float overlapX =
        std::min(boxRight, wallRight) -
        std::max(boxLeft, wallLeft);

    float overlapY =
        std::min(boxBottom, wallBottom) -
        std::max(boxTop, wallTop);

    if (overlapX <= 0.f || overlapY <= 0.f)
        return result;

    result.isColliding = true;

    if (overlapX < overlapY)
    {
        if (boxPosition.x < wallPosition.x + wallSize.x / 2.f)
            result.normal = { -1.f, 0.f };
        else if (boxPosition.x > wallPosition.x + wallSize.x / 2.f)
            result.normal = { 1.f, 0.f };
        else
            result.normal =
            box.getVelocity().x >= 0.f
            ? sf::Vector2f(-1.f, 0.f)
            : sf::Vector2f(1.f, 0.f);

        result.penetration = overlapX;
    }
    else
    {
        if (boxPosition.y < wallPosition.y + wallSize.y / 2.f)
            result.normal = { 0.f, -1.f };
        else if (boxPosition.y > wallPosition.y + wallSize.y / 2.f)
            result.normal = { 0.f, 1.f };
        else
            result.normal =
            box.getVelocity().y >= 0.f
            ? sf::Vector2f(0.f, -1.f)
            : sf::Vector2f(0.f, 1.f);

        result.penetration = overlapY;
    }

    return result;
}

// =====================================================
// BOX - WALL: POSITION CORRECTION
// =====================================================

void correctBoxWallPenetration(
    Box& box,
    const Collision& collision
)
{
    if (!collision.isColliding ||
        inverseMass(box) <= EPSILON)
    {
        return;
    }

    float correctionMagnitude =
        std::max(
            collision.penetration - POSITION_SLOP,
            0.f
        ) * POSITION_CORRECTION;

    box.setPosition(
        box.getPosition() +
        collision.normal * correctionMagnitude
    );
}

// =====================================================
// BOX - WALL: RESOLUTION
// =====================================================

void resolveBoxWallCollision(
    Box& box,
    const Wall& wall,
    float restitution
)
{
    Collision collision =
        checkBoxWallCollision(box, wall);

    if (!collision.isColliding)
        return;

    correctBoxWallPenetration(
        box,
        collision
    );

    applyBoxWallImpulse(
        box,
        collision.normal,
        restitution
    );
}

// =====================================================
// BOX - BOX: COLLISION DETECTION
// =====================================================

Collision checkBoxBoxCollision(
    const Box& a,
    const Box& b
)
{
    Collision result;

    sf::Vector2f positionA =
        a.getPosition();

    sf::Vector2f positionB =
        b.getPosition();

    sf::Vector2f sizeA =
        a.getSize();

    sf::Vector2f sizeB =
        b.getSize();

    if (!isValidVector(positionA) ||
        !isValidVector(positionB) ||
        !isValidVector(sizeA) ||
        !isValidVector(sizeB) ||
        sizeA.x <= 0.f ||
        sizeA.y <= 0.f ||
        sizeB.x <= 0.f ||
        sizeB.y <= 0.f)
    {
        return result;
    }

    sf::Vector2f difference =
        positionB - positionA;

    sf::Vector2f halfA =
        sizeA / 2.f;

    sf::Vector2f halfB =
        sizeB / 2.f;

    float overlapX =
        halfA.x + halfB.x -
        std::abs(difference.x);

    float overlapY =
        halfA.y + halfB.y -
        std::abs(difference.y);

    if (overlapX <= 0.f || overlapY <= 0.f)
        return result;

    result.isColliding = true;

    if (overlapX < overlapY)
    {
        result.normal =
            difference.x >= 0.f
            ? sf::Vector2f(1.f, 0.f)
            : sf::Vector2f(-1.f, 0.f);

        result.penetration = overlapX;
    }
    else
    {
        result.normal =
            difference.y >= 0.f
            ? sf::Vector2f(0.f, 1.f)
            : sf::Vector2f(0.f, -1.f);

        result.penetration = overlapY;
    }

    return result;
}

// =====================================================
// BOX - BOX: POSITION CORRECTION
// =====================================================

void correctBoxBoxPenetration(
    Box& a,
    Box& b,
    const Collision& collision
)
{
    if (!collision.isColliding)
        return;

    float invMassA =
        inverseMass(a);

    float invMassB =
        inverseMass(b);

    float totalInverseMass =
        invMassA + invMassB;

    if (totalInverseMass <= EPSILON)
        return;

    float correctionMagnitude =
        std::max(
            collision.penetration - POSITION_SLOP,
            0.f
        );

    correctionMagnitude *= POSITION_CORRECTION;
    correctionMagnitude /= totalInverseMass;

    sf::Vector2f correction =
        collision.normal * correctionMagnitude;

    if (invMassA > 0.f)
    {
        a.setPosition(
            a.getPosition() -
            correction * invMassA
        );
    }

    if (invMassB > 0.f)
    {
        b.setPosition(
            b.getPosition() +
            correction * invMassB
        );
    }
}

// =====================================================
// BOX - BOX: IMPACT
// =====================================================

void resolveBoxBoxImpact(
    Box& a,
    Box& b,
    sf::Vector2f normal,
    float restitution
)
{
    normal = normalize(normal);

    if (lengthSquared(normal) <= EPSILON)
        return;

    applyBoxImpulse(
        a,
        b,
        normal,
        restitution
    );
}

// =====================================================
// BOX - BOX: RESOLUTION
// =====================================================

void resolveBoxBoxCollision(
    Box& a,
    Box& b,
    float restitution
)
{
    Collision collision =
        checkBoxBoxCollision(a, b);

    if (!collision.isColliding)
        return;

    correctBoxBoxPenetration(
        a,
        b,
        collision
    );

    resolveBoxBoxImpact(
        a,
        b,
        collision.normal,
        restitution
    );
}

// =====================================================
// CCD: BOX - BOX
// =====================================================


BoxSweepResult checkBoxBoxCCD(
    const Box& a,
    const Box& b,
    float dt
)
{
    BoxSweepResult result;

    if (!std::isfinite(dt) || dt <= 0.f)
        return result;

    if (a.getGroup() == PhysicsGroup::Static &&
        b.getGroup() == PhysicsGroup::Static)
        return result;

    sf::Vector2f positionA = a.getPosition();
    sf::Vector2f positionB = b.getPosition();

    sf::Vector2f velocityA = a.getVelocity();
    sf::Vector2f velocityB = b.getVelocity();

    // Статические тела не перемещаются,
    // даже если в их velocity случайно записано значение.
    if (a.getGroup() == PhysicsGroup::Static)
        velocityA = { 0.f, 0.f };

    if (b.getGroup() == PhysicsGroup::Static)
        velocityB = { 0.f, 0.f };

    sf::Vector2f sizeA = a.getSize();
    sf::Vector2f sizeB = b.getSize();

    if (!isValidVector(positionA) ||
        !isValidVector(positionB) ||
        !isValidVector(velocityA) ||
        !isValidVector(velocityB) ||
        !isValidVector(sizeA) ||
        !isValidVector(sizeB) ||
        sizeA.x <= 0.f || sizeA.y <= 0.f ||
        sizeB.x <= 0.f || sizeB.y <= 0.f)
        return result;

    if (checkBoxBoxCollision(a, b).isColliding)
        return result;

    sf::Vector2f relativePosition =
        positionA - positionB;

    sf::Vector2f relativeVelocity =
        velocityA - velocityB;

    sf::Vector2f halfSize =
        (sizeA + sizeB) / 2.f;

    float entryX;
    float exitX;
    float entryY;
    float exitY;

    // Проверка пересечения по X.
    if (std::abs(relativeVelocity.x) < EPSILON)
    {
        if (std::abs(relativePosition.x) > halfSize.x + EPSILON)
            return result;

        entryX = -std::numeric_limits<float>::infinity();
        exitX = std::numeric_limits<float>::infinity();
    }
    else
    {
        float x1 =
            (-halfSize.x - relativePosition.x) /
            relativeVelocity.x;

        float x2 =
            (halfSize.x - relativePosition.x) /
            relativeVelocity.x;

        entryX = std::min(x1, x2);
        exitX = std::max(x1, x2);
    }

    // Проверка пересечения по Y.
    if (std::abs(relativeVelocity.y) < EPSILON)
    {
        if (std::abs(relativePosition.y) > halfSize.y + EPSILON)
            return result;

        entryY = -std::numeric_limits<float>::infinity();
        exitY = std::numeric_limits<float>::infinity();
    }
    else
    {
        float y1 =
            (-halfSize.y - relativePosition.y) /
            relativeVelocity.y;

        float y2 =
            (halfSize.y - relativePosition.y) /
            relativeVelocity.y;

        entryY = std::min(y1, y2);
        exitY = std::max(y1, y2);
    }

    float entryTime = std::max(entryX, entryY);
    float exitTime = std::min(exitX, exitY);

    if (entryTime > exitTime ||
        exitTime < -EPSILON ||
        entryTime < -EPSILON ||
        entryTime > dt + EPSILON)
        return result;

    // Определяем ось первого контакта.
    if (entryX > entryY)
    {
        result.normal =
            relativeVelocity.x > 0.f
            ? sf::Vector2f(1.f, 0.f)
            : sf::Vector2f(-1.f, 0.f);
    }
    else
    {
        result.normal =
            relativeVelocity.y > 0.f
            ? sf::Vector2f(0.f, 1.f)
            : sf::Vector2f(0.f, -1.f);
    }

    sf::Vector2f relativeContactVelocity =
        velocityB - velocityA;

    // Не обрабатываем тела, которые уже расходятся.
    if (dot(relativeContactVelocity, result.normal) >= 0.f)
        return BoxSweepResult();

    result.hit = true;
    result.time = std::clamp(entryTime, 0.f, dt);

    return result;
}

// =====================================================
// CCD: BOX - WALL
// =====================================================


BoxWallSweepResult checkBoxWallCCD(
    const Box& box,
    const Wall& wall,
    float dt
)
{
    BoxWallSweepResult result;

    if (!std::isfinite(dt) || dt <= 0.f)
        return result;

    if (box.getGroup() == PhysicsGroup::Static)
        return result;

    sf::Vector2f position = box.getPosition();
    sf::Vector2f velocity = box.getVelocity();
    sf::Vector2f size = box.getSize();

    sf::Vector2f wallPosition = wall.getPosition();
    sf::Vector2f wallSize = wall.getSize();

    if (!isValidVector(position) ||
        !isValidVector(velocity) ||
        !isValidVector(size) ||
        !isValidVector(wallPosition) ||
        !isValidVector(wallSize) ||
        size.x <= 0.f || size.y <= 0.f ||
        wallSize.x <= 0.f || wallSize.y <= 0.f)
        return result;

    if (checkBoxWallCollision(box, wall).isColliding)
        return result;

    sf::Vector2f halfSize = size / 2.f;

    // Расширяем стену на половину размера Box.
    float left = wallPosition.x - halfSize.x;
    float right = wallPosition.x + wallSize.x + halfSize.x;
    float top = wallPosition.y - halfSize.y;
    float bottom = wallPosition.y + wallSize.y + halfSize.y;

    float entryX;
    float exitX;
    float entryY;
    float exitY;

    if (std::abs(velocity.x) < EPSILON)
    {
        if (position.x < left - EPSILON ||
            position.x > right + EPSILON)
            return result;

        entryX = -std::numeric_limits<float>::infinity();
        exitX = std::numeric_limits<float>::infinity();
    }
    else
    {
        float x1 = (left - position.x) / velocity.x;
        float x2 = (right - position.x) / velocity.x;

        entryX = std::min(x1, x2);
        exitX = std::max(x1, x2);
    }

    if (std::abs(velocity.y) < EPSILON)
    {
        if (position.y < top - EPSILON ||
            position.y > bottom + EPSILON)
            return result;

        entryY = -std::numeric_limits<float>::infinity();
        exitY = std::numeric_limits<float>::infinity();
    }
    else
    {
        float y1 = (top - position.y) / velocity.y;
        float y2 = (bottom - position.y) / velocity.y;

        entryY = std::min(y1, y2);
        exitY = std::max(y1, y2);
    }

    float entryTime = std::max(entryX, entryY);
    float exitTime = std::min(exitX, exitY);

    if (entryTime > exitTime ||
        exitTime < -EPSILON ||
        entryTime < -EPSILON ||
        entryTime > dt + EPSILON)
        return result;

    if (entryX > entryY)
    {
        result.normal =
            velocity.x > 0.f
            ? sf::Vector2f(-1.f, 0.f)
            : sf::Vector2f(1.f, 0.f);
    }
    else
    {
        result.normal =
            velocity.y > 0.f
            ? sf::Vector2f(0.f, -1.f)
            : sf::Vector2f(0.f, 1.f);
    }

    if (dot(velocity, result.normal) >= 0.f)
        return BoxWallSweepResult();

    result.hit = true;
    result.time = std::clamp(entryTime, 0.f, dt);

    return result;
}


// =====================================================
// CCD: BOX - WALL IMPACT
// =====================================================

void resolveBoxWallImpact(
    Box& box,
    sf::Vector2f normal,
    float restitution
)
{
    normal = normalize(normal);

    if (lengthSquared(normal) <= EPSILON)
        return;

    applyBoxWallImpulse(
        box,
        normal,
        restitution
    );
}

// =====================================================
// ITERATIVE BOX SOLVER
// =====================================================


void solveBoxCollisions(
    std::vector<Box>& boxes,
    const std::vector<Wall>& walls,
    float restitution,
    int iterations
)
{
    if (iterations <= 0)
        return;

    restitution = safeRestitution(restitution);

    constexpr float FRICTION = 0.5f;
    constexpr float SOLVER_SLOP = 0.01f;
    constexpr float POSITION_FACTOR = 0.2f;

    // Контакт Box-Wall.
    struct WallContact
    {
        std::size_t boxIndex;
        std::size_t wallIndex;

        sf::Vector2f normal;

        float accumulatedNormal = 0.f;
        float accumulatedTangent = 0.f;
        float restitutionBias = 0.f;
    };

    // Контакт Box-Box.
    struct BoxContact
    {
        std::size_t indexA;
        std::size_t indexB;

        sf::Vector2f normal;

        float accumulatedNormal = 0.f;
        float accumulatedTangent = 0.f;
        float restitutionBias = 0.f;
    };

    std::vector<WallContact> wallContacts;
    std::vector<BoxContact> boxContacts;

    // -----------------------------------------
    // СОЗДАНИЕ СПИСКА КОНТАКТОВ
    // -----------------------------------------

    for (std::size_t i = 0; i < boxes.size(); ++i)
    {
        Box& box = boxes[i];

        if (inverseMass(box) <= EPSILON)
            continue;

        for (std::size_t j = 0; j < walls.size(); ++j)
        {
            Collision collision =
                checkBoxWallCollision(box, walls[j]);

            if (!collision.isColliding)
                continue;

            float velocityAlongNormal =
                dot(box.getVelocity(), collision.normal);

            float e = restitution;

            if (std::abs(velocityAlongNormal) <
                RESTITUTION_THRESHOLD)
            {
                e = 0.f;
            }

            float restitutionBias = 0.f;

            if (velocityAlongNormal < 0.f)
            {
                restitutionBias =
                    -e * velocityAlongNormal;
            }

            wallContacts.push_back({
                i,
                j,
                collision.normal,
                0.f,
                0.f,
                restitutionBias
                });
        }
    }

    for (std::size_t i = 0; i < boxes.size(); ++i)
    {
        for (std::size_t j = i + 1; j < boxes.size(); ++j)
        {
            Box& a = boxes[i];
            Box& b = boxes[j];

            float invA = inverseMass(a);
            float invB = inverseMass(b);

            if (invA + invB <= EPSILON)
                continue;

            Collision collision =
                checkBoxBoxCollision(a, b);

            if (!collision.isColliding)
                continue;

            sf::Vector2f relativeVelocity =
                b.getVelocity() - a.getVelocity();

            float velocityAlongNormal =
                dot(relativeVelocity, collision.normal);

            float e = restitution;

            if (std::abs(velocityAlongNormal) <
                RESTITUTION_THRESHOLD)
            {
                e = 0.f;
            }

            float restitutionBias = 0.f;

            if (velocityAlongNormal < 0.f)
            {
                restitutionBias =
                    -e * velocityAlongNormal;
            }

            boxContacts.push_back({
                i,
                j,
                collision.normal,
                0.f,
                0.f,
                restitutionBias
                });
        }
    }

    // -----------------------------------------
    // ПОСЛЕДОВАТЕЛЬНЫЕ ИМПУЛЬСЫ
    // -----------------------------------------

    for (int iteration = 0;
        iteration < iterations;
        ++iteration)
    {
        // Box-Wall.
        for (WallContact& contact : wallContacts)
        {
            Box& box = boxes[contact.boxIndex];

            float invMass = inverseMass(box);

            if (invMass <= EPSILON)
                continue;

            sf::Vector2f normal =
                contact.normal;

            sf::Vector2f tangent(
                -normal.y,
                normal.x
            );

            sf::Vector2f velocity =
                box.getVelocity();

            float normalVelocity =
                dot(velocity, normal);

            float lambda =
                (contact.restitutionBias -
                    normalVelocity) / invMass;

            float oldNormalImpulse =
                contact.accumulatedNormal;

            contact.accumulatedNormal =
                std::max(
                    0.f,
                    oldNormalImpulse + lambda
                );

            float appliedNormalImpulse =
                contact.accumulatedNormal -
                oldNormalImpulse;

            velocity +=
                normal * (appliedNormalImpulse * invMass);

            box.setVelocity(velocity);

            // Касательный импульс — трение.
            velocity = box.getVelocity();

            float tangentVelocity =
                dot(velocity, tangent);

            float tangentLambda =
                -tangentVelocity / invMass;

            float oldTangentImpulse =
                contact.accumulatedTangent;

            float maximumFriction =
                FRICTION * contact.accumulatedNormal;

            contact.accumulatedTangent =
                std::clamp(
                    oldTangentImpulse + tangentLambda,
                    -maximumFriction,
                    maximumFriction
                );

            float appliedTangentImpulse =
                contact.accumulatedTangent -
                oldTangentImpulse;

            velocity +=
                tangent * (appliedTangentImpulse * invMass);

            box.setVelocity(velocity);
        }

        // Box-Box.
        for (BoxContact& contact : boxContacts)
        {
            Box& a = boxes[contact.indexA];
            Box& b = boxes[contact.indexB];

            float invA = inverseMass(a);
            float invB = inverseMass(b);

            float totalInverseMass = invA + invB;

            if (totalInverseMass <= EPSILON)
                continue;

            sf::Vector2f normal =
                contact.normal;

            sf::Vector2f tangent(
                -normal.y,
                normal.x
            );

            sf::Vector2f relativeVelocity =
                b.getVelocity() - a.getVelocity();

            float normalVelocity =
                dot(relativeVelocity, normal);

            float lambda =
                (contact.restitutionBias -
                    normalVelocity) / totalInverseMass;

            float oldNormalImpulse =
                contact.accumulatedNormal;

            contact.accumulatedNormal =
                std::max(
                    0.f,
                    oldNormalImpulse + lambda
                );

            float appliedNormalImpulse =
                contact.accumulatedNormal -
                oldNormalImpulse;

            sf::Vector2f normalImpulse =
                normal * appliedNormalImpulse;

            if (invA > 0.f)
            {
                a.setVelocity(
                    a.getVelocity() -
                    normalImpulse * invA
                );
            }

            if (invB > 0.f)
            {
                b.setVelocity(
                    b.getVelocity() +
                    normalImpulse * invB
                );
            }

            // Касательный импульс — трение.
            relativeVelocity =
                b.getVelocity() - a.getVelocity();

            float tangentVelocity =
                dot(relativeVelocity, tangent);

            float tangentLambda =
                -tangentVelocity / totalInverseMass;

            float oldTangentImpulse =
                contact.accumulatedTangent;

            float maximumFriction =
                FRICTION * contact.accumulatedNormal;

            contact.accumulatedTangent =
                std::clamp(
                    oldTangentImpulse + tangentLambda,
                    -maximumFriction,
                    maximumFriction
                );

            float appliedTangentImpulse =
                contact.accumulatedTangent -
                oldTangentImpulse;

            sf::Vector2f tangentImpulse =
                tangent * appliedTangentImpulse;

            if (invA > 0.f)
            {
                a.setVelocity(
                    a.getVelocity() -
                    tangentImpulse * invA
                );
            }

            if (invB > 0.f)
            {
                b.setVelocity(
                    b.getVelocity() +
                    tangentImpulse * invB
                );
            }
        }
    }

    // -----------------------------------------
    // ОТДЕЛЬНАЯ КОРРЕКЦИЯ ПОЗИЦИЙ
    // -----------------------------------------

    // Несколько мягких проходов уменьшают
    // проникновение, не добавляя скорость телам.

    for (int iteration = 0;
        iteration < iterations;
        ++iteration)
    {
        // Box-Wall.
        for (Box& box : boxes)
        {
            float invMass = inverseMass(box);

            if (invMass <= EPSILON)
                continue;

            for (const Wall& wall : walls)
            {
                Collision collision =
                    checkBoxWallCollision(box, wall);

                if (!collision.isColliding)
                    continue;

                float correction =
                    std::max(
                        collision.penetration - SOLVER_SLOP,
                        0.f
                    ) * POSITION_FACTOR;

                box.setPosition(
                    box.getPosition() +
                    collision.normal * correction
                );
            }
        }

        // Box-Box.
        for (std::size_t i = 0; i < boxes.size(); ++i)
        {
            for (std::size_t j = i + 1;
                j < boxes.size();
                ++j)
            {
                Box& a = boxes[i];
                Box& b = boxes[j];

                float invA = inverseMass(a);
                float invB = inverseMass(b);

                float totalInverseMass = invA + invB;

                if (totalInverseMass <= EPSILON)
                    continue;

                Collision collision =
                    checkBoxBoxCollision(a, b);

                if (!collision.isColliding)
                    continue;

                float correction =
                    std::max(
                        collision.penetration - SOLVER_SLOP,
                        0.f
                    ) * POSITION_FACTOR;

                correction /= totalInverseMass;

                sf::Vector2f correctionVector =
                    collision.normal * correction;

                if (invA > 0.f)
                {
                    a.setPosition(
                        a.getPosition() -
                        correctionVector * invA
                    );
                }

                if (invB > 0.f)
                {
                    b.setPosition(
                        b.getPosition() +
                        correctionVector * invB
                    );
                }
            }
        }
    }
}
