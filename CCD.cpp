
#include "CCD.h"
#include "Collision.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    constexpr float EPSILON = 0.0001f;
    constexpr int MAX_CCD_EVENTS = 32;
    constexpr int SOLVER_ITERATIONS = 8;
    constexpr int FALLBACK_SUBSTEPS = 16;

    float dot(sf::Vector2f a, sf::Vector2f b)
    {
        return a.x * b.x + a.y * b.y;
    }

    float ballInverseMass(const Ball& ball)
    {
        if (ball.getGroup() == PhysicsGroup::Static)
            return 0.f;

        float mass = ball.getMass();

        if (!std::isfinite(mass) || mass <= EPSILON)
            return 0.f;

        return 1.f / mass;
    }

    void advanceBall(Ball& ball, float dt)
    {
        if (ball.getGroup() != PhysicsGroup::Static)
            ball.update(dt);
    }

    void advanceBoxes(
        std::vector<Box>& boxes,
        float dt,
        PhysicsGroup group
    )
    {
        for (Box& box : boxes)
        {
            if (box.getGroup() == group)
                box.update(dt);
        }
    }

    bool isRelevantBoxPair(
        const Box& first,
        const Box& second,
        PhysicsGroup group
    )
    {
        bool firstSelected = first.getGroup() == group;
        bool secondSelected = second.getGroup() == group;

        bool firstStatic =
            first.getGroup() == PhysicsGroup::Static;

        bool secondStatic =
            second.getGroup() == PhysicsGroup::Static;

        return
            (firstSelected && secondSelected) ||
            (firstSelected && secondStatic) ||
            (secondSelected && firstStatic);
    }

    void solveSelectedBoxCollisions(
        std::vector<Box>& boxes,
        const std::vector<Wall>& walls,
        PhysicsGroup group,
        float restitution,
        int iterations
    )
    {
        if (group == PhysicsGroup::Static)
            return;

        for (int iteration = 0; iteration < iterations; ++iteration)
        {
            // Box - Wall
            for (Box& box : boxes)
            {
                if (box.getGroup() != group)
                    continue;

                for (const Wall& wall : walls)
                {
                    resolveBoxWallCollision(
                        box,
                        wall,
                        restitution
                    );
                }
            }

            // Box - Box
            for (std::size_t i = 0; i < boxes.size(); ++i)
            {
                for (std::size_t j = i + 1; j < boxes.size(); ++j)
                {
                    if (!isRelevantBoxPair(
                        boxes[i],
                        boxes[j],
                        group))
                    {
                        continue;
                    }

                    resolveBoxBoxCollision(
                        boxes[i],
                        boxes[j],
                        restitution
                    );
                }
            }
        }
    }

    bool isRelevantBallPair(
        const Ball& first,
        const Ball& second,
        PhysicsGroup group
    )
    {
        bool firstSelected = first.getGroup() == group;
        bool secondSelected = second.getGroup() == group;

        bool firstStatic =
            first.getGroup() == PhysicsGroup::Static;

        bool secondStatic =
            second.getGroup() == PhysicsGroup::Static;

        return
            (firstSelected && secondSelected) ||
            (firstSelected && secondStatic) ||
            (secondSelected && firstStatic);
    }

    void solveSelectedBallCollisions(
        std::vector<Ball>& balls,
        const std::vector<Wall>& walls,
        PhysicsGroup group,
        float restitution
    )
    {
        if (group == PhysicsGroup::Static)
            return;

        for (Ball& ball : balls)
        {
            if (ball.getGroup() != group)
                continue;

            for (const Wall& wall : walls)
            {
                resolveBallWallCollision(
                    ball,
                    wall,
                    restitution
                );
            }
        }

        for (std::size_t i = 0; i < balls.size(); ++i)
        {
            for (std::size_t j = i + 1; j < balls.size(); ++j)
            {
                if (!isRelevantBallPair(
                    balls[i],
                    balls[j],
                    group))
                {
                    continue;
                }

                // Передаём динамический шар первым,
                // чтобы направление нормали было согласованным.
                if (balls[i].getGroup() == group)
                {
                    resolveCollision(
                        balls[i],
                        balls[j],
                        restitution
                    );
                }
                else
                {
                    resolveCollision(
                        balls[j],
                        balls[i],
                        restitution
                    );
                }
            }
        }
    }

    BallSweepResult checkBallWallCCD(
        const Ball& ball,
        const Wall& wall,
        float dt
    )
    {
        BallSweepResult result;

        if (dt <= 0.f ||
            ball.getGroup() == PhysicsGroup::Static)
        {
            return result;
        }

        sf::Vector2f position = ball.getPosition();
        sf::Vector2f velocity = ball.getVelocity();

        float radius = ball.getRadius();

        if (!std::isfinite(radius) || radius <= 0.f)
            return result;

        sf::Vector2f wallPosition = wall.getPosition();
        sf::Vector2f wallSize = wall.getSize();

        float left = wallPosition.x - radius;
        float right = wallPosition.x + wallSize.x + radius;
        float top = wallPosition.y - radius;
        float bottom = wallPosition.y + wallSize.y + radius;

        float entryX;
        float exitX;
        float entryY;
        float exitY;

        if (std::abs(velocity.x) < EPSILON)
        {
            if (position.x < left || position.x > right)
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
            if (position.y < top || position.y > bottom)
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
            exitTime < 0.f ||
            entryTime < 0.f ||
            entryTime > dt)
        {
            return result;
        }

        result.hit = true;
        result.time = entryTime;

        if (entryX > entryY)
        {
            result.normal = velocity.x > 0.f
                ? sf::Vector2f(-1.f, 0.f)
                : sf::Vector2f(1.f, 0.f);
        }
        else
        {
            result.normal = velocity.y > 0.f
                ? sf::Vector2f(0.f, -1.f)
                : sf::Vector2f(0.f, 1.f);
        }

        return result;
    }

    void resolveBallWallImpact(
        Ball& ball,
        sf::Vector2f normal,
        float restitution
    )
    {
        if (ball.getGroup() == PhysicsGroup::Static)
            return;

        float velocityAlongNormal =
            dot(ball.getVelocity(), normal);

        if (velocityAlongNormal >= 0.f)
            return;

        ball.setVelocity(
            ball.getVelocity() -
            normal * ((1.f + restitution) * velocityAlongNormal)
        );
    }

    void fallbackBallStep(
        std::vector<Ball>& balls,
        const std::vector<Wall>& walls,
        float remainingTime,
        float restitution,
        PhysicsGroup group
    )
    {
        float substep =
            remainingTime / static_cast<float>(FALLBACK_SUBSTEPS);

        for (int step = 0; step < FALLBACK_SUBSTEPS; ++step)
        {
            for (Ball& ball : balls)
            {
                if (ball.getGroup() == group)
                    advanceBall(ball, substep);
            }

            solveSelectedBallCollisions(
                balls,
                walls,
                group,
                restitution
            );
        }
    }

    void fallbackBoxStep(
        std::vector<Box>& boxes,
        const std::vector<Wall>& walls,
        float remainingTime,
        float restitution,
        PhysicsGroup group
    )
    {
        float substep =
            remainingTime / static_cast<float>(FALLBACK_SUBSTEPS);

        for (int step = 0; step < FALLBACK_SUBSTEPS; ++step)
        {
            advanceBoxes(boxes, substep, group);

            solveSelectedBoxCollisions(
                boxes,
                walls,
                group,
                restitution,
                SOLVER_ITERATIONS
            );
        }
    }

    enum class EventType
    {
        None,
        BoxBox,
        BoxWall
    };

    struct CCDEvent
    {
        EventType type = EventType::None;

        float time = std::numeric_limits<float>::infinity();

        std::size_t firstBox = 0;
        std::size_t secondBox = 0;
        std::size_t wallIndex = 0;

        sf::Vector2f normal = { 0.f, 0.f };
    };
}

// =====================================================
// BALL CCD
// =====================================================

namespace CCD
{
    void updateBallWithCCD(
        Ball& ball,
        const std::vector<Wall>& walls,
        float dt,
        float restitution
    )
    {
        if (dt <= 0.f ||
            ball.getGroup() == PhysicsGroup::Static)
        {
            return;
        }

        restitution = std::clamp(restitution, 0.f, 1.f);

        float remainingTime = dt;
        int eventCount = 0;

        for (const Wall& wall : walls)
        {
            resolveBallWallCollision(
                ball,
                wall,
                restitution
            );
        }

        while (remainingTime > EPSILON &&
            eventCount < MAX_CCD_EVENTS)
        {
            BallSweepResult earliestEvent;
            earliestEvent.time =
                std::numeric_limits<float>::infinity();

            for (const Wall& wall : walls)
            {
                BallSweepResult event =
                    checkBallWallCCD(
                        ball,
                        wall,
                        remainingTime
                    );

                if (event.hit &&
                    event.time < earliestEvent.time)
                {
                    earliestEvent = event;
                }
            }

            if (!earliestEvent.hit)
            {
                advanceBall(ball, remainingTime);
                remainingTime = 0.f;
                break;
            }

            advanceBall(ball, earliestEvent.time);
            remainingTime -= earliestEvent.time;

            resolveBallWallImpact(
                ball,
                earliestEvent.normal,
                restitution
            );

            ball.setPosition(
                ball.getPosition() +
                earliestEvent.normal * EPSILON
            );

            ++eventCount;
        }

        if (remainingTime > 0.f)
        {
            // Для одиночного шара используем небольшие
            // подшаги с проверкой столкновений.
            float substep =
                remainingTime / static_cast<float>(FALLBACK_SUBSTEPS);

            for (int i = 0; i < FALLBACK_SUBSTEPS; ++i)
            {
                advanceBall(ball, substep);

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

    void updateBallsWithCCD(
        std::vector<Ball>& balls,
        const std::vector<Wall>& walls,
        float dt,
        float restitution,
        PhysicsGroup group
    )
    {
        if (dt <= 0.f || group == PhysicsGroup::Static)
            return;

        restitution = std::clamp(restitution, 0.f, 1.f);

        solveSelectedBallCollisions(
            balls,
            walls,
            group,
            restitution
        );

        float remainingTime = dt;
        int eventCount = 0;

        while (remainingTime > EPSILON &&
            eventCount < MAX_CCD_EVENTS)
        {
            BallSweepResult earliestBallEvent;
            earliestBallEvent.time =
                std::numeric_limits<float>::infinity();

            BallSweepResult earliestWallEvent;
            earliestWallEvent.time =
                std::numeric_limits<float>::infinity();

            std::size_t ballFirstIndex = 0;
            std::size_t ballSecondIndex = 0;
            std::size_t wallBallIndex = 0;

            bool ballEventFound = false;
            bool wallEventFound = false;

            // Ball - Ball
            for (std::size_t i = 0; i < balls.size(); ++i)
            {
                if (balls[i].getGroup() != group)
                    continue;

                for (std::size_t j = 0; j < balls.size(); ++j)
                {
                    if (i == j)
                        continue;

                    if (balls[j].getGroup() != group &&
                        balls[j].getGroup() != PhysicsGroup::Static)
                    {
                        continue;
                    }

                    // Динамические пары проверяем один раз.
                    if (balls[j].getGroup() == group && j < i)
                        continue;

                    BallSweepResult event =
                        checkBallBallCCD(
                            balls[i],
                            balls[j],
                            remainingTime
                        );

                    if (event.hit &&
                        event.time < earliestBallEvent.time)
                    {
                        earliestBallEvent = event;
                        ballFirstIndex = i;
                        ballSecondIndex = j;
                        ballEventFound = true;
                    }
                }
            }

            // Ball - Wall
            for (std::size_t i = 0; i < balls.size(); ++i)
            {
                if (balls[i].getGroup() != group)
                    continue;

                for (const Wall& wall : walls)
                {
                    BallSweepResult event =
                        checkBallWallCCD(
                            balls[i],
                            wall,
                            remainingTime
                        );

                    if (event.hit &&
                        event.time < earliestWallEvent.time)
                    {
                        earliestWallEvent = event;
                        wallBallIndex = i;
                        wallEventFound = true;
                    }
                }
            }

            bool useBallEvent =
                ballEventFound &&
                (!wallEventFound ||
                    earliestBallEvent.time <= earliestWallEvent.time);

            if (!ballEventFound && !wallEventFound)
            {
                for (Ball& ball : balls)
                {
                    if (ball.getGroup() == group)
                        advanceBall(ball, remainingTime);
                }

                remainingTime = 0.f;
                break;
            }

            float eventTime = useBallEvent
                ? earliestBallEvent.time
                : earliestWallEvent.time;

            for (Ball& ball : balls)
            {
                if (ball.getGroup() == group)
                    advanceBall(ball, eventTime);
            }

            remainingTime -= eventTime;

            if (useBallEvent)
            {
                resolveBallBallImpact(
                    balls[ballFirstIndex],
                    balls[ballSecondIndex],
                    earliestBallEvent.normal,
                    restitution
                );

                float invA = ballInverseMass(balls[ballFirstIndex]);
                float invB = ballInverseMass(balls[ballSecondIndex]);

                float totalInvMass = invA + invB;

                if (totalInvMass > EPSILON)
                {
                    sf::Vector2f correction =
                        earliestBallEvent.normal * EPSILON;

                    if (invA > 0.f)
                    {
                        balls[ballFirstIndex].setPosition(
                            balls[ballFirstIndex].getPosition() -
                            correction * (invA / totalInvMass)
                        );
                    }

                    if (invB > 0.f)
                    {
                        balls[ballSecondIndex].setPosition(
                            balls[ballSecondIndex].getPosition() +
                            correction * (invB / totalInvMass)
                        );
                    }
                }
            }
            else
            {
                resolveBallWallImpact(
                    balls[wallBallIndex],
                    earliestWallEvent.normal,
                    restitution
                );

                balls[wallBallIndex].setPosition(
                    balls[wallBallIndex].getPosition() +
                    earliestWallEvent.normal * EPSILON
                );
            }

            ++eventCount;
        }

        if (remainingTime > 0.f)
        {
            fallbackBallStep(
                balls,
                walls,
                remainingTime,
                restitution,
                group
            );
        }

        solveSelectedBallCollisions(
            balls,
            walls,
            group,
            restitution
        );
    }

    // =================================================
    // BOX CCD
    // =================================================

    void updateBoxesWithCCD(
        std::vector<Box>& boxes,
        const std::vector<Wall>& walls,
        float dt,
        float restitution,
        PhysicsGroup group
    )
    {
        if (dt <= 0.f ||
            group == PhysicsGroup::Static)
        {
            return;
        }

        restitution = std::clamp(restitution, 0.f, 1.f);

        solveSelectedBoxCollisions(
            boxes,
            walls,
            group,
            restitution,
            SOLVER_ITERATIONS
        );

        float remainingTime = dt;
        int eventCount = 0;

        while (remainingTime > EPSILON &&
            eventCount < MAX_CCD_EVENTS)
        {
            CCDEvent earliestEvent;

            // Box - Box
            for (std::size_t i = 0; i < boxes.size(); ++i)
            {
                if (boxes[i].getGroup() != group)
                    continue;

                for (std::size_t j = 0; j < boxes.size(); ++j)
                {
                    if (i == j)
                        continue;

                    if (boxes[j].getGroup() != group &&
                        boxes[j].getGroup() != PhysicsGroup::Static)
                    {
                        continue;
                    }

                    if (boxes[j].getGroup() == group && j < i)
                        continue;

                    BoxSweepResult result =
                        checkBoxBoxCCD(
                            boxes[i],
                            boxes[j],
                            remainingTime
                        );

                    if (result.hit &&
                        result.time < earliestEvent.time)
                    {
                        earliestEvent.type = EventType::BoxBox;
                        earliestEvent.time = result.time;
                        earliestEvent.firstBox = i;
                        earliestEvent.secondBox = j;
                        earliestEvent.normal = result.normal;
                    }
                }
            }

            // Box - Wall
            for (std::size_t i = 0; i < boxes.size(); ++i)
            {
                if (boxes[i].getGroup() != group)
                    continue;

                for (std::size_t j = 0; j < walls.size(); ++j)
                {
                    BoxWallSweepResult result =
                        checkBoxWallCCD(
                            boxes[i],
                            walls[j],
                            remainingTime
                        );

                    if (result.hit &&
                        result.time < earliestEvent.time)
                    {
                        earliestEvent.type = EventType::BoxWall;
                        earliestEvent.time = result.time;
                        earliestEvent.firstBox = i;
                        earliestEvent.wallIndex = j;
                        earliestEvent.normal = result.normal;
                    }
                }
            }

            if (earliestEvent.type == EventType::None)
            {
                advanceBoxes(
                    boxes,
                    remainingTime,
                    group
                );

                remainingTime = 0.f;
                break;
            }

            advanceBoxes(
                boxes,
                earliestEvent.time,
                group
            );

            remainingTime -= earliestEvent.time;

            if (earliestEvent.type == EventType::BoxBox)
            {
                resolveBoxBoxImpact(
                    boxes[earliestEvent.firstBox],
                    boxes[earliestEvent.secondBox],
                    earliestEvent.normal,
                    restitution
                );
            }
            else
            {
                resolveBoxWallImpact(
                    boxes[earliestEvent.firstBox],
                    earliestEvent.normal,
                    restitution
                );
            }

            solveSelectedBoxCollisions(
                boxes,
                walls,
                group,
                restitution,
                SOLVER_ITERATIONS
            );

            ++eventCount;
        }

        if (remainingTime > 0.f)
        {
            fallbackBoxStep(
                boxes,
                walls,
                remainingTime,
                restitution,
                group
            );
        }

        solveSelectedBoxCollisions(
            boxes,
            walls,
            group,
            restitution,
            SOLVER_ITERATIONS
        );
    }
}
// =====================================================
// GLOBAL CCD FOR MULTI-RATE
// =====================================================

namespace CCD
{
    // -------------------------------------------------
    // Общая обработка столкновений шаров
    // -------------------------------------------------

    void updateAllBallsWithCCD(
        std::vector<Ball>& balls,
        const std::vector<Wall>& walls,
        float dt,
        float restitution
    )
    {
        if (dt <= 0.f)
            return;

        restitution = std::clamp(restitution, 0.f, 1.f);

        auto advanceAll = [&](float time)
            {
                for (Ball& ball : balls)
                {
                    if (ball.getGroup() != PhysicsGroup::Static)
                        ball.update(time);
                }
            };

        auto solveAll = [&]()
            {
                // Несколько проходов для устранения пересечений.
                for (int iteration = 0; iteration < 4; ++iteration)
                {
                    for (std::size_t i = 0; i < balls.size(); ++i)
                    {
                        for (std::size_t j = i + 1; j < balls.size(); ++j)
                        {
                            if (balls[i].getGroup() == PhysicsGroup::Static &&
                                balls[j].getGroup() == PhysicsGroup::Static)
                            {
                                continue;
                            }

                            resolveCollision(
                                balls[i],
                                balls[j],
                                restitution
                            );
                        }
                    }

                    for (Ball& ball : balls)
                    {
                        if (ball.getGroup() == PhysicsGroup::Static)
                            continue;

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
            };

        solveAll();

        float remainingTime = dt;
        int eventCount = 0;

        while (remainingTime > EPSILON &&
            eventCount < MAX_CCD_EVENTS)
        {
            enum class Type
            {
                None,
                BallBall,
                BallWall
            };

            Type eventType = Type::None;

            float earliestTime =
                std::numeric_limits<float>::infinity();

            std::size_t firstIndex = 0;
            std::size_t secondIndex = 0;
            std::size_t wallIndex = 0;

            sf::Vector2f eventNormal(0.f, 0.f);

            // Поиск столкновений шаров между собой.
            for (std::size_t i = 0; i < balls.size(); ++i)
            {
                if (balls[i].getGroup() == PhysicsGroup::Static)
                    continue;

                for (std::size_t j = i + 1; j < balls.size(); ++j)
                {
                    if (balls[j].getGroup() == PhysicsGroup::Static &&
                        balls[i].getGroup() == PhysicsGroup::Static)
                    {
                        continue;
                    }

                    BallSweepResult result =
                        checkBallBallCCD(
                            balls[i],
                            balls[j],
                            remainingTime
                        );

                    if (result.hit && result.time < earliestTime)
                    {
                        earliestTime = result.time;
                        eventType = Type::BallBall;
                        firstIndex = i;
                        secondIndex = j;
                        eventNormal = result.normal;
                    }
                }
            }

            // Поиск столкновений шаров со стенами.
            for (std::size_t i = 0; i < balls.size(); ++i)
            {
                if (balls[i].getGroup() == PhysicsGroup::Static)
                    continue;

                for (std::size_t j = 0; j < walls.size(); ++j)
                {
                    BallSweepResult result =
                        checkBallWallCCD(
                            balls[i],
                            walls[j],
                            remainingTime
                        );

                    if (result.hit && result.time < earliestTime)
                    {
                        earliestTime = result.time;
                        eventType = Type::BallWall;
                        firstIndex = i;
                        wallIndex = j;
                        eventNormal = result.normal;
                    }
                }
            }

            // Столкновений больше нет.
            if (eventType == Type::None)
            {
                advanceAll(remainingTime);
                remainingTime = 0.f;
                break;
            }

            // Перемещаем ВСЕ динамические шары
            // к одному и тому же моменту столкновения.
            advanceAll(earliestTime);
            remainingTime -= earliestTime;

            if (eventType == Type::BallBall)
            {
                resolveBallBallImpact(
                    balls[firstIndex],
                    balls[secondIndex],
                    eventNormal,
                    restitution
                );

                float invA = ballInverseMass(balls[firstIndex]);
                float invB = ballInverseMass(balls[secondIndex]);

                float totalInvMass = invA + invB;

                if (totalInvMass > EPSILON)
                {
                    sf::Vector2f correction =
                        eventNormal * EPSILON;

                    if (invA > 0.f)
                    {
                        balls[firstIndex].setPosition(
                            balls[firstIndex].getPosition() -
                            correction * (invA / totalInvMass)
                        );
                    }

                    if (invB > 0.f)
                    {
                        balls[secondIndex].setPosition(
                            balls[secondIndex].getPosition() +
                            correction * (invB / totalInvMass)
                        );
                    }
                }
            }
            else
            {
                resolveBallWallImpact(
                    balls[firstIndex],
                    eventNormal,
                    restitution
                );

                balls[firstIndex].setPosition(
                    balls[firstIndex].getPosition() +
                    eventNormal * EPSILON
                );
            }

            ++eventCount;
        }

        // Запасной режим, если превышено число событий CCD.
        if (remainingTime > 0.f)
        {
            float substep =
                remainingTime / static_cast<float>(FALLBACK_SUBSTEPS);

            for (int step = 0; step < FALLBACK_SUBSTEPS; ++step)
            {
                advanceAll(substep);
                solveAll();
            }
        }

        solveAll();
    }

    // -------------------------------------------------
    // Общая обработка столкновений прямоугольников
    // -------------------------------------------------

    void updateAllBoxesWithCCD(
        std::vector<Box>& boxes,
        const std::vector<Wall>& walls,
        float dt,
        float restitution
    )
    {
        if (dt <= 0.f)
            return;

        restitution = std::clamp(restitution, 0.f, 1.f);

        auto advanceAll = [&](float time)
            {
                for (Box& box : boxes)
                {
                    if (box.getGroup() != PhysicsGroup::Static)
                        box.update(time);
                }
            };

        auto solveAll = [&]()
            {
                ::solveBoxCollisions(
                    boxes,
                    walls,
                    restitution,
                    SOLVER_ITERATIONS
                );
            };

        solveAll();

        float remainingTime = dt;
        int eventCount = 0;

        while (remainingTime > EPSILON &&
            eventCount < MAX_CCD_EVENTS)
        {
            CCDEvent earliestEvent;

            // Box - Box: рассматриваем все динамические группы.
            for (std::size_t i = 0; i < boxes.size(); ++i)
            {
                if (boxes[i].getGroup() == PhysicsGroup::Static)
                    continue;

                for (std::size_t j = i + 1; j < boxes.size(); ++j)
                {
                    if (boxes[j].getGroup() == PhysicsGroup::Static &&
                        boxes[i].getGroup() == PhysicsGroup::Static)
                    {
                        continue;
                    }

                    BoxSweepResult result =
                        checkBoxBoxCCD(
                            boxes[i],
                            boxes[j],
                            remainingTime
                        );

                    if (result.hit && result.time < earliestEvent.time)
                    {
                        earliestEvent.type = EventType::BoxBox;
                        earliestEvent.time = result.time;
                        earliestEvent.firstBox = i;
                        earliestEvent.secondBox = j;
                        earliestEvent.normal = result.normal;
                    }
                }
            }

            // Box - Wall.
            for (std::size_t i = 0; i < boxes.size(); ++i)
            {
                if (boxes[i].getGroup() == PhysicsGroup::Static)
                    continue;

                for (std::size_t j = 0; j < walls.size(); ++j)
                {
                    BoxWallSweepResult result =
                        checkBoxWallCCD(
                            boxes[i],
                            walls[j],
                            remainingTime
                        );

                    if (result.hit && result.time < earliestEvent.time)
                    {
                        earliestEvent.type = EventType::BoxWall;
                        earliestEvent.time = result.time;
                        earliestEvent.firstBox = i;
                        earliestEvent.wallIndex = j;
                        earliestEvent.normal = result.normal;
                    }
                }
            }

            if (earliestEvent.type == EventType::None)
            {
                advanceAll(remainingTime);
                remainingTime = 0.f;
                break;
            }

            // Все динамические прямоугольники перемещаются
            // к одному моменту времени.
            advanceAll(earliestEvent.time);
            remainingTime -= earliestEvent.time;

            if (earliestEvent.type == EventType::BoxBox)
            {
                resolveBoxBoxImpact(
                    boxes[earliestEvent.firstBox],
                    boxes[earliestEvent.secondBox],
                    earliestEvent.normal,
                    restitution
                );
            }
            else
            {
                resolveBoxWallImpact(
                    boxes[earliestEvent.firstBox],
                    earliestEvent.normal,
                    restitution
                );
            }

            solveAll();

            ++eventCount;
        }

        // Запасные подшаги при большом количестве столкновений.
        if (remainingTime > 0.f)
        {
            float substep =
                remainingTime / static_cast<float>(FALLBACK_SUBSTEPS);

            for (int step = 0; step < FALLBACK_SUBSTEPS; ++step)
            {
                advanceAll(substep);
                solveAll();
            }
        }

        solveAll();
    }
}