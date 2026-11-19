
#include <SFML/Graphics.hpp>

#include "World.h"
#include "PhysicsGroup.h"

int main()
{
    sf::RenderWindow window(
        sf::VideoMode(1000, 700),
        "2D Physics Engine"
    );

    window.setFramerateLimit(60);

    World world;

    // =============================================
    // WALLS
    // =============================================

    world.addWall(
        Wall(
            sf::Vector2f(100.f, 100.f),
            sf::Vector2f(10.f, 450.f)
        )
    );

    world.addWall(
        Wall(
            sf::Vector2f(600.f, 100.f),
            sf::Vector2f(10.f, 450.f)
        )
    );

    world.addWall(
        Wall(
            sf::Vector2f(200.f, 500.f),
            sf::Vector2f(400.f, 10.f)
        )
    );

    // =============================================
    // BOXES
    // =============================================

    Box box(
        sf::Vector2f(400.f, 300.f),
        sf::Vector2f(40.f, 40.f),
        sf::Vector2f(0.f, 0.f),
        2.f
    );

    box.setGroup(PhysicsGroup::Fast);

    world.addBox(box);

    // =============================================
    // BALLS
    // =============================================

    // Пример создания шара:
    //
    // Ball ball(
    //     sf::Vector2f(300.f, 200.f),
    //     sf::Vector2f(200.f, 0.f),
    //     10.f,
    //     1.f
    // );
    //
    // ball.setGroup(PhysicsGroup::Normal);
    // world.addBall(ball);

    // =============================================
    // PHYSICS MODE
    // =============================================

    sf::Clock clock;

    while (window.isOpen())
    {
        sf::Event event;

        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }

        float dt = clock.restart().asSeconds();

        // Выбирай только один режим.

        // world.update(dt, 0.8f);

        // world.updateWithCCD(dt, 0.8f);

        world.updateWithFullCCD(dt, 0.8f);

        // world.updateMultiRate(dt, 0.8f);

        window.clear(sf::Color(30, 30, 40));

        world.draw(window);

        window.display();
    }

    return 0;
}
