
#include <SFML/Graphics.hpp>
#include "CCD.h"
#include "World.h"

int main()
{
    sf::RenderWindow window(
        sf::VideoMode(1000, 700),
        "2D Physics Engine"
    );

    window.setFramerateLimit(60);

    World world;

    world.addWall(Wall(
        sf::Vector2f(100.f, 100.f),
        sf::Vector2f(10.f, 450.f)
    ));

    world.addWall(Wall(
        sf::Vector2f(600.f, 100.f),
        sf::Vector2f(10.f, 450.f)
    ));

    world.addWall(Wall(
        sf::Vector2f(200.f, 500.f),
        sf::Vector2f(400.f, 10.f)
    ));

    world.addBall(Ball(
        sf::Vector2f(300.f, 300.f),
        sf::Vector2f(100.f, 0.f),
        20.f,
        1.f
    ));

    world.addBall(Ball(
        sf::Vector2f(335.f, 300.f),
        sf::Vector2f(-100.f, 0.f),
        20.f,
        1.f
    ));

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

        // Обновляем физический мир
        //world.update(dt, 0.8f);
        world.updateWithFullCCD(dt, 0.8f);

        window.clear(sf::Color(30, 30, 40));

        // Рисуем все объекты
        world.draw(window);

        window.display();
    }

    return 0;
}
