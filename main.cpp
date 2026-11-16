
#include <SFML/Graphics.hpp>
#include "Ball.h"
#include "Wall.h"
#include "Collision.h"

int main()
{
    sf::RenderWindow window(
        sf::VideoMode(1000, 700),
        "2D Physics Engine"
    );

    window.setFramerateLimit(60);

    Ball ball1(
        sf::Vector2f(200.f, 300.f),
        sf::Vector2f(150.f, 0.f),
        25.f,
        1.f
    );

    Ball ball2(
        sf::Vector2f(600.f, 300.f),
        sf::Vector2f(-150.f, 0.f),
        25.f,
        1.f
    );

    Wall wall(sf::Vector2f(600.f, 100.f), sf::Vector2f(15.f, 450.f));
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

        ball1.update(dt);
        ball2.update(dt);
        resolveCollision(ball1, ball2, 0.8f);

        window.clear(sf::Color(30, 30, 40));

        ball1.draw(window);
        ball2.draw(window);
        wall.draw(window);

        window.display();
    }

    return 0;
}
