
#include <SFML/Graphics.hpp>
#include "Ball.h"

int main()
{
    sf::RenderWindow window(
        sf::VideoMode(1000, 700),
        "2D Physics Engine"
    );

    window.setFramerateLimit(60);

    Ball ball(
        sf::Vector2f(200.f, 200.f),
        sf::Vector2f(100.f, 50.f),
        20.f,
        1.f
    );

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

        ball.update(dt);

        window.clear(sf::Color(30, 30, 40));

        ball.draw(window);

        window.display();
    }

    return 0;
}
