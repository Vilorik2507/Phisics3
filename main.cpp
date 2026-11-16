
#include <SFML/Graphics.hpp>

int main()
{
    // Создаем окно размером 1000 на 700 пикселей
    sf::RenderWindow window(
        sf::VideoMode(1000, 700),
        "2D Physics Engine"
    );

    // Ограничиваем частоту кадров
    window.setFramerateLimit(60);

    // Главный цикл приложения
    while (window.isOpen())
    {
        sf::Event event;

        // Обрабатываем события окна
        while (window.pollEvent(event))
        {
            // Проверяем, нажал ли пользователь крестик
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }

        // Очищаем экран
        window.clear(sf::Color(30, 30, 40));

        // Показываем изображение
        window.display();
    }

    return 0;
}
