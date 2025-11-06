#include "F1_Physics_build_2.h"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <vector>

class F1Simulator {
private:
    sf::RenderWindow window;
    sf::Font font;
    sf::Text infoText;

    F1PhysicsEngine f1_engine;
    std::atomic<bool> running;
    std::atomic<bool> gas_pressed;
    std::atomic<bool> brake_pressed;
    std::atomic<double> steering;

    std::ofstream csv_file;
    int frame_counter;
    std::chrono::steady_clock::time_point start_time;

    // Графические элементы
    sf::RectangleShape rpmBar;
    sf::RectangleShape rpmBarBackground;

public:
    F1Simulator()
        : window(sf::VideoMode({1200, 800}), "F1 Physics Simulation") // Изменено для SFML 3.0
        , infoText(font, "", 16) // Добавлена инициализация
        , running(true)
        , gas_pressed(false)
        , brake_pressed(false)
        , steering(0.0)
        , frame_counter(0)
    {
        setupWindow();
        setupCSV();
        start_time = std::chrono::steady_clock::now();
    }

    void setupWindow() {
        // Загрузка шрифта
        if (!font.openFromFile("C:/Windows/Fonts/arial.ttf")) { // Изменено на openFromFile
            font.openFromFile("arial.ttf");
        }

        infoText.setFont(font);
        infoText.setCharacterSize(16);
        infoText.setFillColor(sf::Color::White);
        infoText.setPosition({10.f, 10.f}); // Изменено для SFML 3.0

        // Настройка прогресс-бара RPM
        rpmBarBackground.setSize({400.f, 20.f}); // Изменено для SFML 3.0
        rpmBarBackground.setFillColor(sf::Color(50, 50, 50));
        rpmBarBackground.setPosition({10.f, 750.f}); // Изменено для SFML 3.0

        rpmBar.setSize({0.f, 20.f}); // Изменено для SFML 3.0
        rpmBar.setFillColor(sf::Color::Green);
        rpmBar.setPosition({10.f, 750.f}); // Изменено для SFML 3.0

        window.setFramerateLimit(60);
    }

    void setupCSV() {
        csv_file.open("f1_simulation_log.csv");
        csv_file << "Time(s);CenterMass_X(m);CenterMass_Y(m);Speed(km/h);Accel_X(m/s2);Accel_Y(m/s2);"
                 << "Car_Angle(deg);Steering_Angle(deg);Gear;"
                 << "Engine_RPM;Wheel_RPM;Traction_Force(N);Drag_Force(N);Brake_Force(N);"
                 << "FL_Actual_X;FL_Actual_Y;FR_Actual_X;FR_Actual_Y;RL_Actual_X;RL_Actual_Y;RR_Actual_X;RR_Actual_Y;"
                 << "FL_Vel_X;FL_Vel_Y;FR_Vel_X;FR_Vel_Y;RL_Vel_X;RL_Vel_Y;RR_Vel_X;RR_Vel_Y;"
                 << "FL_Ideal_X;FL_Ideal_Y;FR_Ideal_X;FR_Ideal_Y;RL_Ideal_X;RL_Ideal_Y;RR_Ideal_X;RR_Ideal_Y;"
                 << "FL_Error(m);FR_Error(m);RL_Error(m);RR_Error(m)\n";
    }

    void run() {
        // Запуск потока физики
        std::thread physics_thread([&]() {
            while (running) {
                f1_engine.update(0.01, gas_pressed, brake_pressed, steering);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // Основной цикл рендеринга
        while (window.isOpen()) {
            handleEvents();
            update();
            render();
        }

        // Завершение работы
        running = false;
        if (physics_thread.joinable()) {
            physics_thread.join();
        }

        csv_file.close();
        std::cout << "F1 Physics simulation stopped." << std::endl;
        std::cout << "Data saved to f1_simulation_log.csv" << std::endl;
    }

private:
    void handleEvents() {
        // В SFML 3.0 обработка событий изменилась
        for (auto event = window.pollEvent(); event.has_value(); event = window.pollEvent()) {
            const auto& ev = event.value();

            if (ev.is<sf::Event::Closed>()) {
                window.close();
                running = false;
            }

            // Обработка нажатий клавиш
            if (const auto* keyEvent = ev.getIf<sf::Event::KeyPressed>()) {
                handleKeyPress(keyEvent->scancode, true);
            }
            if (const auto* keyEvent = ev.getIf<sf::Event::KeyReleased>()) {
                handleKeyPress(keyEvent->scancode, false);
            }
        }

        // Непрерывная обработка для плавного управления
        handleContinuousInput();
    }

    void handleKeyPress(sf::Keyboard::Scancode scancode, bool pressed) {
        switch (scancode) {
            case sf::Keyboard::Scancode::W:
                if (pressed) gas_pressed = !gas_pressed;
                break;

            case sf::Keyboard::Scancode::S:
                if (pressed) brake_pressed = !brake_pressed;
                break;

            case sf::Keyboard::Scancode::A:
                if (pressed) steering = std::max(steering - 0.1, -1.0);
                break;

            case sf::Keyboard::Scancode::D:
                if (pressed) steering = std::min(steering + 0.1, 1.0);
                break;

            case sf::Keyboard::Scancode::Space:
                if (pressed) steering = 0.0;
                break;

            case sf::Keyboard::Scancode::Left:
                if (pressed) f1_engine.shiftDown();
                break;

            case sf::Keyboard::Scancode::Right:
                if (pressed) f1_engine.shiftUp();
                break;

            case sf::Keyboard::Scancode::R:
                if (pressed) {
                    f1_engine.reset();
                    gas_pressed = false;
                    brake_pressed = false;
                    steering = 0.0;
                    start_time = std::chrono::steady_clock::now();
                }
                break;

            case sf::Keyboard::Scancode::Escape:
                window.close();
                running = false;
                break;

            default:
                break;
        }
    }

    void handleContinuousInput() {
        // Плавное управление рулем при зажатых клавишах
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::A)) {
            steering = std::max(steering - 0.05, -1.0);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::D)) {
            steering = std::min(steering + 0.05, 1.0);
        }
    }

    void update() {
        // Логирование в CSV каждые 30 кадров
        frame_counter++;
        if (frame_counter >= 30) {
            logToCSV();
            frame_counter = 0;
        }
    }

    void render() {
        window.clear(sf::Color(30, 30, 50)); // Темно-синий фон

        auto state = f1_engine.getState();

        // Формируем текст для отображения
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);

        ss << "=== FORMULA 1 PHYSICS SIMULATION ===\n";
        ss << "=====================================\n\n";

        ss << "CENTER OF MASS:\n";
        ss << "  Speed: " << state.speed * 3.6 << " km/h\n";
        ss << "  Position: (" << state.position.x << ", " << state.position.y << ") m\n";
        ss << "  Acceleration: (" << state.acceleration.x << ", " << state.acceleration.y << ") m/s²\n\n";

        ss << "ANGLES:\n";
        ss << "  Car Angle: " << state.car_angle * 180.0 / M_PI << "°\n";
        ss << "  Steering Wheel: " << steering * 180.0 / M_PI << "°\n\n";

        ss << "ENGINE AND TRANSMISSION:\n";
        ss << "  Gear: " << state.current_gear << "\n";
        ss << "  Engine RPM: " << static_cast<int>(state.engine_rpm) << "\n";
        ss << "  Wheel RPM: " << state.wheel_rpm << "\n\n";

        ss << "FORCES:\n";
        ss << "  Traction: " << state.traction_force << " N\n";
        ss << "  Drag: " << state.drag_force << " N\n";
        ss << "  Brake: " << state.brake_force << " N\n";
        ss << "  Downforce: " << state.down_force << " N\n\n";

        // Координаты колес
        ss << "WHEEL POSITIONS:\n";
        const char* wheel_names[] = {"FL", "FR", "RL", "RR"};
        std::array<F1PhysicsEngine::Point2D, 4> ideal_positions;
        for (int i = 0; i < 4; i++) {
            ideal_positions[i] = f1_engine.calculateIdealWheelPosition(i);
            double dx = state.wheel_positions[i].x - ideal_positions[i].x;
            double dy = state.wheel_positions[i].y - ideal_positions[i].y;
            double error = std::sqrt(dx*dx + dy*dy);

            ss << "  " << wheel_names[i] << ": (" << state.wheel_positions[i].x << ", " << state.wheel_positions[i].y
               << ") Error: " << error << " m\n";
        }
        ss << "\n";

        // Управление
        ss << "CONTROLS:\n";
        ss << "  W - Gas: " << (gas_pressed ? "ON" : "OFF") << "\n";
        ss << "  S - Brake: " << (brake_pressed ? "ON" : "OFF") << "\n";
        ss << "  A/D - Steering | SPACE - Center\n";
        ss << "  LEFT/RIGHT - Gear shift | R - Reset | ESC - Exit\n";

        infoText.setString(ss.str());

        // Обновление прогресс-бара RPM
        double rpm_percentage = (state.engine_rpm / 15000.0) * 100.0;
        rpmBar.setSize({400.f * static_cast<float>(rpm_percentage / 100.0), 20.f}); // Изменено для SFML 3.0

        // Изменение цвета прогресс-бара в зависимости от RPM
        if (rpm_percentage < 70) {
            rpmBar.setFillColor(sf::Color::Green);
        } else if (rpm_percentage < 90) {
            rpmBar.setFillColor(sf::Color::Yellow);
        } else {
            rpmBar.setFillColor(sf::Color::Red);
        }

        // Отрисовка
        window.draw(infoText);
        window.draw(rpmBarBackground);
        window.draw(rpmBar);

        // Отображение процентов RPM
        sf::Text rpmText(font, "RPM: " + std::to_string(static_cast<int>(state.engine_rpm)) +
                         " (" + std::to_string(static_cast<int>(rpm_percentage)) + "%)", 14); // Изменено для SFML 3.0
        rpmText.setFillColor(sf::Color::White);
        rpmText.setPosition({420.f, 750.f}); // Изменено для SFML 3.0
        window.draw(rpmText);

        window.display();
    }

    void logToCSV() {
        auto state = f1_engine.getState();
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);

        csv_file << std::fixed << std::setprecision(6);
        csv_file << elapsed.count() / 1000.0 << ";";
        csv_file << state.position.x << ";" << state.position.y << ";"
                 << state.speed * 3.6 << ";"
                 << state.acceleration.x << ";" << state.acceleration.y << ";";
        csv_file << state.car_angle * 180.0 / M_PI << ";"
                 << steering * 180.0 / M_PI << ";"
                 << state.current_gear << ";";
        csv_file << state.engine_rpm << ";" << state.wheel_rpm << ";"
                 << state.traction_force << ";" << state.drag_force << ";" << state.brake_force << ";";

        // Координаты колес
        for (int i = 0; i < 4; i++) {
            csv_file << state.wheel_positions[i].x << ";" << state.wheel_positions[i].y;
            if (i < 3) csv_file << ";";
        }
        csv_file << ";";

        // Скорости колес
        for (int i = 0; i < 4; i++) {
            csv_file << state.wheel_velocity[i].x << ";" << state.wheel_velocity[i].y;
            if (i < 3) csv_file << ";";
        }
        csv_file << ";";

        // Идеальные координаты
        std::array<F1PhysicsEngine::Point2D, 4> ideal_positions;
        for (int i = 0; i < 4; i++) {
            ideal_positions[i] = f1_engine.calculateIdealWheelPosition(i);
            csv_file << ideal_positions[i].x << ";" << ideal_positions[i].y;
            if (i < 3) csv_file << ";";
        }
        csv_file << ";";

        // Ошибки позиционирования
        for (int i = 0; i < 4; i++) {
            double dx = state.wheel_positions[i].x - ideal_positions[i].x;
            double dy = state.wheel_positions[i].y - ideal_positions[i].y;
            double error = std::sqrt(dx*dx + dy*dy);
            csv_file << error;
            if (i < 3) csv_file << ";";
        }
        csv_file << "\n";
    }
};

int main() {
    try {
        F1Simulator simulator;
        simulator.run();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}