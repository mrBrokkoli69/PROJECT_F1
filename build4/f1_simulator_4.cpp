#include "F1_Physics_build_2.h"
#include <ncurses.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

int main() {
    // Инициализация ncurses
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);  // НЕБЛОКИРУЮЩИЙ ввод
    curs_set(0);
    
    // Создаем физический движок F1
    F1PhysicsEngine f1_engine;
    
    std::atomic<bool> running(true);
    std::atomic<bool> gas_pressed(false);
    std::atomic<bool> brake_pressed(false);
    double steering = 0.0;
    
    // Переменные для отслеживания состояния клавиш (чтобы избежать двойного срабатывания)
    bool w_was_pressed = false;
    bool s_was_pressed = false;
    bool a_was_pressed = false;
    bool d_was_pressed = false;
    
    // CSV файл для логирования
    std::ofstream csv_file("f1_simulation_log.csv");
    
    // Читаемые заголовки CSV с группировкой
    csv_file << "Time(s),"
             
             // Центр масс
             << "CenterMass_X(m),CenterMass_Y(m),Speed(km/h),Accel_X(m/s2),Accel_Y(m/s2),"
             
             // Углы и управление
             << "Car_Angle(deg),Steering_Angle(deg),Gear,"
             
             // Двигатель
             << "Engine_RPM,Wheel_RPM,Traction_Force(N),Drag_Force(N),Brake_Force(N),"
             
             // Координаты колес - ФАКТИЧЕСКИЕ
             << "FL_Actual_X,FL_Actual_Y,FR_Actual_X,FR_Actual_Y,RL_Actual_X,RL_Actual_Y,RR_Actual_X,RR_Actual_Y,"
             
             // Скорости колес
             << "FL_Vel_X,FL_Vel_Y,FR_Vel_X,FR_Vel_Y,RL_Vel_X,RL_Vel_Y,RR_Vel_X,RR_Vel_Y,"
             
             // Координаты колес - ИДЕАЛЬНЫЕ
             << "FL_Ideal_X,FL_Ideal_Y,FR_Ideal_X,FR_Ideal_Y,RL_Ideal_X,RL_Ideal_Y,RR_Ideal_X,RR_Ideal_Y,"
             
             // Ошибки позиционирования
             << "FL_Error(m),FR_Error(m),RL_Error(m),RR_Error(m)"
             << "\n";
    
    int frame_counter = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    // Поток для обновления физики
    std::thread physics_thread([&]() {
        while (running) {
            // Обновляем физику с временным шагом 0.01 секунды
            f1_engine.update(0.01, gas_pressed, brake_pressed, steering);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
    // Основной цикл обработки ввода и вывода
    while (running) {
        clear();
        
        // Получаем текущее состояние автомобиля
        auto state = f1_engine.getState();
        
        // Вычисляем идеальные позиции используя метод класса
        std::array<F1PhysicsEngine::Point2D, 4> ideal_positions;
        for (int i = 0; i < 4; i++) {
            ideal_positions[i] = f1_engine.calculateIdealWheelPosition(i);
        }
        
        // Вычисляем расхождения между реальными и идеальными позициями
        std::array<double, 4> position_errors;
        for (int i = 0; i < 4; i++) {
            double dx = state.wheel_positions[i].x - ideal_positions[i].x;
            double dy = state.wheel_positions[i].y - ideal_positions[i].y;
            position_errors[i] = std::sqrt(dx*dx + dy*dy);
        }
        
        // === ОТОБРАЖЕНИЕ ИНФОРМАЦИИ В NCURSES ===
        
        // Заголовок
        mvprintw(0, 0, "=== FORMULA 1 PHYSICS SIMULATION ===");
        mvprintw(1, 0, "=====================================");
        
        // Скорость и позиция центра масс
        mvprintw(2, 0, "CENTER OF MASS:");
        mvprintw(3, 2, "Speed: %.1f km/h", state.speed * 3.6);
        mvprintw(4, 2, "Position: (%.1f, %.1f) m", state.position.x, state.position.y);
        mvprintw(5, 2, "Acceleration: (%.1f, %.1f) m/s²", state.acceleration.x, state.acceleration.y);
        
        // Углы
        mvprintw(7, 0, "ANGLES:");
        mvprintw(8, 2, "Car Angle: %.1f°", state.car_angle * 180.0 / M_PI);
        mvprintw(9, 2, "Steering Wheel: %.1f°", steering * 180.0 / M_PI);
        
        // Двигатель и трансмиссия
        mvprintw(11, 0, "ENGINE AND TRANSMISSION:");
        mvprintw(12, 2, "Gear: %d", state.current_gear);
        mvprintw(13, 2, "Engine RPM: %.0f", state.engine_rpm);
        mvprintw(14, 2, "Wheel RPM: %.1f", state.wheel_rpm);
        
        // Силы
        mvprintw(16, 0, "FORCES:");
        mvprintw(17, 2, "Traction: %.1f N", state.traction_force);
        mvprintw(18, 2, "Drag: %.1f N", state.drag_force);
        mvprintw(19, 2, "Brake: %.1f N", state.brake_force);
        
        // Координаты колес
        mvprintw(21, 0, "WHEEL POSITIONS (Real / Ideal):");
        const char* wheel_names[] = {"FL", "FR", "RL", "RR"};
        for (int i = 0; i < 4; i++) {
            mvprintw(22 + i, 2, "%s: (%.2f,%.2f) / (%.2f,%.2f) Error: %.3f m", 
                    wheel_names[i],
                    state.wheel_positions[i].x, state.wheel_positions[i].y,
                    ideal_positions[i].x, ideal_positions[i].y,
                    position_errors[i]);
        }
        
        // Скорости колес
        mvprintw(27, 0, "WHEEL VELOCITIES:");
        for (int i = 0; i < 4; i++) {
            double wheel_speed = std::sqrt(state.wheel_velocity[i].x * state.wheel_velocity[i].x + 
                                          state.wheel_velocity[i].y * state.wheel_velocity[i].y);
            mvprintw(28 + i, 2, "%s: (%.1f,%.1f) m/s | Total: %.1f m/s", 
                    wheel_names[i],
                    state.wheel_velocity[i].x, state.wheel_velocity[i].y,
                    wheel_speed);
        }
        
        // Управление
        mvprintw(33, 0, "CONTROLS:");
        mvprintw(34, 2, "W - Gas: %s", gas_pressed ? "ON" : "OFF");
        mvprintw(35, 2, "S - Brake: %s", brake_pressed ? "ON" : "OFF");
        mvprintw(36, 2, "A/D - Steering: %.1f° (A-left, D-right, Space-center)", steering * 180.0 / M_PI);
        mvprintw(37, 2, "LEFT/RIGHT - Gear shift");
        mvprintw(38, 2, "R - Reset | ESC - Exit");
        mvprintw(39, 2, "W/S work as TOGGLE buttons (press to turn ON/OFF)");
        
        // Прогресс оборотов
        double rpm_progress = (state.engine_rpm / 15000.0) * 100;
        mvprintw(41, 0, "RPM PROGRESS: %.1f%%", rpm_progress);
        int bar_width = 40;
        int filled = (rpm_progress / 100.0) * bar_width;
        mvprintw(42, 0, "[");
        for (int i = 0; i < bar_width; i++) {
            if (i < filled) addch('|');
            else addch(' ');
        }
        printw("]");
        
        // === CSV ЛОГИРОВАНИЕ (каждые 30 кадров) ===
        frame_counter++;
        if (frame_counter >= 30) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
            
            csv_file << std::fixed << std::setprecision(6);
            
            // Время
            csv_file << elapsed.count() / 1000.0 << ",";
            
            // Центр масс
            csv_file << state.position.x << "," << state.position.y << ","
                     << state.speed * 3.6 << ","  // км/ч
                     << state.acceleration.x << "," << state.acceleration.y << ",";
            
            // Углы и управление
            csv_file << state.car_angle * 180.0 / M_PI << ","  // градусы
                     << steering * 180.0 / M_PI << ","         // градусы
                     << state.current_gear << ",";
            
            // Двигатель
            csv_file << state.engine_rpm << "," << state.wheel_rpm << ","
                     << state.traction_force << "," << state.drag_force << "," << state.brake_force << ",";
            
            // Фактические координаты колес
            for (int i = 0; i < 4; i++) {
                csv_file << state.wheel_positions[i].x << "," << state.wheel_positions[i].y;
                if (i < 3) csv_file << ",";
            }
            csv_file << ",";
            
            // Скорости колес
            for (int i = 0; i < 4; i++) {
                csv_file << state.wheel_velocity[i].x << "," << state.wheel_velocity[i].y;
                if (i < 3) csv_file << ",";
            }
            csv_file << ",";
            
            // Идеальные координаты колес
            for (int i = 0; i < 4; i++) {
                csv_file << ideal_positions[i].x << "," << ideal_positions[i].y;
                if (i < 3) csv_file << ",";
            }
            csv_file << ",";
            
            // Ошибки позиционирования
            for (int i = 0; i < 4; i++) {
                csv_file << position_errors[i];
                if (i < 3) csv_file << ",";
            }
            
            csv_file << "\n";
            frame_counter = 0;
        }
        
        // === ОБРАБОТКА ВВОДА С РЕЖИМОМ ПЕРЕКЛЮЧЕНИЯ ===
        int ch = getch();
        
        // Обрабатываем все нажатые клавиши в текущем кадре
        while (ch != ERR) {
            switch (ch) {
                case 'w': // W - газ (переключаемый)
                case 'W':
                    if (!w_was_pressed) { // Защита от двойного срабатывания
                        gas_pressed = !gas_pressed;
                        w_was_pressed = true;
                    }
                    break;
                    
                case 's': // S - тормоз (переключаемый)
                case 'S':
                    if (!s_was_pressed) {
                        brake_pressed = !brake_pressed;
                        s_was_pressed = true;
                    }
                    break;
                    
                case 'a': // A - руль влево (мгновенный)
                case 'A':
                    if (!a_was_pressed) {
                        steering = std::max(steering - 0.1, -1.0);
                        a_was_pressed = true;
                    }
                    break;
                    
                case 'd': // D - руль вправо (мгновенный)
                case 'D':
                    if (!d_was_pressed) {
                        steering = std::min(steering + 0.1, 1.0);
                        d_was_pressed = true;
                    }
                    break;
                    
                case ' ': // Пробел - сброс руля
                    steering = 0.0;
                    break;
                    
                case KEY_LEFT: // Стрелка влево - понижение передачи
                    f1_engine.shiftDown();
                    break;
                    
                case KEY_RIGHT: // Стрелка вправо - повышение передачи
                    f1_engine.shiftUp();
                    break;
                    
                case 'r': // R - сброс
                case 'R':
                    f1_engine.reset();
                    gas_pressed = false;
                    brake_pressed = false;
                    steering = 0.0;
                    break;
                    
                case 27: // ESC - выход
                    running = false;
                    break;
            }
            ch = getch(); // Читаем следующую клавишу
        }
        
        // Сбрасываем флаги нажатия для следующего кадра
        w_was_pressed = false;
        s_was_pressed = false;
        a_was_pressed = false;
        d_was_pressed = false;
        
        refresh();
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }
    
    // === ОЧИСТКА ===
    running = false;
    if (physics_thread.joinable()) {
        physics_thread.join();
    }
    
    csv_file.close();
    endwin();
    std::cout << "F1 Physics simulation stopped." << std::endl;
    std::cout << "Data saved to f1_simulation_log.csv" << std::endl;
    std::cout << "Open in Excel and use 'Format as Table' for better visualization!" << std::endl;
    
    return 0;
}
