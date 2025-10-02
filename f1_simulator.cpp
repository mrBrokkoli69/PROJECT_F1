#include "F1_Physics_build_2.h"
#include <ncurses.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>

int main() {
    // Инициализация ncurses
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    
    // Создаем физический движок F1
    F1PhysicsEngine f1_engine;
    
    std::atomic<bool> running(true);
    std::atomic<bool> gas_pressed(false);
    std::atomic<bool> brake_pressed(false);
    double steering = 0.0;
    
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
        
        // Выводим информацию
        mvprintw(0, 0, "=== FORMULA 1 PHYSICS SIMULATION ===");
        mvprintw(1, 0, "=====================================");
        
        // Двигатель и трансмиссия
        mvprintw(2, 0, "ENGINE AND TRANSMISSION:");
        mvprintw(3, 2, "Current Gear: %d", state.current_gear);
        mvprintw(4, 2, "Engine RPM: %.0f", state.engine_rpm);
        mvprintw(5, 2, "Engine Torque: %.1f Nm", state.engine_torque);
        mvprintw(6, 2, "Wheel RPM: %.1f", state.wheel_rpm);
        mvprintw(7, 2, "Wheel Torque: %.1f Nm", state.wheel_torque);
        
        // Скорость и движение
        mvprintw(9, 0, "SPEED AND MOTION:");
        mvprintw(10, 2, "Speed: %.1f km/h", state.speed * 3.6);
        mvprintw(11, 2, "Position X: %.1f m", state.position.x);
        mvprintw(12, 2, "Acceleration: %.1f m/s²", state.acceleration.x);
        
        // Силы
        mvprintw(14, 0, "FORCES:");
        mvprintw(15, 2, "Traction Force: %.1f N", state.traction_force);
        mvprintw(16, 2, "Drag Force: %.1f N", state.drag_force);
        mvprintw(17, 2, "Brake Force: %.1f N", state.brake_force);
        mvprintw(18, 2, "Down Force: %.1f N", state.down_force);
        mvprintw(19, 2, "Brake Factor: %.2f", state.brake_factor);
        
        // Координаты колес
        mvprintw(21, 0, "WHEEL POSITIONS:");
        mvprintw(22, 2, "FL: (%.1f, %.1f)", state.wheel_positions[0].x, state.wheel_positions[0].y);
        mvprintw(23, 2, "FR: (%.1f, %.1f)", state.wheel_positions[1].x, state.wheel_positions[1].y);
        mvprintw(24, 2, "RL: (%.1f, %.1f)", state.wheel_positions[2].x, state.wheel_positions[2].y);
        mvprintw(25, 2, "RR: (%.1f, %.1f)", state.wheel_positions[3].x, state.wheel_positions[3].y);
        
        // Управление
        mvprintw(27, 0, "CONTROLS:");
        mvprintw(28, 2, "W - Gas: %s", gas_pressed ? "PRESSED" : "RELEASED");
        mvprintw(29, 2, "S - Brake: %s", brake_pressed ? "PRESSED" : "RELEASED");
        mvprintw(30, 2, "LEFT Arrow - Shift down");
        mvprintw(31, 2, "RIGHT Arrow - Shift up");
        mvprintw(32, 2, "R - Reset");
        mvprintw(33, 2, "ESC - Exit");
        
        // Прогресс оборотов
        double rpm_progress = (state.engine_rpm / 15000.0) * 100;
        mvprintw(35, 0, "RPM PROGRESS: %.1f%%", rpm_progress);
        
        // Простая текстовая шкала прогресса
        int bar_width = 40;
        int filled = (rpm_progress / 100.0) * bar_width;
        mvprintw(36, 0, "[");
        for (int i = 0; i < bar_width; i++) {
            if (i < filled) {
                addch('|');
            } else {
                addch(' ');
            }
        }
        printw("]");
        
        // Обработка ввода
        int ch = getch();
        
        switch (ch) {
            case 'w': // W - педаль газа
            case 'W':
                gas_pressed = true;
                break;
                
            case 's': // S - тормоз
            case 'S':
                brake_pressed = true;
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
                break;
                
            case 27: // ESC - выход
                running = false;
                break;
        }
        
        // Если клавиши отпущены
        if (ch != 'w' && ch != 'W') {
            gas_pressed = false;
        }
        
        if (ch != 's' && ch != 'S') {
            brake_pressed = false;
        }
        
        refresh();
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
    
    // Очистка
    running = false;
    if (physics_thread.joinable()) {
        physics_thread.join();
    }
    
    endwin();
    std::cout << "F1 Physics simulation stopped." << std::endl;
    
    return 0;
}
