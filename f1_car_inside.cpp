#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <ncurses.h>
#include <vector>

struct f1_car_inside {
private:
    const double max_rpm = 15000.0;
    const double deceleration_rate = 500.0;
    const double acceleration_rate_max = 3000.0;
    const double time_to_max_rpm = 5.0;
    const double max_torque = 500.0;
    const double peak_rpm = 11000.0;
    const double null_rpm = 4000.0;
    const double back_wheel_radius = 0.371;
    const double front_wheel_radius = 0.330;
    const std::vector<double> gear_ratios = {
        3.0,   // 1-я
        2.4,   // 2-я
        2.0,   // 3-я  
        1.7,   // 4-я
        1.4,   // 5-я
        1.2,   // 6-я
        1.1,   // 7-я
        1.0    // 8-я
    };
    
    const double final_drive_ratio = 3.2;
    
    unsigned int gear = 1;
    double rpm = 0;
    double torque = 0;
    double rpm_wheels = 0;
    double torque_wheels = 0;
    
    //Сила тяги на передних и задних колесах
    double back_traction_force = 0;
    double front_traction_force = 0;
    
    double gear_factor = gear_ratios[gear-1] * final_drive_ratio;

public:
    
    double sigma_factor() {
        if ((rpm > 0) && (rpm < max_rpm/3) || (rpm > max_rpm/3*2) && (rpm < max_rpm)) {
            return 0.5 * acceleration_rate_max;
        }
        else {
            return acceleration_rate_max;
        }
    }
    
    void calculate_wheels() {
        gear_factor = gear_ratios[gear-1] * final_drive_ratio;
        rpm_wheels = rpm / gear_factor;
        torque_wheels = torque * gear_factor;
        back_traction_force = torque_wheels / back_wheel_radius;
        front_traction_force = torque_wheels / front_wheel_radius;
    }
    
    void calculate_rpm(bool pedal_pos, double dt) {
        if (pedal_pos) {
            rpm = rpm + dt * sigma_factor();
            if (rpm > max_rpm) {
                rpm = max_rpm;
            }
        } else {
            rpm = rpm - dt * deceleration_rate;
            if (rpm < 0) {
                rpm = 0;
            }
        }
    }
    
    void calculate_torque() {
        if (rpm < null_rpm) {
            torque = 0;
        }
        else if (rpm <= peak_rpm) {
            torque = max_torque * (rpm / peak_rpm);
        }
        else {
            double drop_factor = 1.0 - 0.4 * (rpm - peak_rpm) / (max_rpm - peak_rpm);
            torque = max_torque * drop_factor;
        }
    }
    
    void calculate_params(bool ped_pos) {
        // Дифференциальное время
        const double diff_t = 0.01;
        
        calculate_rpm(ped_pos, diff_t);
        calculate_torque();
        calculate_wheels();
    }
    
    void change_gear(bool up_or_down) {
        // Сохраняем текущие обороты колес для синхронизации
        double current_wheel_rpm = rpm_wheels;
        
        if (up_or_down) {
            // Повышение передачи
            if (gear < gear_ratios.size()) {
                gear++;
                // Синхронизируем RPM двигателя с новым передаточным числом
                gear_factor = gear_ratios[gear-1] * final_drive_ratio;
                rpm = current_wheel_rpm * gear_factor;
            } 
        } else {
            // Понижение передачи
            if (gear > 1) {
                gear--;
                // Синхронизируем RPM двигателя с новым передаточным числом
                gear_factor = gear_ratios[gear-1] * final_drive_ratio;
                rpm = current_wheel_rpm * gear_factor;
                
                if (rpm > max_rpm) {
                    rpm = max_rpm;
                }
            }
        }
        
        // Пересчитываем параметры после переключения
        calculate_torque();
        calculate_wheels();
    }
    
    // Геттеры
    double get_rpm() const { return rpm; }
    double get_torque() const { return torque; }
    double get_max_rpm() const { return max_rpm; }
    unsigned int get_gear() const { return gear; }
    double get_wheel_rpm() const { return rpm_wheels; }
    double get_wheel_torque() const { return torque_wheels; }
    double get_gear_ratio() const { return gear_ratios[gear-1]; }
    double get_final_drive() const { return final_drive_ratio; }
    double get_total_ratio() const { return gear_factor; }
    
    void set_rpm(double new_rpm) { rpm = new_rpm; }
};

int main() {
    // Инициализация ncurses
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    
    f1_car_inside f1_engine;
    
    std::atomic<bool> running(true);
    std::atomic<bool> gas_pressed(false);
    
    // Поток для обновления оборотов
    std::thread engine_thread([&]() {
        while (running) {
            // Используем метод calculate_params для обновления всех параметров
            f1_engine.calculate_params(gas_pressed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // dt = 0.01 секунды
        }
    });
    
    // Основной цикл обработки ввода
    while (running) {
        clear();
        
        // Выводим информацию
        mvprintw(0, 0, "Formula 1 Engine RPM Simulator");
        mvprintw(1, 0, "==============================");
        mvprintw(2, 0, "Current Gear: %d", f1_engine.get_gear());
        mvprintw(3, 0, "Engine RPM: %.0f", f1_engine.get_rpm());
        mvprintw(4, 0, "Engine Torque: %.1f Nm", f1_engine.get_torque());
        mvprintw(5, 0, "Wheel RPM: %.1f", f1_engine.get_wheel_rpm());
        mvprintw(6, 0, "Wheel Torque: %.1f Nm", f1_engine.get_wheel_torque());
        mvprintw(7, 0, "Gear Ratio: %.1f", f1_engine.get_gear_ratio());
        mvprintw(8, 0, "Total Ratio: %.1f", f1_engine.get_total_ratio());
        mvprintw(9, 0, "Gas pedal: %s", gas_pressed ? "PRESSED (W)" : "RELEASED");
        mvprintw(10, 0, "Progress: %.1f%%", (f1_engine.get_rpm() / f1_engine.get_max_rpm()) * 100);
        
        mvprintw(12, 0, "Controls:");
        mvprintw(13, 0, "W - Hold for gas");
        mvprintw(14, 0, "R - Reset RPM");
        mvprintw(15, 0, "LEFT - Shift down");
        mvprintw(16, 0, "RIGHT - Shift up");
        mvprintw(17, 0, "ESC - Exit");
        
        // Обработка ввода
        int ch = getch();
        
        switch (ch) {
            case 'w': // W - педаль газа
            case 'W':
                if (!gas_pressed) {
                    gas_pressed = true;
                }
                break;
            case 'r': // R - сбросить RPM
            case 'R':
                f1_engine.set_rpm(0.0);
                gas_pressed = false;
                break;
            case KEY_LEFT: // Стрелка влево - понижение передачи
                f1_engine.change_gear(false); // false = downshift
                break;
            case KEY_RIGHT: // Стрелка вправо - повышение передачи
                f1_engine.change_gear(true); // true = upshift
                break;
            case 27: // ESC - выход
                running = false;
                break;
        }
        
        // Если W отпущена
        if (ch != 'w' && ch != 'W' && gas_pressed) {
            gas_pressed = false;
        }
        
        refresh();
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    
    // Очистка
    running = false;
    if (engine_thread.joinable()) {
        engine_thread.join();
    }
    
    endwin();
    std::cout << "F1 Engine simulation stopped." << std::endl;
    
    return 0;
}