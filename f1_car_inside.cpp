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
    const double wheel_radius = 0.330;
    const double brake_factor_coef = 1.0;
    const double brake_rate = 1000.0;
    
    const double dt = 0.01;
    
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
    double traction_force = 0;
    
    double brake_factor = 0;
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
    
    void calculate_brake_factor(bool brake_position) {
        if (brake_position) {
            if (brake_factor + brake_factor_coef*dt <= 1) {
                brake_factor = brake_factor + brake_factor_coef*dt;
            }
        } else {
            if (brake_factor - brake_factor_coef*dt >= 0) {
                brake_factor = brake_factor - brake_factor_coef*dt;
            }
        }
    }
    
    void calculate_wheels() {
        gear_factor = gear_ratios[gear-1] * final_drive_ratio;
        rpm_wheels = rpm / gear_factor;
        torque_wheels = torque * gear_factor;
        traction_force = torque_wheels / wheel_radius;
    }
    
    void calculate_rpm(bool pedal_pos) {
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
    
    void brake() {
        if (brake_factor == 0) { calculate_brake_factor(true); }
        
        if (rpm_wheels - brake_factor * brake_rate * dt >= 0) {
            
            rpm_wheels = rpm_wheels - brake_factor * brake_rate * dt;
            rpm = rpm_wheels * gear_factor;
        
            calculate_torque();
            calculate_wheels();
        }
        
    }
    
    void change_gear(bool up_or_down) {
        
        if (up_or_down) {
            // Повышение передачи
            if (gear < gear_ratios.size()) {
                gear++;
                // Синхронизируем RPM двигателя с новым передаточным числом
                gear_factor = gear_ratios[gear-1] * final_drive_ratio;
                rpm = rpm_wheels * gear_factor;
            } 
        } else {
            // Понижение передачи
            if (gear > 1) {
                
                gear_factor = gear_ratios[gear-2] * final_drive_ratio;
                
                if (rpm_wheels * gear_factor <= max_rpm) {
                       gear--;
                       rpm = rpm_wheels * gear_factor;
                }
                
                
            }
        }
        
        calculate_torque();
        calculate_wheels();
    }
    
    void calculate_params(bool gas_pos, bool brake_pos) {
        
        calculate_rpm(gas_pos);
        calculate_torque();
        calculate_wheels();
        if (brake_factor > 0) {
            calculate_brake_factor(brake_pos);
        }
        if (brake_pos) {
            brake();
        }
        
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
    std::atomic<bool> brake_pressed(false);
    
    // Поток для обновления оборотов
    std::thread engine_thread([&]() {
        while (running) {
            // Используем метод calculate_params для обновления всех параметров
            f1_engine.calculate_params(gas_pressed, brake_pressed);
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
        mvprintw(14, 0, "S - Hold for brake");
        mvprintw(15, 0, "R - Reset RPM");
        mvprintw(16, 0, "LEFT - Shift down");
        mvprintw(17, 0, "RIGHT - Shift up");
        mvprintw(18, 0, "ESC - Exit");
        
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
            case 's': // S - тормоз
            case 'S':
                if (!brake_pressed) {
                    brake_pressed = true;
                }
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
        
        if (ch != 's' && ch != 'S' && brake_pressed) {
            brake_pressed = false;
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