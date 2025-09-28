#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cstdlib>
#include <vector>
#include <algorithm>

// Функция для проверки нажатия клавиши (неблокирующий ввод)
int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

class SimpleF1Car {
public:
    // ===== ПРОСТЫЕ ПАРАМЕТРЫ =====
    double mass = 740.0;             // Масса [кг]
    double wheel_radius = 0.33;      // Радиус колеса [м]
    double drag_coefficient = 0.9;   // Коэффициент сопротивления
    double frontal_area = 1.5;       // Фронтальная площадь [м²]
    double c_l = -3.2;               // Коэффициент прижимной силы
    
    // ===== ТЕКУЩЕЕ СОСТОЯНИЕ =====
    double position = 0.0;           // Положение [м]
    double velocity = 0.0;           // Скорость [м/с]
    double acceleration = 0.0;       // Ускорение [м/с²]
    
    // ===== ФИКСИРОВАННЫЕ ПАРАМЕТРЫ ДВИГАТЕЛЯ =====
    int current_gear = 1;            // Текущая передача
    double engine_rpm = 8000.0;      // Обороты двигателя [об/мин]
    double engine_torque = 500.0;    // Крутящий момент [Н·м]
    
    // Передаточные числа (1-6 передачи)
    double gear_ratio = 3.2;         // Передаточное число текущей передачи
    double final_drive = 3.5;        // Главная передача
    
    // ===== СИЛЫ =====
    double traction_force = 0.0;     // Сила тяги [Н]
    double drag_force = 0.0;         // Сила сопротивления воздуха [Н]
    double total_force = 0.0;        // Суммарная сила [Н]
    double down_force = 0.0;         // Прижимная сила [Н]
    double brake_force = 0.0;        // Сила торможения [Н]

    // История для графиков
    std::vector<double> time_history;
    std::vector<double> position_history;
    std::vector<double> velocity_history;
    std::vector<double> drag_history;

    // Основной метод обновления физики
    void update(double dt, double throttle, double brake, double simulation_time) {
        // 1. Расчет сил
        calculateForces(throttle, brake);
        
        // 2. Расчет ускорения (F = m*a)
        acceleration = total_force / mass;
        
        // 3. Обновление скорости и позиции
        velocity += acceleration * dt;
        position += velocity * dt;
        
        // 4. Обновление параметров
        updateParameters();
        
        // 5. Сохранение истории для графиков
        time_history.push_back(simulation_time);
        position_history.push_back(position);
        velocity_history.push_back(velocity * 3.6); // в км/ч
        drag_history.push_back(std::abs(drag_force));
    }
    
    void calculateForces(double throttle, double brake) {
        // 1. СИЛА ТЯГИ
        double wheel_torque = engine_torque * gear_ratio * final_drive;
        traction_force = (wheel_torque / wheel_radius) * throttle;
        
        // 2. СИЛА СОПРОТИВЛЕНИЯ ВОЗДУХА
        drag_force = -0.5 * 1.225 * drag_coefficient * frontal_area * 
                    velocity * std::abs(velocity);
        
        // 3. СИЛА ТОРМОЖЕНИЯ
        brake_force = -brake * 10000.0;
        
        // 4. ПРИЖИМНАЯ СИЛА 
        down_force = 0.5 * 1.225 * std::abs(c_l) * frontal_area * velocity * std::abs(velocity);
        
        // 5. СУММАРНАЯ СИЛА
        total_force = traction_force + drag_force + brake_force;
    }
    
    void updateParameters() {
        // Автоматическое переключение передач
        if (velocity * 3.6 > 100.0 && current_gear == 1) {
            current_gear = 2;
            gear_ratio = 2.5;
            engine_rpm = 12000.0;
            engine_torque = 450.0;
        }
        else if (velocity * 3.6 > 150.0 && current_gear == 2) {
            current_gear = 3;
            gear_ratio = 1.9;
            engine_rpm = 14000.0;
            engine_torque = 400.0;
        }
        else if (velocity * 3.6 > 200.0 && current_gear == 3) {
            current_gear = 4;
            gear_ratio = 1.5;
            engine_rpm = 13000.0;
            engine_torque = 380.0;
        }
    }
    
    // Метод для вывода параметров в виде таблицы
    void printStatusTable(double simulation_time) const {
        std::cout << std::fixed << std::setprecision(2);
        
        // Заголовок таблицы
        std::cout << "+------------+------------+------------+------------+------------+------------+------------+\n";
        std::cout << "|   Время    |  Позиция   |  Скорость  | Ускорение  |  Обороты   | Передача   |   Силы     |\n";
        std::cout << "|    (с)     |    (м)     |   (км/ч)   |  (м/с²)    |  (об/мин)  |            |    (Н)     |\n";
        std::cout << "+------------+------------+------------+------------+------------+------------+------------+\n";
        
        // Данные
        std::cout << "| " << std::setw(10) << simulation_time << " | "
                  << std::setw(10) << position << " | "
                  << std::setw(10) << velocity * 3.6 << " | "
                  << std::setw(10) << acceleration << " | "
                  << std::setw(10) << engine_rpm << " | "
                  << std::setw(10) << current_gear << " | "
                  << std::setw(10) << total_force << " |\n";
        
        // Вторая строка с деталями по силам
        std::cout << "+------------+------------+------------+------------+------------+------------+------------+\n";
        std::cout << "| Тяга: " << std::setw(8) << traction_force << " Н"
                  << "| Сопр: " << std::setw(8) << drag_force << " Н"
                  << "| Торм: " << std::setw(8) << brake_force << " Н"
                  << "| Приж: " << std::setw(8) << down_force << " Н |\n";
        std::cout << "+------------+------------+------------+------------+------------+------------+------------+\n";
    }
};

// Функция для построения ASCII графика
void plotGraph(const std::vector<double>& x, const std::vector<double>& y, 
               const std::string& title, const std::string& xlabel, const std::string& ylabel,
               int width = 60, int height = 20) {
    
    if (x.empty() || y.empty()) return;
    
    // Находим min и max значения
    double min_x = *std::min_element(x.begin(), x.end());
    double max_x = *std::max_element(x.begin(), x.end());
    double min_y = *std::min_element(y.begin(), y.end());
    double max_y = *std::max_element(y.begin(), y.end());
    
    // Добавляем немного места сверху
    max_y *= 1.1;
    if (min_y > 0) min_y = 0;
    
    std::cout << "\n" << title << "\n";
    std::cout << std::string(title.length(), '=') << "\n";
    
    // Создаем сетку
    std::vector<std::vector<char>> grid(height, std::vector<char>(width, ' '));
    
    // Рисуем оси
    int zero_y = (int)((0 - min_y) / (max_y - min_y) * (height - 1));
    zero_y = std::min(height - 1, std::max(0, zero_y));
    
    for (int i = 0; i < width; i++) {
        grid[zero_y][i] = '-'; // Ось X
    }
    for (int i = 0; i < height; i++) {
        grid[i][0] = '|'; // Ось Y
    }
    grid[zero_y][0] = '+'; // Начало координат
    
    // Рисуем данные
    for (size_t i = 0; i < x.size(); i++) {
        int plot_x = (int)((x[i] - min_x) / (max_x - min_x) * (width - 1));
        int plot_y = (int)((y[i] - min_y) / (max_y - min_y) * (height - 1));
        
        plot_x = std::min(width - 1, std::max(0, plot_x));
        plot_y = std::min(height - 1, std::max(0, plot_y));
        
        // Инвертируем Y для правильного отображения (0 внизу)
        plot_y = height - 1 - plot_y;
        
        grid[plot_y][plot_x] = '*';
    }
    
    // Выводим сетку
    for (int i = 0; i < height; i++) {
        std::cout << " ";
        for (int j = 0; j < width; j++) {
            std::cout << grid[i][j];
        }
        std::cout << "\n";
    }
    
    // Подписи осей
    std::cout << " " << std::string(width, ' ') << "^\n";
    std::cout << " " << std::string(width, ' ') << "| " << ylabel << " (max: " << std::fixed << std::setprecision(1) << max_y << ")\n";
    std::cout << " +";
    for (int i = 0; i < width - 1; i++) std::cout << "-";
    std::cout << "> " << xlabel << " (0-" << (int)max_x << " сек)\n\n";
}

// Функция для очистки экрана
void clearScreen() {
    std::cout << "\033[2J\033[H";
}

int main() {
    SimpleF1Car car;
    double dt = 0.1; // шаг времени 100 мс
    double simulation_time = 0.0;
    
    // Настройки управления
    double throttle = 0.0;
    double brake = 0.0;
    char key = ' ';
    
    std::cout << "=== ПРОСТАЯ МОДЕЛЬ F1 CAR (Режим реального времени) ===" << std::endl;
    std::cout << "Нажмите любую клавишу для начала..." << std::endl;
    std::cin.get();
    
    // Основной цикл симуляции (30 секунд)
    while (simulation_time <= 30.0) {
        // Очищаем экран и выводим обновленную таблицу
        clearScreen();
        std::cout << "=== ПРОСТАЯ МОДЕЛЬ F1 CAR ===" << std::endl;
        std::cout << "Симуляция: " << std::fixed << std::setprecision(1) << simulation_time << " / 30.0 сек" << std::endl;
        car.printStatusTable(simulation_time);
        
        // Инструкция
        std::cout << "Управление: [1]Газ [2]Тормоз [3]Нейтраль [Q]Выход\n";
        
        // Проверяем нажатие клавиши
        if (kbhit()) {
            key = getchar();
            
            switch (key) {
                case '1': // Газ
                    throttle = 1.0;
                    brake = 0.0;
                    break;
                case '2': // Тормоз
                    throttle = 0.0;
                    brake = 1.0;
                    break;
                case '3': // Нейтраль
                    throttle = 0.0;
                    brake = 0.0;
                    break;
                case 'q':
                case 'Q':
                    std::cout << "Досрочный выход из программы..." << std::endl;
                    break;
                default:
                    break;
            }
        }
        
        // Обновляем физику
        car.update(dt, throttle, brake, simulation_time);
        simulation_time += dt;
        
        // Задержка для визуализации (100 мс)
        usleep(100000);
    }
    
    // После завершения симуляции рисуем графики
    clearScreen();
    std::cout << "=== РЕЗУЛЬТАТЫ СИМУЛЯЦИИ (30 секунд) ===" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // График позиции
    plotGraph(car.time_history, car.position_history,
              "ПОЗИЦИЯ АВТОМОБИЛЯ", "Время (с)", "Позиция (м)");
    
    // График скорости
    plotGraph(car.time_history, car.velocity_history,
              "СКОРОСТЬ АВТОМОБИЛЯ", "Время (с)", "Скорость (км/ч)");
    
    // График сопротивления воздуха
    plotGraph(car.time_history, car.drag_history,
              "СОПРОТИВЛЕНИЕ ВОЗДУХА", "Время (с)", "Сила (Н)");
    
    std::cout << "Нажмите любую клавишу для выхода...";
    std::cin.ignore();
    std::cin.get();
    
    return 0;
}