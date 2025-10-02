#ifndef F1_PHYSICS_H
#define F1_PHYSICS_H

#include <vector>
#include <array>

class F1PhysicsEngine {
public:
    // Структура для точки в 2D пространстве
    struct Point2D {
        double x, y;
        Point2D(double x = 0, double y = 0) : x(x), y(y) {}
    };
    
    // Структура для вектора в 2D пространстве
    struct Vector2D {
        double x, y;
        Vector2D(double x = 0, double y = 0) : x(x), y(y) {}
    };
    
    // Основная структура состояния автомобиля
    struct CarState {
        // Поступательное движение
        Point2D position;           // Позиция центра масс [м]
        Vector2D velocity;          // Скорость центра масс [м/с]
        Vector2D acceleration;      // Ускорение центра масс [м/с²]
        double speed;               // Модуль скорости [м/с]
        
        // Вращательное движение  
        double angle = 0.0;         // Угол поворота автомобиля [рад]
        double angular_velocity = 0.0;  // Угловая скорость [рад/с]
        
        // Колеса (0=FL, 1=FR, 2=RL, 3=RR)
        std::array<Point2D, 4> wheel_positions;    // Позиции колес
        
        // Двигатель и трансмиссия
        double engine_rpm = 0.0;
        double engine_torque = 0.0;
        double wheel_rpm = 0.0;
        double wheel_torque = 0.0;
        int current_gear = 1;
        double traction_force = 0.0;
        
        // Силы
        double traction_force = 0.0;
        double drag_force = 0.0;
        double brake_force = 0.0;
        double down_force = 0.0;
        
        // Тормозная система
        double brake_factor = 0.0;
    };

private:
    // Текущее состояние (меняется каждый кадр)
    CarState current_state;
    
    // Параметры автомобиля (константы, не меняются)
    struct CarParameters {
        // === ГЕОМЕТРИЯ ===
        double wheelbase = 3.7;         // Колесная база [м]
        double track_width = 1.8;       // Колея [м]
        double wheel_radius = 0.33;     // Радиус колеса [м]
        double mass = 740.0;            // Масса [кг]
        double moment_of_inertia = 1000.0; // Момент инерции [кг·м²]
        
        // === ДВИГАТЕЛЬ И ТРАНСМИССИЯ ===
        double max_rpm = 15000.0;       // Максимальные обороты двигателя
        double max_torque = 500.0;      // Максимальный крутящий момент [Н·м]
        double peak_rpm = 11000.0;      // Обороты максимального момента
        double null_rpm = 4000.0;       // Обороты холостого хода
        double deceleration_rate = 500.0; // Скорость снижения RPM
        double acceleration_rate_max = 3000.0; // Макс. скорость ускорения RPM
        double time_to_max_rpm = 5.0;   // Время до макс. RPM
        std::array<double, 8> gear_ratios = {3.2, 2.5, 2.0, 1.7, 1.4, 1.2, 1.1, 1.0}; // КПП
        double final_drive = 3.5;       // Главная передача
        
        // === АЭРОДИНАМИКА ===
        double drag_coefficient = 0.9;  // Коэффициент лобового сопротивления
        double frontal_area = 1.5;      // Фронтальная площадь [м²]
        double air_density = 1.225;     // Плотность воздуха [кг/м³]
        const double downforce_coefficient = -3.0; // Коэффициент прижимной силы 
        
        // === ШИНЫ И ТОРМОЗА ===
        double tire_friction = 1.5;     // Коэффициент трения шин
        double max_brake_force = 15000.0; // Максимальная сила торможения [Н]
        double brake_factor_coef = 1.0; // Коэффициент торможения
        double brake_rate = 1000.0;     // Скорость торможения
    };
    
    CarParameters params;

public:
    // Конструктор
    F1PhysicsEngine();
    
    // Сброс состояния
    void reset();
    
    // === ПУБЛИЧНЫЙ ИНТЕРФЕЙС ===
    
    // Основной метод обновления физики
    void update(double dt, bool gas_pedal, bool brake_pedal, double steering = 0.0);
    
    // Управление передачами
    void shiftUp();
    void shiftDown();
    
    // === ГЕТТЕРЫ для отрисовки ===
    const CarState& getState() const { return current_state; }

private:
    // === ПРИВАТНЫЕ МЕТОДЫ РАСЧЕТА ===
    
    // Двигатель и трансмиссия
    void calculateEnginePhysics(bool gas_pedal, double dt);
    void calculateRPM(bool gas_pedal, double dt);
    void calculateTorque();
    void calculateWheelParameters();
    void calculateBrakeFactor(bool brake_pedal, double dt);
    void applyBrakes(double dt);
    double sigmaFactor() const;
    
    // Силы
    void calculateForces(bool gas_pedal, bool brake_pedal, double steering);
    double calculateTractionForce() const;
    double calculateDragForce() const;
    double calculateDownForce() const;
    double calculateBrakeForce() const;
    
    // Движение
    void integrateMotion(double dt);
    
    // Геометрия
    void calculateWheelPositions();
};

#endif // F1_PHYSICS_H