#ifndef F1_PHYSICS_H
#define F1_PHYSICS_H

#include <vector>
#include <array>
#include <cmath>

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
        double car_angle = 0.0;         // Угол поворота автомобиля [рад]
        double angular_velocity = 0.0;  // Угловая скорость [рад/с]
        double steering_wheel = 0.0;    // Угол поворота руля
        double rotation_radius = 0.0;   // радиус кривизны поворота
        
        // Нагрузки
        std::array<double, 4> wheel_loads; // нагрузка распределенная по колесам
        
        // Колеса (0=FL, 1=FR, 2=RL, 3=RR)
        std::array<Point2D, 4> wheel_positions;    // Позиции колес
        std::array<Vector2D, 4> wheel_velocity;    // Скорости колес
        std::array<Vector2D, 4> wheel_acceleration; // Ускорения колес
        std::array<double, 4> wheel_slip_angle;    // угол скольжения каждого колеса

        // Силы на колеса:
        std::array<Vector2D, 4> wheel_forces;      // x - продольная, y - поперечная
        
        // Двигатель и трансмиссия
        double engine_rpm = 4000.0;
        double engine_torque = 0.0;
        double wheel_rpm = 0.0;
        double wheel_torque = 0.0;
        int current_gear = 0;
        double wheel_based_speed = 0.0;    // Скорость из оборотов колес [м/с]

        // Силы
        double traction_force = 0.0;
        double drag_force = 0.0;
        double brake_force = 0.0;
        double down_force = 0.0;
        std::array<double, 4> side_force;          // Боковые силы
        std::array<double, 4> centr_force;         // Центробежные силы

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
        double rotor_inertia = 1000.0;
        double rotor_resist = 1000;
        double max_torque = 900.0;      // Максимальный крутящий момент [Н·м]
        double peak_rpm = 14000.0;      // Обороты максимального момента
        double null_rpm = 4000.0;       // Обороты холостого хода
        double acceleration_rate_max = 3000.0; // Макс. скорость ускорения RPM
        double time_to_max_rpm = 5.0;   // Время до макс. RPM
        std::array<double, 9> gear_ratios = {0.0, 3.2, 2.5, 2.0, 1.7, 1.4, 1.2, 1.1, 1.0}; // КПП
        double final_drive = 3.5;       // Главная передача

        // === АЭРОДИНАМИКА ===
        double drag_coefficient = 0.6;  // Коэффициент лобового сопротивления
        double frontal_area = 1.7;      // Фронтальная площадь [м²]
        double air_density = 1.1;     // Плотность воздуха [кг/м³]
        double downforce_coefficient = 3.0; // Коэффициент прижимной силы

        // === ШИНЫ И ТОРМОЗА ===
        double tire_friction = 1.5;     // Коэффициент трения шин
        double max_brake_force = 10000.0; // Максимальная сила торможения [Н]
        double brake_factor_coef = 1.0; // Коэффициент торможения
        double brake_rate = 1000.0;     // Скорость торможения

        // === ПОВОРОТ И ШИНЫ ===
        double k_front = 80000.0;       // Жесткость передних шин [Н/рад]
        double k_rear = 60000.0;        // Жесткость задних шин [Н/рад]
        double mu_front = 1.6;          // Трение передних шин
        double mu_rear = 1.4;           // Трение задних шин
        double stiffness = 0.2;         // Коэффициент упругости подвески
        double max_steering_angle = 0.5; // Макс угол руля [рад]
    };

    CarParameters params;

public:
    // Конструктор
    F1PhysicsEngine();

    // Сброс состояния
    void reset();

    // === ПУБЛИЧНЫЙ ИНТЕРФЕЙС ===

    // Основной метод обновления физики
    void update(double dt, bool gas_pedal, bool brake_pedal, double steering_wheel = 0.0);
    Point2D calculateIdealWheelPosition(int wheel_index);

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
    void TestMaxRPMForCurrentGear();
    
    // Силы центра масс
    void calculateForces(bool gas_pedal, bool brake_pedal, double steering);
    double calculateTractionForce() const;
    double calculateDragForce() const;
    double calculateDownForce() const;
    double calculateBrakeForce() const;
    
    // Силы поворота
    void calculateRotationForces(double dt);
    void calculateWheelLoads();
    void calculateAngularVelocity();
    void calculateWheelSlipAngle(double dt);
    void calculateRotationRadius();
    void calculateSideForce();
    void calculateCentrForce();
    void calculateWheelForces();
    
    // Движение и геометрия
    void integrateMotion(double dt);
    void checkRealPosition();
    
};

#endif // F1_PHYSICS_H