#include "F1_Physics.h"
#include <cmath>
#include <algorithm>

// === КОНСТРУКТОР И СБРОС ===

F1PhysicsEngine::F1PhysicsEngine() {
    reset();
}

void F1PhysicsEngine::reset() {
    current_state = CarState();  // Обнуляем всё состояние
    current_state.current_gear = 1;
    calculateWheelPositions();   // Рассчитываем начальные позиции колес
}

// === ПУБЛИЧНЫЕ МЕТОДЫ ===

void F1PhysicsEngine::update(double dt, bool gas_pedal, bool brake_pedal, double steering) {
    // 1. Двигатель и трансмиссия
    calculateEnginePhysics(gas_pedal, dt);
    
    // 2. Силы
    calculateForces(gas_pedal, brake_pedal, steering);
    
    // 3. Движение
    integrateMotion(dt);
    
    // 4. Геометрия
    calculateWheelPositions();
}

void F1PhysicsEngine::shiftUp() {
    if (current_state.current_gear < 8) {
        current_state.current_gear++;
        // Синхронизируем RPM двигателя с новым передаточным числом
        double gear_factor = params.gear_ratios[current_state.current_gear - 1] * params.final_drive;
        current_state.engine_rpm = current_state.wheel_rpm * gear_factor;
    }
}

void F1PhysicsEngine::shiftDown() {
    if (current_state.current_gear > 1) {
        double new_gear_factor = params.gear_ratios[current_state.current_gear - 2] * params.final_drive;
        
        // Проверяем, не превысит ли понижение передачи максимальные обороты
        if (current_state.wheel_rpm * new_gear_factor <= params.max_rpm) {
            current_state.current_gear--;
            current_state.engine_rpm = current_state.wheel_rpm * new_gear_factor;
        }
    }
}

// === ПРИВАТНЫЕ МЕТОДЫ РАСЧЕТА ===

void F1PhysicsEngine::calculateEnginePhysics(bool gas_pedal, double dt) {
    calculateRPM(gas_pedal, dt);
    calculateTorque();
    calculateWheelParameters();
}

void F1PhysicsEngine::calculateRPM(bool gas_pedal, double dt) {
    if (gas_pedal) {
        current_state.engine_rpm += dt * sigmaFactor();
        if (current_state.engine_rpm > params.max_rpm) {
            current_state.engine_rpm = params.max_rpm;
        }
    } else {
        current_state.engine_rpm -= dt * params.deceleration_rate;
        if (current_state.engine_rpm < 0) {
            current_state.engine_rpm = 0;
        }
    }
}

void F1PhysicsEngine::calculateTorque() {
    if (current_state.engine_rpm < params.null_rpm) {
        current_state.engine_torque = 0;
    }
    else if (current_state.engine_rpm <= params.peak_rpm) {
        current_state.engine_torque = params.max_torque * (current_state.engine_rpm / params.peak_rpm);
    }
    else {
        double drop_factor = 1.0 - 0.4 * (current_state.engine_rpm - params.peak_rpm) / (params.max_rpm - params.peak_rpm);
        current_state.engine_torque = params.max_torque * drop_factor;
    }
}

void F1PhysicsEngine::calculateWheelParameters() {
    double gear_factor = params.gear_ratios[current_state.current_gear - 1] * params.final_drive;
    current_state.wheel_rpm = current_state.engine_rpm / gear_factor;
    current_state.wheel_torque = current_state.engine_torque * gear_factor;
    current_state.traction_force = current_state.wheel_torque / params.wheel_radius;
}

double F1PhysicsEngine::sigmaFactor() const {
    if ((current_state.engine_rpm > 0 && current_state.engine_rpm < params.max_rpm / 3) || 
        (current_state.engine_rpm > params.max_rpm / 3 * 2 && current_state.engine_rpm < params.max_rpm)) {
        return 0.5 * params.acceleration_rate_max;
    }
    else {
        return params.acceleration_rate_max;
    }
}

void F1PhysicsEngine::calculateBrakeFactor(bool brake_pedal, double dt) {
    if (brake_pedal) {
        if (current_state.brake_factor + params.brake_factor_coef * dt <= 1) {
            current_state.brake_factor += params.brake_factor_coef * dt;
        }
    } else {
        if (current_state.brake_factor - params.brake_factor_coef * dt >= 0) {
            current_state.brake_factor -= params.brake_factor_coef * dt;
        }
    }
}

void F1PhysicsEngine::applyBrakes(double dt) {
    if (current_state.brake_factor == 0) { 
        calculateBrakeFactor(true, dt); 
    }
    
    if (current_state.wheel_rpm - current_state.brake_factor * params.brake_rate * dt >= 0) {
        current_state.wheel_rpm -= current_state.brake_factor * params.brake_rate * dt;
        double gear_factor = params.gear_ratios[current_state.current_gear - 1] * params.final_drive;
        current_state.engine_rpm = current_state.wheel_rpm * gear_factor;
        
        calculateTorque();
        calculateWheelParameters();
    }
}

void F1PhysicsEngine::calculateWheelPositions() {
    double half_wheelbase = params.wheelbase / 2.0;
    double half_track = params.track_width / 2.0;
    
    // Пока просто ставим колеса вокруг центра (без учета поворота)
    current_state.wheel_positions[0] = {  // FL (переднее левое)
        current_state.position.x + half_wheelbase,
        current_state.position.y + half_track
    };
    
    current_state.wheel_positions[1] = {  // FR (переднее правое)
        current_state.position.x + half_wheelbase, 
        current_state.position.y - half_track
    };
    
    current_state.wheel_positions[2] = {  // RL (заднее левое)
        current_state.position.x - half_wheelbase,
        current_state.position.y + half_track
    };
    
    current_state.wheel_positions[3] = {  // RR (заднее правое)
        current_state.position.x - half_wheelbase,
        current_state.position.y - half_track
    };
}

void F1PhysicsEngine::calculateForces(bool gas_pedal, bool brake_pedal, double steering) {
    // 1. СИЛА ТЯГИ (только при нажатом газе)
    current_state.traction_force = gas_pedal ? calculateTractionForce() : 0.0;
    
    // 2. СОПРОТИВЛЕНИЕ ВОЗДУХА (всегда против движения)
    current_state.drag_force = calculateDragForce();
    
    // 3. СИЛА ТОРМОЖЕНИЯ (только при нажатом тормозе)
    current_state.brake_force = brake_pedal ? calculateBrakeForce() : 0.0;
    
    // 4. ПРИЖИМНАЯ СИЛА (вертикальная - пока не влияет на 1D движение)
    current_state.down_force = calculateDownForce();
    
    // 5. Управление тормозным фактором и применение тормозов
    calculateBrakeFactor(brake_pedal, 0.01); // dt = 0.01 как в оригинале
    if (brake_pedal) {
        applyBrakes(0.01);
    }
    
    // 💡 ПРИМЕЧАНИЕ: steering пока не используем для 1D движения
}

double F1PhysicsEngine::calculateTractionForce() const {
    double max_traction = params.tire_friction * (params.mass * 9.81 + current_state.down_force);
    
    // Не даем силе тяги превысить силу сцепления
    return std::min(current_state.traction_force, max_traction);
}

double F1PhysicsEngine::calculateDragForce() const {
    // Формула сопротивления воздуха: F_drag = -0.5 * ρ * v² * C_d * A
    double drag_force = -0.5 * params.air_density * 
                       current_state.speed * std::abs(current_state.speed) *
                       params.drag_coefficient * params.frontal_area;
    
    return drag_force;
}

double F1PhysicsEngine::calculateDownForce() const {
    // Формула прижимной силы: F_down = 0.5 * ρ * v² * C_l * A
    double down_force = 0.5 * params.air_density * 
                       current_state.speed * std::abs(current_state.speed) *
                       params.downforce_coefficient * params.frontal_area;
    
    // Прижимная сила всегда отрицательная (вниз)
    return down_force;
}

double F1PhysicsEngine::calculateBrakeForce() const {
    // Используем brake_factor для расчета силы торможения
    return current_state.brake_factor * params.max_brake_force;
}

void F1PhysicsEngine::integrateMotion(double dt) {
    // 1. СУММИРУЕМ ВСЕ СИЛЫ (пока только продольные для 1D)
    double total_force = current_state.traction_force + 
                        current_state.drag_force + 
                        current_state.brake_force;
    
    // 2. ВТОРОЙ ЗАКОН НЬЮТОНА: F = m × a
    current_state.acceleration.x = total_force / params.mass;
    current_state.acceleration.y = 0.0;  // Пока нет бокового движения
    
    // 3. ИНТЕГРИРУЕМ УСКОРЕНИЕ → СКОРОСТЬ (метод Эйлера)
    current_state.velocity.x += current_state.acceleration.x * dt;
    
    // 4. ИНТЕГРИРУЕМ СКОРОСТЬ → ПОЗИЦИЯ
    current_state.position.x += current_state.velocity.x * dt;
    
    // 5. РАСЧЕТ МОДУЛЯ СКОРОСТИ
    current_state.speed = std::sqrt(current_state.velocity.x * current_state.velocity.x + 
                                   current_state.velocity.y * current_state.velocity.y);
    
    // 6. ЗАЩИТА ОТ ОТРИЦАТЕЛЬНОЙ СКОРОСТИ
    if (current_state.velocity.x < 0 && current_state.brake_force == 0) {
        current_state.velocity.x = 0;
        current_state.speed = 0;
    }
}