#include "F1_Physics_build_2.h"
#include <cmath>
#include <algorithm>

// === КОНСТРУКТОР И СБРОС ===

F1PhysicsEngine::F1PhysicsEngine() {
    reset();
}

void F1PhysicsEngine::reset() {
    current_state = CarState();
    current_state.current_gear = 1;
    for (int i = 0; i < 4; i++) {
        current_state.wheel_positions[i] = calculateIdealWheelPosition(i);
    }
}

// === ПУБЛИЧНЫЕ МЕТОДЫ ===

void F1PhysicsEngine::update(double dt, bool gas_pedal, bool brake_pedal, double steering) {
    current_state.steering_wheel = steering;
    
    calculateEnginePhysics(gas_pedal, dt);
    calculateForces(gas_pedal, brake_pedal, steering);
    calculateRotationForces(dt);
    integrateMotion(dt);
}

void F1PhysicsEngine::shiftUp() {
    if (current_state.current_gear < 8) {
        current_state.current_gear++;
        double gear_factor = params.gear_ratios[current_state.current_gear - 1] * params.final_drive;
        current_state.engine_rpm = current_state.wheel_rpm * gear_factor;
    }
}

void F1PhysicsEngine::shiftDown() {
    if (current_state.current_gear > 1) {
        double new_gear_factor = params.gear_ratios[current_state.current_gear - 2] * params.final_drive;
        if (current_state.wheel_rpm * new_gear_factor <= params.max_rpm) {
            current_state.current_gear--;
            current_state.engine_rpm = current_state.wheel_rpm * new_gear_factor;
        }
    }
}

// === ДВИГАТЕЛЬ И ТРАНСМИССИЯ ===

void F1PhysicsEngine::calculateEnginePhysics(bool gas_pedal, double dt) {
    calculateRPM(gas_pedal, dt);
    TestMaxRPMForCurrentGear();
    calculateTorque();
    calculateWheelParameters();
}

void F1PhysicsEngine::calculateRPM(bool gas_pedal, double dt) {
    if (gas_pedal) {
        current_state.engine_rpm += dt * sigmaFactor();
    } else {
        current_state.engine_rpm -= dt * params.deceleration_rate;
        if (current_state.engine_rpm < 0) {
            current_state.engine_rpm = 0;
        }
    }
    
    // ЖЕСТКОЕ ОГРАНИЧЕНИЕ ОБОРОТОВ ДВИГАТЕЛЯ
    if (current_state.engine_rpm > params.max_rpm) {
        current_state.engine_rpm = params.max_rpm;
    }
}



void F1PhysicsEngine::TestMaxRPMForCurrentGear() { //проверка максимальных оборотов для дальнейше связи с TractionForce
    double gear_ratio = params.gear_ratios[current_state.current_gear - 1];
    double gear_factor = gear_ratio * params.final_drive;
    
    // ВЫЧИСЛЯЕМ RPM КОЛЕС ИЗ RPM ДВИГАТЕЛЯ
    current_state.wheel_rpm = current_state.engine_rpm / gear_factor;
    
    // ВЫЧИСЛЯЕМ МАКСИМАЛЬНЫЕ RPM КОЛЕС ДЛЯ ТЕКУЩЕЙ ПЕРЕДАЧИ
    double max_wheel_rpm = params.max_rpm / gear_factor;

        // ОГРАНИЧИВАЕМ RPM КОЛЕС
    if (current_state.wheel_rpm > max_wheel_rpm) {
        current_state.wheel_rpm = max_wheel_rpm;
        // ОБРАТНАЯ СВЯЗЬ: ЕСЛИ КОЛЕСА ОГРАНИЧЕНЫ - ОГРАНИЧИВАЕМ И ДВИГАТЕЛЬ
        current_state.engine_rpm = current_state.wheel_rpm * gear_factor;
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
    double gear_ratio = params.gear_ratios[current_state.current_gear - 1];
    double gear_factor = gear_ratio * params.final_drive;

    // СКОРОСТЬ ИЗ RPM КОЛЕС (для отображения)
    current_state.wheel_based_speed = current_state.wheel_rpm * (2 * M_PI / 60.0) * params.wheel_radius;
    
    // СИЛЫ И МОМЕНТЫ
    current_state.wheel_torque = current_state.engine_torque * gear_factor;
    current_state.traction_force = current_state.wheel_torque / params.wheel_radius;
}

double F1PhysicsEngine::sigmaFactor() const {
    if ((current_state.engine_rpm > 0 && current_state.engine_rpm < params.max_rpm / 3) || 
        (current_state.engine_rpm > params.max_rpm / 3 * 2 && current_state.engine_rpm < params.max_rpm)) {
        return  params.acceleration_rate_max;
    }
    else {
        return params.acceleration_rate_max;
    }
}

// === СИЛЫ ЦЕНТРА МАСС ===

double F1PhysicsEngine::calculateTractionForce() const {
    double max_traction = params.tire_friction * (params.mass * 9.81 + current_state.down_force);

    return std::min(current_state.traction_force, max_traction);
}

double F1PhysicsEngine::calculateDragForce() const {
    return -0.5 * params.air_density * 
           current_state.velocity.x * current_state.speed *
           params.drag_coefficient * params.frontal_area;
}

double F1PhysicsEngine::calculateDownForce() const {
    return 0.5 * params.air_density * 
           current_state.velocity.x * current_state.speed *
           std::abs(params.downforce_coefficient) * params.frontal_area;
}

double F1PhysicsEngine::calculateBrakeForce() const {
    if (current_state.velocity.x <= 0) {
        return 0;
    } else {
        return -current_state.brake_factor * params.max_brake_force;
    }
}

void F1PhysicsEngine::calculateForces(bool gas_pedal, bool brake_pedal, double steering) {
    current_state.steering_wheel = steering;


    
    current_state.traction_force = calculateTractionForce();

    double gear_ratio = params.gear_ratios[current_state.current_gear - 1];
    double gear_factor = gear_ratio * params.final_drive;

    double max_wheel_rpm = params.max_rpm / gear_factor;
    if (current_state.wheel_rpm >= max_wheel_rpm) {
        if (gas_pedal) {
            current_state.traction_force = -1 * current_state.drag_force;
        }
    }


    

    current_state.drag_force = calculateDragForce();
    current_state.brake_force = brake_pedal ? calculateBrakeForce() : 0.0;
    current_state.down_force = calculateDownForce();
    
    calculateBrakeFactor(brake_pedal, 0.01);
    if (brake_pedal) {
        applyBrakes(0.01);
    }
}

void F1PhysicsEngine::calculateBrakeFactor(bool brake_pedal, double dt) {
    if (brake_pedal) {
        current_state.brake_factor = std::min(current_state.brake_factor + params.brake_factor_coef * dt, 1.0);
    } else {
        current_state.brake_factor = std::max(current_state.brake_factor - params.brake_factor_coef * dt, 0.0);
    }
}

void F1PhysicsEngine::applyBrakes(double dt) {
    if (current_state.wheel_rpm - current_state.brake_factor * params.brake_rate * dt >= 0) {
        current_state.wheel_rpm -= current_state.brake_factor * params.brake_rate * dt;
        double gear_factor = params.gear_ratios[current_state.current_gear - 1] * params.final_drive;
        current_state.engine_rpm = current_state.wheel_rpm * gear_factor;
    }
}

// === СИЛЫ ПОВОРОТА ===

void F1PhysicsEngine::calculateRotationForces(double dt) {
    calculateWheelLoads();
    calculateAngularVelocity();
    calculateWheelSlipAngle(dt);
    calculateSideForce();
    calculateWheelForces();
}

void F1PhysicsEngine::calculateWheelLoads() {
    double base_load = params.mass * 9.81 / 4;
    for (int i = 0; i < 4; i++) {
        current_state.wheel_loads[i] = base_load;
    }
}

void F1PhysicsEngine::calculateAngularVelocity() {
    if(current_state.speed >= 10){
         double speed_factor = 1.0 / (1.0 + current_state.speed / 30.0);
         current_state.angular_velocity = current_state.steering_wheel * 2.0 * speed_factor;
    }

}

void F1PhysicsEngine::calculateWheelSlipAngle(double dt) {
    current_state.car_angle += current_state.angular_velocity * dt;
    
    if (current_state.car_angle > 2 * M_PI) current_state.car_angle -= 2 * M_PI;
    if (current_state.car_angle < 0) current_state.car_angle += 2 * M_PI;
    
    for (int i = 0; i < 4; i++) {
        current_state.wheel_slip_angle[i] = 0.0;
    }
}

void F1PhysicsEngine::calculateSideForce() {
    if (current_state.speed < 0.1 || std::abs(current_state.steering_wheel) < 0.05) {
        for (int i = 0; i < 4; i++) {
            current_state.side_force[i] = 0.0;
        }
        return;
    }
    
    double base_force = current_state.steering_wheel * current_state.speed * 150.0;
    
    for (int i = 0; i < 4; i++) {
        current_state.side_force[i] = base_force;
    }
}

void F1PhysicsEngine::calculateWheelForces() {
    for (int i = 0; i < 4; i++) {
        double longitudinal = 0.0;
        double lateral = current_state.side_force[i];
        
        if (i >= 2) {
            longitudinal += current_state.traction_force * 0.5;
        }
        
        longitudinal += current_state.brake_force * 0.25;
        longitudinal += current_state.drag_force * 0.25;
        
        current_state.wheel_forces[i] = Vector2D(longitudinal, lateral);
    }
}

// === ДВИЖЕНИЕ И ГЕОМЕТРИЯ ===

void F1PhysicsEngine::integrateMotion(double dt) {
    // 1. Центр масс - основное движение
    Vector2D total_force = {0.0, 0.0};
    for (int i = 0; i < 4; i++) {
        total_force.x += current_state.wheel_forces[i].x;
        total_force.y += current_state.wheel_forces[i].y;
    }
    
    current_state.acceleration.x = total_force.x / params.mass;
    current_state.acceleration.y = total_force.y / params.mass;
    
    current_state.velocity.x += current_state.acceleration.x * dt;
    current_state.velocity.y += current_state.acceleration.y * dt;
    
    // 2. ОБНОВЛЯЕМ СКОРОСТЬ И ПОЗИЦИЮ
    double current_speed = std::sqrt(current_state.velocity.x * current_state.velocity.x
                                    );
    
    current_state.position.x += current_state.velocity.x * dt;
    current_state.position.y += current_state.velocity.y * dt;
    current_state.speed = current_speed;
    
    // 3. ОБНОВЛЯЕМ RPM КОЛЕС И ДВИГАТЕЛЯ НА ОСНОВЕ РЕАЛЬНОЙ СКОРОСТИ
     double gear_ratio = params.gear_ratios[current_state.current_gear - 1];
     double gear_factor = gear_ratio * params.final_drive;

     // Вычисляем ЦЕЛЕВЫЕ RPM из реальной скорости
     double target_wheel_rpm = current_speed / ((2 * M_PI / 60.0) * params.wheel_radius);

     // Плавная коррекция RPM колес к реальной скорости
     double rpm_correction_factor = 5.0; // увеличил для лучшей связи
    

    // // ОБРАТНАЯ СВЯЗЬ: обновляем RPM двигателя из RPM колес
     current_state.engine_rpm = current_state.wheel_rpm * gear_factor;

    // // 4. ПРИМЕНЯЕМ ОГРАНИЧЕНИЯ RPM
     double max_wheel_rpm = params.max_rpm / gear_factor;
     if (current_state.wheel_rpm > max_wheel_rpm) {
         current_state.wheel_rpm = max_wheel_rpm;
         current_state.engine_rpm = current_state.wheel_rpm * gear_factor;
     }
    
    // 5. Обновление скоростей и позиций колес
    for (int i = 0; i < 4; i++) {
        Point2D wheel_rel_pos = {
            current_state.wheel_positions[i].x - current_state.position.x,
            current_state.wheel_positions[i].y - current_state.position.y
        };
        
        double rot_vx = -current_state.angular_velocity * wheel_rel_pos.y;
        double rot_vy = current_state.angular_velocity * wheel_rel_pos.x;
        
        current_state.wheel_velocity[i].x = current_state.velocity.x + rot_vx;
        current_state.wheel_velocity[i].y = current_state.velocity.y + rot_vy;
        
        current_state.wheel_positions[i] = calculateIdealWheelPosition(i);
    }
    
    checkRealPosition();
}

void F1PhysicsEngine::checkRealPosition() {
    for (int i = 0; i < 4; i++) {
        Point2D ideal_position = calculateIdealWheelPosition(i);
        current_state.wheel_positions[i] = ideal_position;
    }
}

F1PhysicsEngine::Point2D F1PhysicsEngine::calculateIdealWheelPosition(int wheel_index) {
    double half_wheelbase = params.wheelbase / 2.0;
    double half_track = params.track_width / 2.0;
    
    double cos_angle = std::cos(current_state.car_angle);
    double sin_angle = std::sin(current_state.car_angle);
    
    switch(wheel_index) {
        case 0: // FL
            return Point2D(
                current_state.position.x + half_wheelbase * cos_angle - half_track * sin_angle,
                current_state.position.y + half_wheelbase * sin_angle + half_track * cos_angle
            );
        case 1: // FR
            return Point2D(
                current_state.position.x + half_wheelbase * cos_angle + half_track * sin_angle,
                current_state.position.y + half_wheelbase * sin_angle - half_track * cos_angle
            );
        case 2: // RL
            return Point2D(
                current_state.position.x - half_wheelbase * cos_angle - half_track * sin_angle,
                current_state.position.y - half_wheelbase * sin_angle + half_track * cos_angle
            );
        case 3: // RR
            return Point2D(
                current_state.position.x - half_wheelbase * cos_angle + half_track * sin_angle,
                current_state.position.y - half_wheelbase * sin_angle - half_track * cos_angle
            );
        default:
            return current_state.position;
    }
}
