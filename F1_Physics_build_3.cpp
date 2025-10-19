#include "F1_Physics_build_2.h"
#include <cmath>
#include <algorithm>

// === КОНСТРУКТОР И СБРОС ===

F1PhysicsEngine::F1PhysicsEngine() {
    reset();
}

void F1PhysicsEngine::reset() {
    current_state = CarState();  // Обнуляем всё состояние
    current_state.current_gear = 1;
    // Рассчитываем начальные позиции колес
    for (int i = 0; i < 4; i++) {
        current_state.wheel_positions[i] = calculateIdealWheelPosition(i);
    }
}

// === ПУБЛИЧНЫЕ МЕТОДЫ ===

void F1PhysicsEngine::update(double dt, bool gas_pedal, bool brake_pedal, double steering) {
    current_state.steering_wheel = steering;  // Сохраняем ввод руля
    
    // 1. Двигатель и трансмиссия
    calculateEnginePhysics(gas_pedal, dt);
    
    // 2. Силы
    calculateForces(gas_pedal, brake_pedal, steering);
    calculateRotationForces(dt);
    
    // 3. Движение
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

// === СИЛЫ ЦЕНТРА МАСС ===

double F1PhysicsEngine::calculateTractionForce() const {
    double max_traction = params.tire_friction * (params.mass * 9.81 + current_state.down_force);
    return std::min(current_state.traction_force, max_traction);
}

double F1PhysicsEngine::calculateDragForce() const {
    double drag_force = -0.5 * params.air_density * 
                       current_state.speed * std::abs(current_state.speed) *
                       params.drag_coefficient * params.frontal_area;
    return drag_force;
}

double F1PhysicsEngine::calculateDownForce() const {
    double down_force = 0.5 * params.air_density * 
                       current_state.speed * std::abs(current_state.speed) *
                       params.downforce_coefficient * params.frontal_area;
    return down_force;
}

double F1PhysicsEngine::calculateBrakeForce() const {
    return -current_state.brake_factor * params.max_brake_force;
}

void F1PhysicsEngine::calculateForces(bool gas_pedal, bool brake_pedal, double steering) {

    current_state.steering_wheel = steering;

    current_state.traction_force = gas_pedal ? calculateTractionForce() : 0.0;
    current_state.drag_force = calculateDragForce();
    current_state.brake_force = brake_pedal ? calculateBrakeForce() : 0.0;
    current_state.down_force = calculateDownForce();
    
    calculateBrakeFactor(brake_pedal, 0.01);
    if (brake_pedal) {
        applyBrakes(0.01);
    }
}

// === СИЛЫ ПОВОРОТА ===

void F1PhysicsEngine::calculateRotationForces(double dt) {
    calculateWheelLoads();
    calculateAngularVelocity();
    calculateWheelSlipAngle(dt);
    calculateRotationRadius();
    calculateSideForce();
    calculateCentrForce();
    calculateWheelForces();
}

void F1PhysicsEngine::calculateWheelLoads() {
    double base_load = params.mass * 9.81 / 4;
    double weight_transfer = std::abs(current_state.steering_wheel) * 0.3;
  
    current_state.wheel_loads[0] = base_load * (1 - weight_transfer);
    current_state.wheel_loads[1] = base_load * (1 + weight_transfer);
    current_state.wheel_loads[2] = base_load * (1 - weight_transfer);
    current_state.wheel_loads[3] = base_load * (1 + weight_transfer);
}

void F1PhysicsEngine::calculateAngularVelocity() {
    double max_angular_velocity = 2.0;
    current_state.angular_velocity = current_state.steering_wheel * 
                                   (current_state.speed / 50.0) * 
                                   max_angular_velocity;
    current_state.angular_velocity = std::clamp(current_state.angular_velocity, -3.0, 3.0);
}

void F1PhysicsEngine::calculateWheelSlipAngle(double dt) {
    current_state.car_angle += current_state.angular_velocity * dt;
    
    // Нормализация угла
    if (current_state.car_angle > 2 * M_PI) current_state.car_angle -= 2 * M_PI;
    if (current_state.car_angle < 0) current_state.car_angle += 2 * M_PI;
    
    double movement_direction = std::atan2(current_state.velocity.y, current_state.velocity.x);
    
    current_state.wheel_slip_angle[0] = movement_direction - (current_state.car_angle + current_state.steering_wheel);
    current_state.wheel_slip_angle[1] = movement_direction - (current_state.car_angle + current_state.steering_wheel);
    current_state.wheel_slip_angle[2] = movement_direction - current_state.car_angle;
    current_state.wheel_slip_angle[3] = movement_direction - current_state.car_angle;
    
    // Ограничение углов скольжения
    for (int i = 0; i < 4; i++) {
        current_state.wheel_slip_angle[i] = std::clamp(current_state.wheel_slip_angle[i], -0.5, 0.5);
    }
}

void F1PhysicsEngine::calculateRotationRadius() {
    double min_radius = 10.0;
    double max_radius = 200.0;
    current_state.rotation_radius = min_radius + 
                                  (1.0 - std::abs(current_state.steering_wheel)) * 
                                  (max_radius - min_radius);
    current_state.rotation_radius = std::clamp(current_state.rotation_radius, 5.0, 500.0);
}

void F1PhysicsEngine::calculateSideForce() {
    // Если руль прямой - боковых сил нет
    if (std::abs(current_state.steering_wheel) < 0.01) {
        for (int i = 0; i < 4; i++) {
            current_state.side_force[i] = 0.0;
        }
        return;
    }
    
    // Передние колеса
    for (int i = 0; i < 2; i++) {
        double lateral_force = params.k_front * current_state.wheel_slip_angle[i] * current_state.wheel_loads[i];
        double max_lateral_force = params.mu_front * current_state.wheel_loads[i];
        current_state.side_force[i] = std::clamp(lateral_force, -max_lateral_force, max_lateral_force);
    }
    
    // Задние колеса
    for (int i = 2; i < 4; i++) {
        double lateral_force = params.k_rear * current_state.wheel_slip_angle[i] * current_state.wheel_loads[i];
        double max_lateral_force = params.mu_rear * current_state.wheel_loads[i];
        current_state.side_force[i] = std::clamp(lateral_force, -max_lateral_force, max_lateral_force);
    }
}

void F1PhysicsEngine::calculateCentrForce() {
    double total_centrifugal_force = params.mass * current_state.speed * current_state.speed / current_state.rotation_radius;
    
    double total_load = 0.0;
    for (int i = 0; i < 4; i++) {
        total_load += current_state.wheel_loads[i];
    }
    
    for (int i = 0; i < 4; i++) {
        double load_ratio = current_state.wheel_loads[i] / total_load;
        current_state.centr_force[i] = total_centrifugal_force * load_ratio;
    }
}

void F1PhysicsEngine::calculateWheelForces() {
    double traction_per_rear_wheel = current_state.traction_force / 2.0;
    double brake_front = current_state.brake_force * 0.6 / 2.0;
    double brake_rear = current_state.brake_force * 0.4 / 2.0;
    double drag_per_wheel = current_state.drag_force / 4.0;
    
    for (int i = 0; i < 4; i++) {
        double longitudinal = 0.0;
        if (i < 2) {
            longitudinal = -brake_front - drag_per_wheel;
        } else {
            longitudinal = traction_per_rear_wheel - brake_rear - drag_per_wheel;
        }
        
        double lateral = current_state.side_force[i] - current_state.centr_force[i];
        current_state.wheel_forces[i] = Vector2D(longitudinal, lateral);
    }
}

// === ДВИЖЕНИЕ И ГЕОМЕТРИЯ ===

void F1PhysicsEngine::integrateMotion(double dt) {
    // 1. Центр масс
    Vector2D total_force = {0.0, 0.0};
    for (int i = 0; i < 4; i++) {
        total_force.x += current_state.wheel_forces[i].x;
        total_force.y += current_state.wheel_forces[i].y;
    }
    
    current_state.acceleration.x = total_force.x / params.mass;
    current_state.acceleration.y = total_force.y / params.mass;
    
    current_state.velocity.x += current_state.acceleration.x * dt;
    current_state.velocity.y += current_state.acceleration.y * dt;
    
    current_state.position.x += current_state.velocity.x * dt;
    current_state.position.y += current_state.velocity.y * dt;
    
    // 2. Колеса
    for (int i = 0; i < 4; i++) {
        current_state.wheel_acceleration[i].x = current_state.wheel_forces[i].x / (params.mass / 4.0);
        current_state.wheel_acceleration[i].y = current_state.wheel_forces[i].y / (params.mass / 4.0);
        
        current_state.wheel_velocity[i].x += current_state.wheel_acceleration[i].x * dt;
        current_state.wheel_velocity[i].y += current_state.wheel_acceleration[i].y * dt;
        
        current_state.wheel_positions[i].x += current_state.wheel_velocity[i].x * dt;
        current_state.wheel_positions[i].y += current_state.wheel_velocity[i].y * dt;
    }
    
    // 3. Общая скорость
    current_state.speed = std::sqrt(current_state.velocity.x * current_state.velocity.x + 
                                   current_state.velocity.y * current_state.velocity.y);
    
    // 4. Коррекция
    checkRealPosition();
}

void F1PhysicsEngine::checkRealPosition() {
    for (int i = 0; i < 4; i++) {
        Point2D ideal_position = calculateIdealWheelPosition(i);
        Point2D position_diff = {
            ideal_position.x - current_state.wheel_positions[i].x,
            ideal_position.y - current_state.wheel_positions[i].y
        };
        
        current_state.wheel_positions[i].x += position_diff.x * params.stiffness;
        current_state.wheel_positions[i].y += position_diff.y * params.stiffness;
        
        current_state.wheel_velocity[i].x += position_diff.x * params.stiffness * 0.1;
        current_state.wheel_velocity[i].y += position_diff.y * params.stiffness * 0.1;
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