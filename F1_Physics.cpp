#include "F1_Physics.h"
#include <cmath>
#include <algorithm>

// === КОНСТРУКТОР И СБРОС ===

F1PhysicsEngine::F1PhysicsEngine() {
    reset();
}

void F1PhysicsEngine::reset() {
    current_state = CarState();  // Обнуляем всё состояние
    calculateWheelPositions();   // Рассчитываем начальные позиции колес
}

// === ПУБЛИЧНЫЕ МЕТОДЫ ===

void F1PhysicsEngine::update(double dt, bool gas_pedal, bool brake_pedal, double steering) {
    // 1. Двигатель
    calculateEnginePhysics(gas_pedal, dt);
    
    // 2. Силы
    calculateForces(gas_pedal, brake_pedal, steering);
    
    // 3. Движение
    integrateMotion(dt);
    
    // 4. Геометрия
    calculateWheelPositions();
}


// === ПРИВАТНЫЕ МЕТОДЫ РАСЧЕТА ===

void F1PhysicsEngine::calculateEnginePhysics(bool gas_pedal, double dt) {

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

// === ЗАГЛУШКИ - реализуем в следующих уроках ===

void F1PhysicsEngine::calculateForces(bool gas_pedal, bool brake_pedal, double steering) {
    // 1. СИЛА ТЯГИ (только при нажатом газе)
    current_state.traction_force = gas_pedal ? calculateTractionForce() : 0.0;
    
    // 2. СОПРОТИВЛЕНИЕ ВОЗДУХА (всегда против движения)
    current_state.drag_force = calculateDragForce();
    
    // 3. СИЛА ТОРМОЖЕНИЯ (только при нажатом тормозе)
    current_state.brake_force = brake_pedal ? calculateBrakeForce() : 0.0;
    
    // 4. ПРИЖИМНАЯ СИЛА (вертикальная - пока не влияет на 1D движение)
    current_state.down_force = calculateDownForce();
    
    // 💡 ПРИМЕЧАНИЕ: steering пока не используем для 1D движения
}

double F1PhysicsEngine::calculateTractionForce() const {
    // То, что сюда должен саня скинуть 
    double traction_force = 1;
    double max_traction = params.tire_friction * (params.mass * 9.81 + current_state.down_force);
    
    // Не даем силе тяги превысить силу сцепления
    return std::min(traction_force, max_traction);


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
    // Для F1 C_l ≈ -3.0 (отрицательный = прижимная сила)
    
  
    
    double down_force = 0.5 * params.air_density * 
                       current_state.speed * std::abs(current_state.speed) *
                       params.downforce_coefficient * params.frontal_area;
    
    // Прижимная сила всегда отрицательная (вниз)
    return down_force;
}
double F1PhysicsEngine::calculateBrakeForce() const {
    //Саня должен делать
    double brake_force = 1;
    return brake_force;
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