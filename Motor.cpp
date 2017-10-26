#include "Motor.h"

using namespace std;

Motor::Motor() {
    _step = 0;
    _frequency = PI / 8;
    _limit = 2 * PI / (frequency);
}

Motor::Motor(float initial_phase, float frequency) {
    _step = initial_phase;
    _frequency = frequency;
    _limit = 2 * PI / (frequency);
}

void Motor::set_frequency(float frequency) {
    _frequency = frequency;
}

double Motor::get_frequency() {
    return _frequency;
}

void Motor::_step_reset() {
    _step = 0;
}

void Motor::_step_increment() {
    ++_step;

    if (_step > _limit)
        _step_reset();
}

vector<double> Motor::compute_motor_force_point(Strut c) {
    double angle = _frequency * step;
    double new_x = c.radius * cos(angle);
    double new_y = c.radius * sin(angle);

    vector<double> coords = {new_x, new_y};

    _step_increment();

    return coords;
}
