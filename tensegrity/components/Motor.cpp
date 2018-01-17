#include "Motor.h"

using namespace std;

Motor::Motor() {
    _step = 0;
    _frequency = PI / 8;
    _limit = (2 * PI) / _frequency;
    _speed = 1.0;
    _strut = NULL;
}

Motor::Motor(float initial_phase, float frequency) {
    _step = initial_phase;
    _frequency = frequency;
    _limit = 2 * PI / (_frequency);
    _speed = 1.0;
    _strut = NULL;
}

void Motor::attach_to_strut(Strut *strut) {_strut = strut;}

void Motor::set_frequency(double frequency) {_frequency = frequency;}
double Motor::get_frequency() {return _frequency;}

void Motor::set_speed(double speed) {_speed = speed;}
double Motor::get_speed() {return _speed;}


void Motor::_step_reset() {_step = 0;}
void Motor::_step_increment() {
    ++_step;

    if (_step > _limit)
        _step_reset();
}

vector<double> Motor::compute_motor_force_point(Strut strut) {
    double angle = _frequency * _step;
    double strut_radius = strut.get_radius();

    double next_x = strut_radius * cos(angle);
    double next_y = strut_radius * sin(angle);

    vector<double> force_point(next_x, next_y);

    _step_increment();

    return force_point;
}
