#include <iostream>

using namespace std;

class Motor {
    public:
        Initialize(float initial_phase, float frequency);
    private:
        float _frequency;
        int _step;
        int _limit;
        float PI = 3.14;
}

Motor::Initialize(float initial_phase, float frequency) {
    _step = initial_phase;
    _frequency = frequency;
    _limit = 2 * PI / (frequency);
}

void Motor::set_frequency(float frequency) {
    _frequency = frequency;
}

float Motor::get_frequency(void) {
    return _frequency;
}

Motor::_step_reset(void) {
    _step = 0;
}

Motor::_step_increment(void) {
    ++_step;

    if (_step > _limit)
        _step_reset();
}

void Motor::compute_motor_force(Strut c, float *coords) {
    float angle = _frequency * step;
    float new_x = c.radius * cos(angle);
    float new_y = c.radius * sin(angle);

    coords[0] = new_x;
    coords[1] = new_y;

    _step_increment();
}
