#ifndef MOTOR_H
#define MOTOR_H
#include <iostream>
#include <vector>
#include <cmath>

#include "Strut.h"

#define PI 3.14

class Motor {
    public:
        Motor();
        Motor(float initial_phase, float frequency);

        void attach_to_strut(Strut*);

        double get_frequency();
        void set_frequency(double);

        double get_speed();
        void set_speed(double);

        void _step_reset();
        void _step_increment();

        std::vector<double> compute_motor_force_point(Strut strut);
    private:
        Strut* _strut;
        double _frequency, _speed;
        int _step, _limit;
};
#endif
