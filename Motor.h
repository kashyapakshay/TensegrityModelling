#ifndef MOTOR_H
#define MOTOR_H
#include <iostream>
#include <vector>

#define PI 3.14

class Motor {
    public:
        Motor();
        Motor(float initial_phase, float frequency);

        double get_frequency();
        void set_frequency(double);
        void _step_reset();
        void _step_increment();
        std::vector<double> compute_motor_force_point(Strut strut);
    private:
        double _frequency;
        int _step;
        int _limit;
};
#endif
