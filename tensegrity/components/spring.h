#ifndef SPRING_H
#define SPRING_H
#include <iostream>
#include <vector>

typedef std::vector<double> d_vector;
typedef std::vector<d_vector> 2d_vector;

class Spring {
    public:
        Spring();
        Spring(double spring_constant, double resting_length);

        double get_spring_constant();
        double get_resting_length();
        2d_vector compute_spring_force_vectors();

        // d_vector compute_motor_force_point(Strut strut);
    private:
        double _spring_constant, _resting_length;
};
#endif
