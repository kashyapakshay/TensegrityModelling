#ifndef STRUT_H
#define STRUT_H
#include <iostream>
#include <vector>

#include "ode/ode.h"

typedef std::vector<double> d_vector;

class Strut {
    public:
        Strut(d_vector coords, d_vector angles);
        Strut(d_vector coords, d_vector angles, dReal mass,
            dReal length, dReal radius, d_vector color);

        // d_vector get_coords();
        // d_vector get_angles();
        dReal get_mass();
        // dReal get_length();
        // dReal get_radius();
        // d_vector get_color();
        // d_vector compute_motor_force_point(Strut strut);
    private:
        // dSpaceID _space;
        // dWorldID _world;
        d_vector _coords;
        d_vector _angles;
        d_vector _color;
        dReal _length, _mass, _radius;
};
#endif
