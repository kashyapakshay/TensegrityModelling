#ifndef STRUT_H
#define STRUT_H
#include <iostream>
#include <vector>

typedef std::vector<double> d_vector;

class Strut {
    public:
        Strut();
        Strut(d_vector coords, d_vector angles);
        Strut(d_vector coords, d_vector angles, double mass,
            double length, double radius, d_vector color);

        d_vector get_coords();
        d_vector get_angles();
        double get_mass();
        double get_length();
        double get_radius();
        d_vector get_color();
        // d_vector compute_motor_force_point(Strut strut);
    private:
        // dSpaceID _space;
        // dWorldID _world;
        d_vector _coords;
        d_vector _angles;
        d_vector _color;
        double _length, _mass, _radius;
};
#endif
