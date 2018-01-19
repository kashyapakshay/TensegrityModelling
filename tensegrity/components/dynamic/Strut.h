#ifndef STRUT_H
#define STRUT_H
#include <iostream>
#include <vector>

#include "ode/ode.h"

#include "Motor.h"

typedef std::vector<double> d_vector;

class Strut {
    public:
        Strut(dWorldID, dSpaceID);
        Strut(dWorldID, dSpaceID, d_vector, d_vector);
        Strut(dWorldID, dSpaceID, d_vector, d_vector, double, double, double, d_vector);

        static int TOP_EDGE, BOTTOM_EDGE;

        d_vector get_coords();
        d_vector get_angles();
        double get_mass();
        double get_length();
        double get_radius();
        d_vector get_color();
        d_vector get_edge_coords(int);
        d_vector get_top_edge_coords();
        d_vector get_bottom_edge_coords();

        void attach_motor(Motor*);
        Motor* get_attached_motor();
        // d_vector compute_motor_force_point(Strut strut);

    private:
        dWorldID _world_ID;
        dSpaceID _space_ID;
        dBodyID _body_ID;

        d_vector _coords;
        d_vector _angles;
        d_vector _color;
        double _length, _mass, _radius;

        dMass _mass_obj;

        const float _DENSITY = 0.5;

        Motor *_motor;
};
#endif
