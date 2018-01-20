#ifndef STRUT_H
#define STRUT_H
#include <iostream>
#include <vector>

#include "ode/ode.h"

// #include "Motor.h"

typedef std::vector<double> d_vector;

class Strut {
    public:
        Strut(dWorldID, dSpaceID);
        Strut(dWorldID, dSpaceID, d_vector, d_vector);
        Strut(dWorldID, dSpaceID, d_vector, d_vector, double, double, double, d_vector);

        static int TOP_EDGE, BOTTOM_EDGE;

		double get_mass();
		double get_length();
		double get_radius();

		d_vector get_coords();
        d_vector get_color();
		d_vector get_angles();

		d_vector get_edge_coords(int);
        d_vector get_top_edge_coords();
        d_vector get_bottom_edge_coords();

		void apply_edge_force(int, d_vector);

        // void attach_motor(Motor*);
        // Motor* get_attached_motor();

    private:
        dWorldID _world;
        dSpaceID _space;
        dBodyID _body;
		dGeomID _geom;

        d_vector _coords;
        d_vector _angles;
        d_vector _color;
        double _length, _mass, _radius;

        dMass _mass_obj;

        const float _DENSITY = 0.5;

        // Motor *_motor;
};
#endif
