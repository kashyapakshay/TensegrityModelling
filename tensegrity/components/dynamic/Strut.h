#ifndef STRUT_H
#define STRUT_H
#include <iostream>
#include <vector>

#include "ode/ode.h"
#include "drawstuff/drawstuff.h"

// #include "Motor.h"

typedef std::vector<double> d_vector;

class Strut {
    public:
        Strut(dWorldID, dSpaceID);
        // Strut(dWorldID, dSpaceID, d_vector, d_vector);
        // Strut(dWorldID, dSpaceID, d_vector, d_vector, double, double, double, d_vector);

        static int TOP_EDGE, BOTTOM_EDGE;

        dBodyID get_body();

		dReal get_mass();
		dReal get_length();
		dReal get_radius();

		d_vector get_coords();
        d_vector get_color();
		// d_vector get_angles();

		d_vector get_edge_coords(int);
        d_vector get_top_edge_coords();
        d_vector get_bottom_edge_coords();

		void apply_edge_force(int, d_vector);
        // void draw();

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
        dReal _length, _mass, _radius;

        dMass _mass_obj;

        const double _DENSITY = 0.5;

        // Motor *_motor;
};
#endif
