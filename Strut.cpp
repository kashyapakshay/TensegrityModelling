#include "Strut.h"

using namespace std;

Strut::Strut(d_vector coords, d_vector angles) {
	// _space = space;
	// _world = world;
	_coords = coords;
	_angles = angles;
	_length = 0.1;
	_mass = 1.0;
	_radius = 0.02;
	_color = d_vector(0, 0, 0);
}

// dSpaceID space, dWorldID world
Strut::Strut(d_vector coords, d_vector angles, dReal mass,
	dReal length, dReal radius, d_vector color) {

	// _space = space;
	// _world = world;
	_coords = coords;
	_angles = angles;
	_length = length;
	_mass = mass;
	_radius = radius;
	_color = color;
}

dReal Strut::get_mass() {
	return _mass;
}
