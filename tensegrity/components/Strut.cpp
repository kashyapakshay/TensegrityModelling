#include "Strut.h"

Strut::Strut() {
	// _space = space;
	// _world = world;
	// std::d_vector default_vec(0.0, 0.0, 0.0);
	_coords = {0, 0, 0};
	_angles = {0, 0, 0};
	_length = 0.1;
	_mass = 1.0;
	_radius = 0.02;
	_color = {0, 0, 0};
}

Strut::Strut(d_vector coords, d_vector angles) {
	// _space = space;
	// _world = world;
	// std::d_vector default_vec(0.0, 0.0, 0.0);
	_coords = coords;
	_angles = angles;
	_length = 0.1;
	_mass = 1.0;
	_radius = 0.02;
	_color = {0, 0, 0};
}

// dSpaceID space, dWorldID world
Strut::Strut(d_vector coords, d_vector angles, double mass,
	double length, double radius, d_vector color) {

	// _space = space;
	// _world = world;
	_coords = coords;
	_angles = angles;
	_length = length;
	_mass = mass;
	_radius = radius;
	_color = color;
}

double Strut::get_mass() {return _mass;}
double Strut::get_radius() {return _radius;}
double Strut::get_length() {return _length;}
d_vector Strut::get_coords() {return _coords;}
d_vector Strut::get_color() {return _color;}
d_vector Strut::get_angles() {return _angles;}
