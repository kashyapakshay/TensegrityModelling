#include "Strut.h"

int Strut::TOP_EDGE = 1;
int Strut::BOTTOM_EDGE = -1;

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
	// _motor = NULL;
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
	// _motor = NULL;
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
	// _motor = NULL;
}

double Strut::get_mass() {return _mass;}
double Strut::get_radius() {return _radius;}
double Strut::get_length() {return _length;}

d_vector Strut::get_edge_coords(int edge_dir) {
	// Find a better way to translate. Maybe search for a translate() method?
	//  Also, this is wrong.
	d_vector top_edge = {
		_coords[0] + edge_dir * (_length / 2),
		_coords[1] + edge_dir * (_length / 2),
		_coords[2] + edge_dir * (_length / 2)
	};

	// d_vector top_edge;
	// dBodyGetRelPointPos(_body, 0, 0, _length / 2, &top_edge);

	return top_edge;
}

d_vector Strut::get_top_edge_coords() {return get_edge_coords(1);}
d_vector Strut::get_bottom_edge_coords() {return get_edge_coords(-1);}

d_vector Strut::get_coords() {return _coords;}
d_vector Strut::get_color() {return _color;}
d_vector Strut::get_angles() {return _angles;}

void Strut::attach_motor(Motor *motor) {_motor = motor;}
Motor* Strut::get_attached_motor() {return _motor;}
