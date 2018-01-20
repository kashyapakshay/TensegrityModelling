#include "Strut.h"

int Strut::TOP_EDGE = 1;
int Strut::BOTTOM_EDGE = -1;

Strut::Strut(dWorldID world, dSpaceID space) {
	_coords = {0, 0, 0};
	_angles = {0, 0, 0};
	_length = 0.1;
	_mass = 1.0;
	_radius = 0.02;
	_color = {0, 0, 0};

	// _motor = NULL;

	_world = world;
	_space = space;
	_body = dBodyCreate(world);

	_geom = dCreateCapsule(space, _radius, _length);

	dMassSetZero(&_mass_obj);
    dMassSetCapsule(&_mass_obj, _DENSITY, 3, _radius, _length);
    dBodySetMass(_body, &_mass_obj);

    dBodySetPosition(_body, _coords[0], _coords[1], _coords[2]);
    dBodySetRotation(_body, _angles);

    dGeomSetBody(_geom, _body);
}

Strut::Strut(dWorldID world, dSpaceID space, d_vector coords, d_vector angles) {
	_coords = coords;
	_angles = angles;
	_length = 0.1;
	_mass = 1.0;
	_radius = 0.02;
	_color = {0, 0, 0};

	// _motor = NULL;

	_world = world;
	_space = space;
	_body = dBodyCreate(world);

	_geom = dCreateCapsule(space, _radius, _length);

	dMassSetZero(&_mass_obj);
    dMassSetCapsule(&_mass_obj, _DENSITY, 3, _radius, _length);
    dBodySetMass(_body, &_mass_obj);

    dBodySetPosition(_body, _coords[0], _coords[1], _coords[2]);
    dBodySetRotation(_body, _angles);

    dGeomSetBody(_geom, _body);
}

// dSpaceID space, dWorldID world
Strut::Strut(dWorldID world, dSpaceID space, d_vector coords,
	d_vector angles, double mass, double length, double radius, d_vector color) {

	_coords = coords;
	_angles = angles;
	_length = length;
	_mass = mass;
	_radius = radius;
	_color = color;

	// _motor = NULL;

	_world = world;
	_space = space;
	_body = dBodyCreate(world);

	_geom = dCreateCapsule(space, _radius, _length);

	dMassSetZero(&_mass_obj);
    dMassSetCapsule(&_mass_obj, _DENSITY, 3, _radius, _length);
    dBodySetMass(_body, &_mass_obj);

    dBodySetPosition(_body, _coords[0], _coords[1], _coords[2]);
    dBodySetRotation(_body, _angles);

    dGeomSetBody(_geom, _body);
}

double Strut::get_mass() {return _mass;}
double Strut::get_radius() {return _radius;}
double Strut::get_length() {return _length;}

d_vector Strut::get_coords() {return _coords;}
d_vector Strut::get_color() {return _color;}
d_vector Strut::get_angles() {
	dMatrix3 angle;
	angle =	dBodyGetRotation(_body);

	return {angle[0], angle[1], angle[2]};
}

d_vector Strut::get_edge_coords(int edge_dir) {
	dVector3 edge_coords;
	dBodyGetRelPointPos(_body, 0, 0, edge_dir * _length / 2, edge_coords);

	return {edge_coords[0], edge_coords[1], edge_coords[2]};
}

d_vector Strut::get_top_edge_coords() {return get_edge_coords(1);}
d_vector Strut::get_bottom_edge_coords() {return get_edge_coords(-1);}

void Strut::apply_edge_force(int edge_dir, d_vector forces) {
	dBodyAddForceAtRelPos(
		_body,
        forces[0], forces[1], forces[2],
        0, 0, edge_dir * _length / 2
	);
}

// void Strut::attach_motor(Motor *motor) {_motor = motor;}
// Motor* Strut::get_attached_motor() {return _motor;}
