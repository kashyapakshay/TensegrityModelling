#include "Tensegrity.h"

Tensegrity::Tensegrity() {
    _n_struts = 0, _n_springs = 0;

    strut_vector = {};
    spring_vector = {};

    _motor_speeds = {1.0, 1.0, 1.0};
}

void Tensegrity::add_strut(Strut *strut) {
    _strut_vector.push_back(strut);
    return ++_n_struts;
}

void Tensegrity::add_spring(Spring *spring) {
    _spring_vector.push_back(spring);
    return ++_n_springs;
}

void Tensegrity::move_fwd() {
    
}

void Tensegrity::set_motor_speed(d_vector motor_speeds) {
    _strut_vector[0]->get_motor()->set_speed(_motor_speeds[0]);
    _strut_vector[1]->get_motor()->set_speed(_motor_speeds[1]);
    _strut_vector[2]->get_motor()->set_speed(_motor_speeds[2]);
}
