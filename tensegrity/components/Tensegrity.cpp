#include "Tensegrity.h"

Tensegrity::Tensegrity() {
    _n_struts = 0, _n_springs = 0;

    _strut_vector = {};
    _spring_vector = {};

    _motor_speeds = {1.0, 1.0, 1.0};
}

void Tensegrity::add_strut(Strut *strut) {
    _strut_vector.push_back(strut);
    ++_n_struts;
}

void Tensegrity::add_spring(Spring *spring) {
    _spring_vector.push_back(spring);
    ++_n_springs;
}

void Tensegrity::move_fwd() {

}
