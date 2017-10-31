#include "Spring.h"

int Spring::TOP_EDGE = 1;
int Spring::BOTTOM_EDGE = 2;

Spring::Spring(Strut strut_1, int edge_1, Strut strut_2, int edge_2) {
    _spring_constant = 0.001;
    _resting_length = 0.2;

    _strut_1 = strut_1;
    _strut_2 = strut_2;

    _edge_1 = edge_1;
    _edge_2 = edge_2;
}

Spring::Spring(Strut strut_1, int edge_1, Strut strut_2, int edge_2,
    double spring_constant, double resting_length) {

    _strut_1 = strut_1;
    _strut_2 = strut_2;

    _spring_constant = spring_constant;
    _resting_length = resting_length;
}

double Spring::_compute_distance_between_strut_edges() {
    d_vector edge_1 = _strut_1.get_edge_coords(_edge_1);
    d_vector edge_2 = _strut_2.get_edge_coords(_edge_2);

    double dist = sqrt(pow(edge_1[0] - edge_2[0], 2) +
                    pow(edge_1[1] - edge_2[1], 2) +
                    pow(edge_1[2] - edge_2[2], 2));

    return dist;
}

d_vector Spring::_compute_distance_vector_between_strut_edges() {
    d_vector edge_1 = _strut_1.get_edge_coords(_edge_1);
    d_vector edge_2 = _strut_2.get_edge_coords(_edge_2);

    d_vector dist_vector = {
        edge_1[0] - edge_2[0],
        edge_1[1] - edge_2[1],
        edge_1[2] - edge_2[2]
    };

    return dist_vector;
}

double Spring::compute_spring_force() {
    double dist = _compute_distance_between_strut_edges();
    double spring_force = _spring_constant * (dist - _resting_length);

    return spring_force;
}

d_vector Spring::compute_spring_force_unit_vector() {
    double dist = _compute_distance_between_strut_edges();
    d_vector dist_vector = _compute_distance_vector_between_strut_edges();

    double spring_force = compute_spring_force();

    d_vector force_unit_vector = {
        dist_vector[0] * (spring_force / dist),
        dist_vector[1] * (spring_force / dist),
        dist_vector[2] * (spring_force / dist)
    };

    return force_unit_vector;
}
