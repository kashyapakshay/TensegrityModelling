#ifndef SPRING_H
#define SPRING_H
#include <iostream>
#include <vector>
#include <cmath>

#include "Strut.h"

typedef std::vector<double> d_vector;
// typedef std::vector<d_vector> 2d_vector;

class Spring {
    public:
        Spring(Strut*, int, Strut*, int);
        Spring(Strut*, int, Strut*, int, double, double);

		int get_edge_one();
		int get_edge_two();

		Strut* get_strut_one();
		Strut* get_strut_two();
		
        double get_spring_constant();
        double get_resting_length();
        double compute_spring_force();
        d_vector compute_spring_force_vector();
    private:
        double _spring_constant, _resting_length;
        Strut *_strut_1, *_strut_2;
        int _edge_1, _edge_2;

        double _compute_distance_between_strut_edges();
        d_vector _compute_distance_vector_between_strut_edges();
};
#endif
