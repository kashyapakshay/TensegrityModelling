#ifndef TENSEGRITY_H
#define TENSEGRITY_H
#include <iostream>
#include <vector>
#include <cmath>

#include "Strut.h"
#include "Spring.h"

typedef std::vector<double> d_vector;

class Spring {
    public:
        Tensegrity();

        void add_strut(Strut *);
        void add_spring(Spring *);
        void move_fwd();
        void set_motor_speed(d_vector);
    private:
        int _n_struts, _n_springs;

        std::vector<Strut> _strut_vector;
        std::vector<Spring> _spring_vector;

        d_vector _motor_speeds;
};
#endif
