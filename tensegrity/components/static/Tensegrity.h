#ifndef TENSEGRITY_H
#define TENSEGRITY_H
#include <iostream>
#include <vector>
#include <cmath>

#include "Strut.h"
#include "Spring.h"
#include "Motor.h"

typedef std::vector<double> d_vector;

class Tensegrity {
    public:
        Tensegrity();

        void add_strut(Strut*);
        void add_spring(Spring*);

        void move_fwd();
    private:
        int _n_struts, _n_springs;

        std::vector<Strut*> _strut_vector;
        std::vector<Spring*> _spring_vector;

        d_vector _motor_speeds;
};
#endif
