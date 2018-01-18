#include <iostream>

#include "tensegrity/components/Strut.h"
#include "tensegrity/components/Spring.h"
#include "tensegrity/components/Motor.h"
#include "tensegrity/components/Tensegrity.h"

using namespace std;

int main() {
    Strut strut_1, strut_2, strut_3, strut_4, strut_5, strut_6;

    const int top_edge = Strut::TOP_EDGE;
    const int bottom_edge = Strut::BOTTOM_EDGE;

    Spring spring_1t_5t(&strut_1, top_edge, &strut_5, top_edge);
    Spring spring_1t_6t(&strut_1, top_edge, &strut_6, top_edge);
    Spring spring_1t_3t(&strut_1, top_edge, &strut_3, top_edge);
    Spring spring_1t_3b(&strut_1, top_edge, &strut_3, bottom_edge);

    Spring spring_2t_5b(&strut_2, top_edge, &strut_5, bottom_edge);
    Spring spring_2t_6b(&strut_2, top_edge, &strut_6, bottom_edge);
    Spring spring_2t_3t(&strut_2, top_edge, &strut_3, top_edge);
    Spring spring_2t_3b(&strut_2, top_edge, &strut_3, bottom_edge);

    Spring spring_1b_5t(&strut_1, bottom_edge, &strut_5, top_edge);
    Spring spring_1b_6t(&strut_1, bottom_edge, &strut_6, top_edge);
    Spring spring_1b_4t(&strut_1, bottom_edge, &strut_4, top_edge);
    Spring spring_1b_4b(&strut_1, bottom_edge, &strut_4, bottom_edge);

    Spring spring_2b_5b(&strut_2, bottom_edge, &strut_5, bottom_edge);
    Spring spring_2b_6b(&strut_2, bottom_edge, &strut_6, bottom_edge);
    Spring spring_2b_4t(&strut_2, bottom_edge, &strut_4, top_edge);
    Spring spring_2b_4b(&strut_2, bottom_edge, &strut_4, bottom_edge);

    Spring spring_3t_5t(&strut_3, top_edge, &strut_5, top_edge);
    Spring spring_3t_5b(&strut_3, top_edge, &strut_5, bottom_edge);
    Spring spring_3b_6t(&strut_3, bottom_edge, &strut_6, top_edge);
    Spring spring_3b_6b(&strut_3, bottom_edge, &strut_6, bottom_edge);

    Spring spring_4t_5t(&strut_4, top_edge, &strut_5, top_edge);
    Spring spring_4t_5b(&strut_4, top_edge, &strut_5, bottom_edge);
    Spring spring_4b_6t(&strut_4, bottom_edge, &strut_6, top_edge);
    Spring spring_4b_6b(&strut_4, bottom_edge, &strut_6, bottom_edge);

    Motor motor_1(1, 0.695), motor_2(1, 0.695);

    strut_1.attach_motor(&motor_1);
    strut_3.attach_motor(&motor_2);

    Tensegrity VALTR;

    VALTR.add_strut(&strut_1);
    VALTR.add_strut(&strut_2);
    VALTR.add_strut(&strut_3);
    VALTR.add_strut(&strut_4);
    VALTR.add_strut(&strut_5);
    VALTR.add_strut(&strut_6);

    VALTR.add_spring(&spring_1t_3t);

    cout << motor_1.get_frequency() << "\n";
    cout << strut_1.get_mass() << "\n";
    cout << spring_4b_6b.compute_spring_force() << "\n";

    return 0;
}
