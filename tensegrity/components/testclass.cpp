#include <iostream>

#include "Strut.h"
#include "Spring.h"
#include "Motor.h"
#include "Tensegrity.h"

using namespace std;

int main() {
    Strut strut, strut2, strut3;
    Spring spring(strut, Strut::TOP_EDGE, strut2, Strut::BOTTOM_EDGE);
    Motor motor(1, 0.695), motor2(1, 0.695);

    cout << motor.get_frequency() << "\n";
    cout << strut.get_mass() << "\n";
    cout << spring.compute_spring_force() << "\n";

    motor.attach_to_strut(&strut);

    return 0;
}
