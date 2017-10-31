#include <iostream>

#include "Motor.h"
#include "Strut.h"
#include "Spring.h"

using namespace std;

int main() {
    Motor motor(1, 0.695);
    Strut strut, strut2;
    Spring spring(strut, 1, strut2, -1);

    cout << motor.get_frequency() << "\n";
    cout << strut.get_mass() << "\n";
    cout << spring.compute_spring_force() << "\n";

    return 0;
}
