#include <iostream>

#include "Motor.h"
#include "Strut.h"

using namespace std;

int main() {
    Motor motor(1, 0.695);
    Strut strut;

    cout << motor.get_frequency() << "\n";
    cout << strut.get_mass() << "\n";

    return 0;
}
