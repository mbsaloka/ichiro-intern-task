#include <iostream>
#include <cmath>

int main() {
    for (int i = 0; i < 12; i++) {
        double angle = (double)i / 12 * M_PI;
        std::cout << i << " " << angle << std::endl;
    }

    for (int i = 12; i > 0; i--) {
        double angle = (double)i / 12 * M_PI * -1;
        std::cout << i << " " << angle << std::endl;
    }
}