#include <iostream>

float f(float x) {
    return (-0.8) * x * x + 3.0 * x - 2.0;
}

float integrateRectangle(float a, float b, float h) {
    float result = 0.0;
    for (float x = a; x < b; x += h) {
        result += f(x) * h;
    }
    return result;
}

int main() {
    float a, b;
    float h = 0.001;

    std::cin >> a >> b;

    float integralResult = integrateRectangle(a, b, h);

    std::cout << integralResult << std::endl;

    return 0;
}
