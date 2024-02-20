#include <iostream>
#include <random>

int main(int argc, char const *argv[]) {
    int counter[11] = {0};
    std::random_device rd;
    // std::normal_distribution<double> nd(5.0, 1.0);
    // for (int i = 1; i <= 10; i++) {
    //     // std::cout << i << " : " << nd(rd) << std::endl;
    //     int j = (int)nd(rd);
    //     counter[j]++;
    // }

    // std::uniform_real_distribution<> urd(1, 11);
    // for (int i = 1; i <= 50; i++) {
    //     int j = (int)urd(rd);
    //     // std::cout << i << " : " << j << std::endl;
    //     counter[j]++;
    // }

    std::mt19937 gen(rd());
    std::bernoulli_distribution bernoulli_dist(0.2);

    for (int i = 0; i < 10; ++i) {
        bool result = bernoulli_dist(gen);
        // std::cout << i << " : " << result << std::endl;

        if (result) {
            counter[1]++;
        } else {
            counter[0]++;
        }
    }

    for (int i = 0; i <= 10; i++) {
        std::cout << i << " : " << counter[i] << std::endl;
    }

    return 0;
}
