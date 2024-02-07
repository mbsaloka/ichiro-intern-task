#include<iostream>
#include<cmath>

int main(){
    for(int i=0; i<=24; i++){
        double angle = (double)i / 12 * M_PI;
        std::cout << i << " " <<  angle << std::endl;
    }
}