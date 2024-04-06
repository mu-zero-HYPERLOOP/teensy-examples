#include <stdio.h>
#include <cmath>
#include "MovingAvg.h"

bool compare_float(float a, float b, float epsilon = __FLT_EPSILON__){
   return (fabs(a - b) <= epsilon * std::fmax(fabs(a), fabs(b)));
}

void printCheck(float expected, float actual) {
    if (compare_float(expected, actual)) {
        printf("Passed.");
    } else {
        printf("Failed: Expected %f, actual: %f.", expected, actual);
    }
}

int main() {
    MovingAvg<4> avgZero = MovingAvg<4>(0);
    printCheck(0, avgZero.getAvg());
    avgZero.reading(5.f);
    printCheck(5.f/4.f, avgZero.getAvg());

    MovingAvg<4> avgFive = MovingAvg<4>(5);
    printCheck(5.f, avgFive.getAvg());
    avgFive.reading(10.f);
    avgFive.reading(20.f);
    printCheck(10.f, avgFive.getAvg());
}
