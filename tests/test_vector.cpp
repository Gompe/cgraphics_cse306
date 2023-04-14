#include <iostream>
#include <chrono>
#include "../gx_vector.h"

template <typename T>
static inline void print(const T& x)
{
    std::cout << x << std::endl;
}

// const int N = 1;
const int N = 100000000;

int main()
{
    
    print(Vector::initMinVector());
    print(Vector::initMaxVector());

    return 0;
}