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
    
    // auto start = std::chrono::high_resolution_clock::now();

    // for(int i=0; i<N; i++){
    //     Vector a1 = Vector(1,2,3);
    //     Vector a2 = Vector(2,3,4);
    //     Vector a3 = Vector(5,3,4);
    //     Vector a4 = Vector(1,3,4);
    //     Vector a5 = Vector(8,3,4);

    //     Vector c = a1 + a2 + a3 + a4 + a5;
    // }

    // auto end = std::chrono::high_resolution_clock::now();
	// auto time_elapsed = std::chrono::duration<double>(end - start).count();
    // std::cout << "elapsed: " << 1000*time_elapsed << std::endl;

    double coords[3][3] = {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };

    Matrix A(coords);

    Vector x(47,15,-30);

    print(A);
    print(A*A);
    print(A+A);
    print(A-A);
    print(2*A);
    print(A/2);    

    A*=A;
    print(A);

    return 0;
}