#include <iostream>
#include "Matrix.h"

int main() {
    // Create a 2x2 matrix
    Matrix A(2, 2);
    A.setElement(0, 0, 1);
    A.setElement(0, 1, 2);
    A.setElement(1, 0, 3);
    A.setElement(1, 1, 4);

    // Create another 2x2 matrix
    Matrix B(2, 2);
    B.setElement(0, 0, 5);
    B.setElement(0, 1, 6);
    B.setElement(1, 0, 7);
    B.setElement(1, 1, 8);

    // Print matrix A
    std::cout << "Matrix A:" << std::endl;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            std::cout << A.getElement(i, j) << " ";
        }
        std::cout << std::endl;
    }

    // Print matrix B
    std::cout << "Matrix B:" << std::endl;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            std::cout << B.getElement(i, j) << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
