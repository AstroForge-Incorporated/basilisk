#ifndef EIGEN_SUPPORT_H
#define EIGEN_SUPPORT_H

#include <Eigen/Dense>

/// @brief Converts input C array to an Eigen Array3d
/// @param inArray Input C array
/// @return Eigen Array3d
Eigen::Array3d cArray2EigenArray3d(double *inArray) {
    return Eigen::Map<Eigen::Array3d>(inArray, 3, 1);
}

/// @brief Helper function to print array, for debugging 
/// @param arr Array to print
void print_arr(double arr[]) {
    int size = *(&arr + 1) - arr;
    int i = 0;

    for (i = 0; i < size; ++i) {
        printf("\n\tarr[%d] : %f", i, arr[i]);
    }
    printf("\n");
}

#endif
