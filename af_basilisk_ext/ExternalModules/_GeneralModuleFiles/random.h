#ifndef RANDOM_H
#define RANDOM_H

#include <Eigen/Dense>
#include <random>
#include <tuple>

/// @brief Build matrix using input distribution
/// @tparam T 
/// @param distrib Distribution (e.g., Normal, uniform)
/// @param rows Number of matrix rows
/// @param cols Number of matrix cols
/// @param generator Random generator
/// @return Matrix
template <typename T>
Eigen::ArrayXd build_mat_constant_distrib(T &distrib, int rows, int cols, std::mt19937 &generator) {
    Eigen::ArrayXd mat(rows, cols);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat(i, j) = distrib(generator);
        }
    }
    return mat;
}

namespace Random {
    /// @brief Constructs 2D matrix drawn from a normal distribution with a single mean and single standard deviation
    /// @param mean Distribution mean
    /// @param std Distribution standard deviation
    /// @param rows Number of matrix rows
    /// @param cols Number of matrix cols
    /// @param generator Random generator
    /// @return Matrix
    Eigen::ArrayXd normal(double mean, double std, int rows, int cols, std::mt19937 &generator) {
        std::normal_distribution<double> distrib(mean, std);
        return build_mat_constant_distrib(distrib, rows, cols, generator);
    }

    /// @brief Constructs 2D matrix drawn from a normal distribution, where each
    /// matrix element has its own mean and standard deviation
    /// @param mean Distribution means, of size RxC
    /// @param std Distribution standard deviations, of size RxC
    /// @param generator Random generator
    /// @return Matrix of size RxC
    template <int R, int C>
    Eigen::Array<double, R, C> normal(const Eigen::Array<double, R, C> &mean,
                                      const Eigen::Array<double, R, C> &std,
                                      std::mt19937 &generator) {
        int rows = mean.rows();
        int cols = mean.cols();

        Eigen::Array<double, R, C> mat;
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                std::normal_distribution<double> distrib(mean(i, j), std(i, j));
                mat(i, j) = distrib(generator);
            }
        }
        return mat;
    }

    /// @brief Constructs 2D matrix drawn from a uniform distribution
    /// @param low Distribution lower bound (low <= x < high)
    /// @param high Distribution higher bound (low <= x < high))
    /// @param rows Number of matrix rows
    /// @param cols Number of matrix cols
    /// @param generator Random generator
    /// @return Matrix        
    Eigen::ArrayXd uniform(double low, double high, int rows, int cols, std::mt19937 &generator) {
        std::uniform_real_distribution<double> distrib(low, high);
        return build_mat_constant_distrib(distrib, rows, cols, generator);
    }

    /// @brief Constructs 2D matrix drawn from a uniform distribution, where
    /// matrix element has its own lower and upper bound
    /// @param low Distribution lower bound (low <= x < high), of size RxC
    /// @param high Distribution higher bound (low <= x < high)), of size RxC
    /// @param generator Random generator
    /// @return Matrix of size RxC
    template <int R, int C>
    Eigen::Array<double, R, C> uniform(const Eigen::Array<double, R, C> &mean,
                                       const Eigen::Array<double, R, C> &std,
                                       std::mt19937 &generator) {
        int rows = mean.rows();
        int cols = mean.cols();

        Eigen::Array<double, R, C> mat;
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                std::uniform_real_distribution<double> distrib(mean(i, j),
                                                               std(i, j));
                mat(i, j) = distrib(generator);
            }
        }
        return mat;
    }
} // namespace Random

#endif
