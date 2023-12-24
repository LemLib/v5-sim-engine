#pragma once
// based on https://people.sc.fsu.edu/~jburkardt/cpp_src/matrix_exponential/matrix_exponential.html
#include "matrix.h"

#define SIGNIFICANT_THRESHOLD 0.01

namespace sim {

    template<int t>
    bool significant(algebra::Matrix<double, t, t> &a, algebra::Matrix<double, t, t> &b) {
        for (int i = 0; i < t; i++) {
            for (int j = 0; j < t; j++) {
                if (fabs(a(i, j) - b(i, j)) > SIGNIFICANT_THRESHOLD) return false;
            }
        }
        return true;
    }

    template<int t>
    algebra::Matrix<double, t, t> expm(algebra::Matrix<double, t, t> &a) {
        auto e = algebra::Matrix<double, t, t>(0);
        auto f = algebra::Matrix<double, t, t>(0);
        for (int i = 0; i < t; i++) {
            f(i, i) = 1;
        }

        int k = 1;
        while (significant<t>(*e, f)) {
            e += f;
            f *= a;
            f *= (1.0 / k);
            k++;
        }
        return e;
    }
}