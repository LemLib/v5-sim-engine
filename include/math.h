#pragma once
// based on https://people.sc.fsu.edu/~jburkardt/cpp_src/matrix_exponential/matrix_exponential.html
#include <iostream>
#include <unistd.h>
#include <memory>
#include "matrix.h"

namespace sim {
#define SIGNIFICANT_THRESHOLD 2.220446049250313e-16

    template<int t>
    bool significant(algebra::Matrix<double, t, t> &a, algebra::Matrix<double, t, t> &b) {
        for (int i = 0; i < t; i++) {
            for (int j = 0; j < t; j++) {
                double sum = a(i, j) + b(i, j);
                double tol = SIGNIFICANT_THRESHOLD * fabs(a(i, j));
                if (tol < fabs(a(i, j) - sum)) return true;
            }
        }
        return false;
    }


    template<int t>
    algebra::Matrix<double, t, t> expm(algebra::Matrix<double, t, t> &a) {
        algebra::Matrix<double, t, t> e(0);
        algebra::Matrix<double, t, t> f(0);
        for (int i = 0; i < t; i++) { f(i, i) = 1; }

        int k = 1;
        while (significant<t>(e, f)) {
            e += f;
            f = a * f;
            f *= (1.0 / k);
            k++;
        }
        return e;
    }

    template<long long unsigned int aw, long long unsigned int bw, long long unsigned int h>
    algebra::Matrix<double, h, aw + bw> hstack(algebra::Matrix<double, h, aw> &a, algebra::Matrix<double, h, bw> &b) {
        algebra::Matrix<double, h, aw + bw> result;
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < aw; j++) { result(i, j) = a(i, j); }
            for (int j = 0; j < bw; j++) { result(i, j + aw) = b(i, j); }
        }
        return result;
    }

// vstack
    template<long long unsigned int ah, long long unsigned int bh, long long unsigned int w>
    algebra::Matrix<double, ah + bh, w> vstack(algebra::Matrix<double, ah, w> &a, algebra::Matrix<double, bh, w> &b) {
        algebra::Matrix<double, ah + bh, w> result;
        for (int i = 0; i < ah; i++) {
            for (int j = 0; j < w; j++) { result(i, j) = a(i, j); }
        }
        for (int i = 0; i < bh; i++) {
            for (int j = 0; j < w; j++) { result(i + ah, j) = b(i, j); }
        }
        return result;
    }

    template<int t, typename M = algebra::Matrix<double, t, t>>
    std::pair<M, M>
    to_discrete(M &A, M &B, double dt) {
        auto em_upper = hstack<t, t, t>(A, B);
        auto padding = algebra::Matrix<double, t, t*2>::zero();
        auto em = vstack<t, t, t*2>(em_upper, padding);
        em = em * dt;
        auto mss = expm<t * 2>(em);

        // Cut out A and B
        for (int i = 0; i < A.rows(); i++) {
            for (int j = 0; j < A.cols(); j++) { A(i, j) = mss(i, j); }
        }
        for (int i = 0; i < B.rows(); i++) {
            for (int j = 0; j < B.cols(); j++) { B(i, j) = mss(i, j + A.cols()); }
        }

        return std::make_pair(A, B);
    }
}