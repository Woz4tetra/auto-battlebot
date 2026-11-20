#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace auto_battlebot::ekf {

template <int N>
using Vec = Eigen::Matrix<double, N, 1>;
template <int Rows, int Cols>
using Mat = Eigen::Matrix<double, Rows, Cols>;

inline double wrap_angle(double angle) {
    const double wrapped = std::atan2(std::sin(angle), std::cos(angle));
    return wrapped <= -M_PI ? M_PI : wrapped;
}

template <int N>
void symmetrize(Mat<N, N> &covariance) {
    covariance = ((covariance + covariance.transpose()) * 0.5).eval();
}

template <int N>
void floor_diagonal(Mat<N, N> &covariance, double floor) {
    for (int i = 0; i < N; ++i) {
        if (covariance(i, i) < floor) covariance(i, i) = floor;
    }
}

template <int N>
void propagate_covariance(Mat<N, N> &covariance, const Mat<N, N> &transition,
                          const Mat<N, N> &process_noise) {
    covariance = (transition * covariance * transition.transpose() + process_noise).eval();
    symmetrize<N>(covariance);
}

struct UpdateOutcome {
    bool accepted = false;
    double nis = 0.0;
};

template <int N, int M>
UpdateOutcome ekf_update(Vec<N> &state, Mat<N, N> &covariance, const Vec<M> &measurement,
                         const Vec<M> &predicted_measurement, const Mat<M, N> &observation,
                         const Mat<M, M> &measurement_noise, double gate_nis,
                         const std::array<bool, M> &angle_rows,
                         const std::array<bool, N> &angle_states, double covariance_floor) {
    Vec<M> innovation = measurement - predicted_measurement;
    for (int i = 0; i < M; ++i) {
        if (angle_rows[static_cast<size_t>(i)]) {
            innovation(i) = wrap_angle(innovation(i));
        }
    }

    const Mat<M, M> innovation_covariance =
        observation * covariance * observation.transpose() + measurement_noise;
    const Eigen::LLT<Mat<M, M>> llt(innovation_covariance);

    UpdateOutcome outcome;
    outcome.nis = innovation.dot(llt.solve(innovation));
    outcome.accepted = gate_nis <= 0.0 || outcome.nis <= gate_nis;
    if (!outcome.accepted) {
        return outcome;
    }

    const Mat<N, M> gain = covariance * observation.transpose() * llt.solve(Mat<M, M>::Identity());
    state += gain * innovation;
    for (int i = 0; i < N; ++i) {
        if (angle_states[static_cast<size_t>(i)]) {
            state(i) = wrap_angle(state(i));
        }
    }

    const Mat<N, N> identity_minus_kh = Mat<N, N>::Identity() - gain * observation;
    covariance = (identity_minus_kh * covariance * identity_minus_kh.transpose() +
                  gain * measurement_noise * gain.transpose())
                     .eval();
    symmetrize<N>(covariance);
    floor_diagonal<N>(covariance, covariance_floor);
    return outcome;
}

}  // namespace auto_battlebot::ekf
