// nanobind module for the run-away solver experiment. Exposes exactly two things: the
// Method enum and run_batch(), which loops and times internally so the language boundary
// is crossed twice per run instead of twice per tick. Timing wraps only the solver call;
// the opponent buffer is hoisted and reused so nothing allocates inside the timed region.

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "target_selector/empty_circle_solver.hpp"

namespace nb = nanobind;
using namespace nb::literals;

namespace {

enum class Method { COARSE_GRID, EXACT, BRANCH_AND_BOUND, BRUTE_FORCE };

using DoubleArray = nb::ndarray<nb::numpy, double, nb::ndim<1>>;
using Int64Array = nb::ndarray<nb::numpy, int64_t, nb::ndim<1>>;

DoubleArray make_double_array(std::vector<double> &&values) {
    auto *data = new std::vector<double>(std::move(values));
    nb::capsule owner(data, [](void *p) noexcept { delete static_cast<std::vector<double> *>(p); });
    return DoubleArray(data->data(), {data->size()}, owner);
}

Int64Array make_int64_array(std::vector<int64_t> &&values) {
    auto *data = new std::vector<int64_t>(std::move(values));
    nb::capsule owner(data,
                      [](void *p) noexcept { delete static_cast<std::vector<int64_t> *>(p); });
    return Int64Array(data->data(), {data->size()}, owner);
}

}  // namespace

NB_MODULE(run_away_solver_ext, m) {
    nb::enum_<Method>(m, "Method")
        .value("COARSE_GRID", Method::COARSE_GRID)
        .value("EXACT", Method::EXACT)
        .value("BRANCH_AND_BOUND", Method::BRANCH_AND_BOUND)
        .value("BRUTE_FORCE", Method::BRUTE_FORCE);

    m.def(
        "run_batch",
        [](double field_w, double field_h,
           nb::ndarray<const double, nb::shape<-1, 2>, nb::c_contig, nb::device::cpu> opponents,
           nb::ndarray<const int64_t, nb::ndim<1>, nb::c_contig, nb::device::cpu> counts,
           Method method, double parameter, int repeats) {
            if (repeats < 1) throw std::invalid_argument("repeats must be >= 1");
            const size_t n_ticks = counts.shape(0);
            const size_t n_opponents = opponents.shape(0);

            std::vector<double> center_x(n_ticks);
            std::vector<double> center_y(n_ticks);
            std::vector<double> radius(n_ticks);
            std::vector<int64_t> evaluations(n_ticks);
            std::vector<int64_t> elapsed_ns(n_ticks);

            {
                nb::gil_scoped_release release;
                auto_battlebot::Size field;
                field.x = field_w;
                field.y = field_h;
                std::vector<auto_battlebot::Pose2D> tick_opponents;
                tick_opponents.reserve(8);

                size_t offset = 0;
                for (size_t t = 0; t < n_ticks; ++t) {
                    const auto count = static_cast<size_t>(counts(t));
                    if (offset + count > n_opponents) {
                        throw std::invalid_argument("counts sum past the opponents array");
                    }
                    tick_opponents.clear();
                    for (size_t i = 0; i < count; ++i) {
                        auto_battlebot::Pose2D pose;
                        pose.x = opponents(offset + i, 0);
                        pose.y = opponents(offset + i, 1);
                        tick_opponents.push_back(pose);
                    }
                    offset += count;

                    auto_battlebot::EmptyCircle result;
                    const auto start = std::chrono::steady_clock::now();
                    for (int r = 0; r < repeats; ++r) {
                        switch (method) {
                            case Method::COARSE_GRID:
                                result = auto_battlebot::solve_coarse_grid(field, tick_opponents,
                                                                           parameter);
                                break;
                            case Method::EXACT:
                                result = auto_battlebot::solve_exact(field, tick_opponents);
                                break;
                            case Method::BRANCH_AND_BOUND:
                                result = auto_battlebot::solve_branch_and_bound(
                                    field, tick_opponents, parameter);
                                break;
                            case Method::BRUTE_FORCE:
                                result = auto_battlebot::solve_brute_force(field, tick_opponents,
                                                                           parameter);
                                break;
                        }
                    }
                    const auto stop = std::chrono::steady_clock::now();

                    center_x[t] = result.center.x;
                    center_y[t] = result.center.y;
                    radius[t] = result.radius;
                    evaluations[t] = result.evaluations;
                    elapsed_ns[t] =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count() /
                        repeats;
                }
                if (offset != n_opponents) {
                    throw std::invalid_argument("counts do not consume the full opponents array");
                }
            }

            nb::dict result;
            result["center_x"] = make_double_array(std::move(center_x));
            result["center_y"] = make_double_array(std::move(center_y));
            result["radius"] = make_double_array(std::move(radius));
            result["evaluations"] = make_int64_array(std::move(evaluations));
            result["elapsed_ns"] = make_int64_array(std::move(elapsed_ns));
            return result;
        },
        "field_w"_a, "field_h"_a, "opponents"_a, "counts"_a, "method"_a, "parameter"_a,
        "repeats"_a = 1,
        "Run one solver over a whole trace. opponents is a flat (M, 2) array; counts gives "
        "the per-tick opponent count and must sum to M. parameter is grid resolution (m) for "
        "COARSE_GRID/BRUTE_FORCE, tolerance (m) for BRANCH_AND_BOUND, ignored for EXACT. "
        "elapsed_ns is per solve, averaged over `repeats` back-to-back calls.");
}
