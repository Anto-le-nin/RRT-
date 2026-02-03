#pragma once

#include <array>
#include <cstddef>

namespace kdtree::detail {

template <size_t K, class Scalar>
inline Scalar squaredDistance(
    const std::array<Scalar, K>& a,
    const std::array<Scalar, K>& b)
{
    Scalar sum{};
    for (std::size_t i = 0; i < K; ++i) {
        Scalar d = a[i] - b[i];
        sum += d * d;
    }
    return sum;
}

} // namespace kdtree::detail
