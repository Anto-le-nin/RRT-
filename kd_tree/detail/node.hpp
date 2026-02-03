#pragma once

#include <array>
#include <cstddef>
#include <utility>

namespace kdtree::detail {

template <size_t K, class Scalar, class Data>
struct Node {
    std::array<Scalar, K> point;
    Data data;

    Node* left  = nullptr;
    Node* right = nullptr;

    Node(const std::array<Scalar, K>& p, Data&& d)
        : point(p), data(std::move(d)) {}

    // optionnel mais pratique
    Node(const std::array<Scalar, K>& p, const Data& d)
        : point(p), data(d) {}
};

} // namespace kdtree::detail
