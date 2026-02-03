#pragma once

#include <array>
#include <vector>
#include <optional>
#include <cstddef>

#include "detail/node.hpp"
#include "detail/distance.hpp"

namespace kdtree {

template <size_t K, class Scalar, class Data>
class KDTree {
public:
    KDTree();
    ~KDTree();

    KDTree(const KDTree&) = delete;
    KDTree& operator=(const KDTree&) = delete;

    // --- Core API ---
    void insert(const std::array<Scalar, K>& point, const Data& data);
    void insert(const std::array<Scalar, K>& point, Data&& data);

    bool empty() const noexcept;
    std::size_t size() const noexcept;

    // Nearest neighbor
    const Data* nearest(const std::array<Scalar, K>& target) const;

    // Radius search (generic)
    std::vector<const Data*> radiusSearch(const std::array<Scalar, K>& target, Scalar radius
) const;

private:
    detail::Node<K, Scalar, Data>* root_ = nullptr;
    std::size_t size_ = 0;

    // Internal recursive helpers (déclarés seulement)
    detail::Node<K, Scalar, Data>* insertRec(
        detail::Node<K, Scalar, Data>* node,
        const std::array<Scalar, K>& point,
        Data data,
        std::size_t depth
    );

    void nearestRec(
        detail::Node<K, Scalar, Data>* node,
        const std::array<Scalar, K>& target,
        std::size_t depth,
        detail::Node<K, Scalar, Data>*& best,
        Scalar& bestDist2
    ) const;

    void radiusRec(
        detail::Node<K, Scalar, Data>* node,
        const std::array<Scalar, K>& target,
        std::size_t depth,
        Scalar radius2,
        std::vector<const Data*>& out
    ) const;

    void destroy(detail::Node<K, Scalar, Data>* node);
};

} // namespace kdtree

#include "kdtree.tpp"
