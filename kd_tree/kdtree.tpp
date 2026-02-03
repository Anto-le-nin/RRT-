#pragma once
#include <utility>
#include <limits>

namespace kdtree {

// Recursive function to insert a point into the KDTree
template<size_t K, class Scalar, class Data>
detail::Node<K, Scalar, Data>* KDTree<K, Scalar, Data>::insertRec(
    detail::Node<K, Scalar, Data>* node,
    const std::array<Scalar, K>& point,
    Data data,
    std::size_t depth)
    {
    // Base case: If node is null, create a new node
    if (!node) {
        return new detail::Node<K, Scalar, Data>(point, std::move(data));
    }

    // Calculate current dimension (cd)
    std::size_t cd = depth % K;

    // Compare point with current node and decide to go left or right
    if (point[cd] < node->point[cd])
        node->left = insertRec(node->left, point, std::move(data), depth + 1);
    else
        node->right = insertRec(node->right, point, std::move(data), depth + 1);

    return node;
}

// Public function to insert a point into the KDTree
template <size_t K, class Scalar, class Data>
void KDTree<K, Scalar, Data>::insert(const std::array<Scalar, K>& point, const Data& data) {
    root_ = insertRec(root_, point, data, 0);
    ++size_;
}

template <size_t K, class Scalar, class Data>
void KDTree<K, Scalar, Data>::nearestRec(
    detail::Node<K, Scalar, Data>* node,
    const std::array<Scalar, K>& target,
    std::size_t depth,
    detail::Node<K, Scalar, Data>*& best,
    Scalar& bestDist2) const
{
    if (!node) return;

    // 1) Mettre à jour le meilleur
    Scalar d2 = detail::squaredDistance<K, Scalar>(node->point, target);
    if (d2 < bestDist2) {
        bestDist2 = d2;
        best = node;
    }

    // 2) Choisir le côté "near" d'abord
    std::size_t cd = depth % (int)K;
    auto* nearChild = (target[cd] < node->point[cd]) ? node->left : node->right;
    auto* farChild  = (target[cd] < node->point[cd]) ? node->right : node->left;

    nearestRec(nearChild, target, depth + 1, best, bestDist2);

    // 3) Tester si on doit explorer l'autre côté
    Scalar diff = target[cd] - node->point[cd];

    if (diff * diff < bestDist2) {
        nearestRec(farChild, target, depth + 1, best, bestDist2);
    }
}

template <size_t K, class Scalar, class Data>
const Data* KDTree<K, Scalar, Data>::nearest(const std::array<Scalar, K>& target) const {
    if (!root) return nullptr;

    auto* bestNode = static_cast<detail::Node<K, Scalar, Data>*>(nullptr);
    Scalar bestDist2 = std::numeric_limits<Scalar>::infinity();

    nearestRec(root, target, 0, bestNode, bestDist2);

    return bestNode ? &bestNode->data : nullptr;
}

template <size_t K, class Scalar, class Data>
void KDTree<K, Scalar, Data>::radiusRec(
    detail::Node<K, Scalar, Data>* node,
    const std::array<Scalar, K>& target,
    std::size_t depth,
    Scalar radius2,
    std::vector<const Data*>& out) const
{
    if (!node) return;

    // 1) Mettre à jour le meilleur
    Scalar d2 = detail::squaredDistance<K, Scalar>(node->point, target);
    if (d2 < radius2) out.push_back(&node->data);

    // 2) Choisir le côté "near" d'abord
    std::size_t cd = depth % (int)K;
    auto* nearChild = (target[cd] < node->point[cd]) ? node->left : node->right;
    auto* farChild  = (target[cd] < node->point[cd]) ? node->right : node->left;

    radiusRec(nearChild, target, depth + 1, out, radius2);

    // 3) Tester si on doit explorer l'autre côté
    Scalar diff = target[cd] - node->point[cd];

    if (diff * diff < radius2) {
        radiusRec(farChild, target, depth + 1, out, radius2);
    }
}

template <size_t K, class Scalar, class Data>
std::vector<const Data*>  KDTree<K, Scalar, Data>::radiusSearch(const std::array<Scalar, K>& target, Scalar radius){
    std::vector<const Data*> out;
    if (!root_) return out;
    radiusRec(root, target, 0, radius*radius, out);
    return out;
}

template <size_t K, class Scalar, class Data>
bool KDTree<K, Scalar, Data>::empty() const noexcept { return size_ == 0; }

template <size_t K, class Scalar, class Data>
std::size_t KDTree<K, Scalar, Data>::size() const noexcept { return size_; }


// // Recursive function to search for a point in the KDTree
// bool KDTree::searchRecursive(Node* node, const std::array<int, K>& point, int depth) const {
//     // Base case: If node is null, the point is not found
//     if (node == nullptr) return false;

//     // If the current node matches the point, return true
//     if (node->point == point) return true;

//     // Calculate current dimension (cd)
//     int cd = depth % K;

//     // Compare point with current node and decide to go left or right
//     if (point[cd] < node->point[cd])
//         return searchRecursive(node->left, point, depth + 1);
//     else
//         return searchRecursive(node->right, point, depth + 1);
// }

// // Recursive function to print the KDTree
// template <size_t K, class Scalar, class Data>
// void KDTree<K, Scalar, Data>::printRecursive(Node* node, int depth) const {
//     // Base case: If node is null, return
//     if (node == nullptr) return;

//     // Print current node with indentation based on depth
//     for (int i = 0; i < depth; i++) std::cout << "  ";
//     std::cout << "(";
//     for (size_t i = 0; i < K; i++) {
//         std::cout << node->point[i];
//         if (i < K - 1) std::cout << ", ";
//     }
//     std::cout << ")" << std::endl;

//     // Recursively print left and right children
//     printRecursive(node->left, depth + 1);
//     printRecursive(node->right, depth + 1);
// }

// //return the distance^2 from a point 
// static double KDTree::dist2(const std::array<int, K>& a, const std::array<int, K>& b) {
//     double s = 0.0;
//     for (size_t i = 0; i < K; ++i) {
//         double d = double(a[i]) - double(b[i]);
//         s += d * d;
//     }
//     return s;
// }

// // Public function to search for a point in the KDTree
// bool KDTree::search(const std::array<int, K>& point) const {
//     return searchRecursive(root, point, 0);
// }

// // Public function to print the KDTree
// void KDTree::print() const {
//     printRecursive(root, 0);
// }



}