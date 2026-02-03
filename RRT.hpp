#include <cstdio>
#include <vector>
#include "math.h"
#include <cstdlib>
#include <ctime>
#include <limits>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <array>
#include <optional>

#include "kdtree/kdtree.hpp"

#define STEP_MAX 10

struct RRTNode {
    int x;
    int y;
    int parent;
};

class Map {
    public :

        Map(std::vector<std::vector<int>> m) : m(m){
            h = m.size();
            w  = m[0].size();
        }

        int h;
        int w;
        std::vector<std::vector<int>> m;

        std::array<int,2> randomPosition();
        bool collisionFree(const RRTNode& start, const std::array<int,2>& final);
        bool checkCollisionPath(const std::vector<std::array<int,2>>& path);
};

class RRT {
    public:
        std::vector<RRTNode> nodes;

        std::array<int,2> q_initial;
        std::array<int,2> q_final;

        Map map;

        int step_max;
        int iterations_max;
        int rayon_rrt;

        kdtree::KDTree<2, double, int> kd_tree;

        RRT(std::array<int,2> q_initial, std::array<int,2> q_final, 
            const std::vector<std::vector<int>>& m, int step_max, int iterations_max) : 
            q_initial(q_initial), q_final(q_final), map(m), 
            step_max(step_max), iterations_max(iterations_max), kd_tree() {
            nodes.push_back({q_initial[0], q_initial[1], -1}); // -1 pour la racine
            kd_tree.insert(conv_double(q_initial), 0);
            get_rayon_rrt();
        }

        // helper to convert in double for the KDtree
        static std::array<double,2> conv_double(const std::array<int,2>& p) {
            return {static_cast<double>(p[0]), static_cast<double>(p[1])};
        }

        int nearest(const std::array<int,2>& pos) const;
        std::vector<int> radiusnear(const std::array<int,2>& pos, double radius) const;

        int find_min_cost(std::vector<int> X_near, std::array<int,2> pos);
        std::optional<std::array<int,2>> steer(RRTNode& node, std::array<int,2>& pos_des);
        void rewire(const std::vector<int>& X_near, int ind_new_node);
        double get_cost_path(int goal_node);
        bool near_end(std::array<int,2>& current_pos);
        std::vector<std::array<int,2>> extract_path(int goal_index_node);
        void get_rayon_rrt();
        void Benchmark(int Iterations);

        void Benchmark_noKD(int Iterations);
        int find_nearest(std::array<int,2> pos);
        std::vector<int> find_near(std::array<int,2> pos);

        std::vector<std::array<int,2>> find_path(int iterations_max);
};

std::vector<std::vector<int>> load_map(std::string filename);
