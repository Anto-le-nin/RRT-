#include "RRT.hpp"
#include "RRT_visualizer.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <array>
#include <string>

#define ITERATIONS 15000

static double ms_since(const std::chrono::high_resolution_clock::time_point& t0,
    const std::chrono::high_resolution_clock::time_point& t1) {
return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

void benchmark(std::vector<std::vector<int>> map){
    std::array<int,2> q_initial = { {int(map[0].size()/2), int(map.size()/2)}};
    std::array<int,2> q_final = { {int(map[0].size() / 8), int(map[0].size() / 8)}};

    int step_max = 10;
    int iterations_max = 1500;

    std::vector<int> iterations = {1000, 2000, 3000, 5000, 10000, 20000};

    std::ofstream log("bench_rrt.txt", std::ios::out | std::ios::trunc);
    if (!log) {
        std::cerr << "Erreur: impossible d'ouvrir bench_rrt.txt\n";
        return;
    }

    // Petit header (format facile a parser ensuite)
    log << "iterations,run,kd_ms,nokd_ms\n";
    log << std::fixed << std::setprecision(3);

    const int runs = 5;

    for (size_t it_i = 0; it_i < iterations.size(); ++it_i) {
        int iters = iterations[it_i];

        double kd_sum = 0.0, nokd_sum = 0.0;

        std::cout << "\n=== Iterations = " << iters << " (" << (it_i + 1)
                  << "/" << iterations.size() << ") ===\n";

        for (int r = 1; r <= runs; ++r) {
            std::cout << "Run " << r << "/" << runs << " ... " << std::flush;

            // --- KD ---
            RRT tree(q_initial, q_final, map, step_max, iterations_max);
            auto t0 = std::chrono::high_resolution_clock::now();
            tree.Benchmark(iters);
            auto t1 = std::chrono::high_resolution_clock::now();
            double kd_ms = ms_since(t0, t1);

            // --- No KD ---
            RRT tree_noKD(q_initial, q_final, map, step_max, iterations_max);
            auto t2 = std::chrono::high_resolution_clock::now();
            tree_noKD.Benchmark_noKD(iters);
            auto t3 = std::chrono::high_resolution_clock::now();
            double nokd_ms = ms_since(t2, t3);

            kd_sum += kd_ms;
            nokd_sum += nokd_ms;

            // log par run
            log << iters << "," << r << "," << kd_ms << "," << nokd_ms << "\n";
            log.flush();

            std::cout << "KD=" << kd_ms << " ms, NoKD=" << nokd_ms << " ms\n";
        }

        double kd_avg = kd_sum / runs;
        double nokd_avg = nokd_sum / runs;

        std::cout << "Moyenne sur " << runs << " runs : "
                  << "KD=" << kd_avg << " ms | "
                  << "NoKD=" << nokd_avg << " ms | "
                  << "speedup x" << (nokd_avg / kd_avg) << "\n";

        // Une ligne "resume" dans le fichier (optionnel)
        log << "AVG_" << iters << ",-,"
            << kd_avg << "," << nokd_avg << "\n";
        log.flush();
    }

    std::cout << "\nBench termine. Resultats dans bench_rrt.txt\n";
    return;
}

int main(int argc, char ** argv)
{

    (void)argc;
    (void)argv;

    //std::vector<std::vector<int>> map = load_map("map.txt");

    std::vector<std::vector<int>> map;

    for(int i = 0; i < 800; i++){
        std::vector<int> ligne_i;
        for(int j = 0; j < 800; j++){
            int zero = 0;
            ligne_i.push_back(zero);
        }
        map.push_back(ligne_i);
    }

    for(int i = 300; i < 330; i++){
        for(int j = 0; j < int(map[0].size()); j++){
                map[i][j] = 1;
        }
    }

    for(int i = 300; i < 330; i++){
        for(int j = 0; j < int(map[0].size()); j++){

            if(j > int(map[0].size() * 0.84) && j < int(map[0].size() * 0.85))
                map[i][j] = 0;
        }
    }
    //ajout d'un obstacle carré
    int square_pos_x = map[0].size() / 4;
    int square_pos_y = map.size() / 4;
    int dim_square   = map[0].size() / 10;

    int half = dim_square / 2;

    for (int y = square_pos_y - half; y < square_pos_y + half; y++) {
        for (int x = square_pos_x - half; x < square_pos_x + half; x++) {
            if (y >= 0 && y < int(map.size()) && x >= 0 && x < int(map[0].size())) {
            map[y][x] = 1;
            }
        }
    }

    std::cout<<"Map générée"<<std::endl;

    //debut programme principal

    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // KDTree<2> kd_tree;

    // kd_tree.insert({{1,1}}, 0);
    // kd_tree.insert({{4,1}}, 1);
    // kd_tree.insert({{0,0}}, 2);
    // kd_tree.insert({{4,3}}, 3);
    // kd_tree.insert({{6,6}}, 4);

    // std::cout<<"point le plus proche : "<<kd_tree.findNearest({{4,2}})<<std::endl;
    // std::vector x_near = kd_tree.findNear({{4,2}}, 100.0);
    // for(auto x : x_near){
    //     std::cout<<x<<std::endl;
    // }

    std::array<int,2> q_initial = { {int(map[0].size()/2), int(map.size()/2)}};
    std::array<int,2> q_final = { {int(map[0].size() / 8), int(map[0].size() / 8)}};

    int step_max = 8;
    int iterations_max = 30000;

    std::vector<std::array<int,2>> final_path;
    while(true){
        RRT tree(q_initial, q_final, map, step_max, iterations_max);
        final_path = tree.find_path(iterations_max);
        if(tree.map.checkCollisionPath(final_path)){
            saveRRTImage(map, tree, final_path, "rrt.ppm");
            break;
        }
    }

    //saveRRTImage(map, tree, std::nullopt, "benchmark_KD.ppm");
}

