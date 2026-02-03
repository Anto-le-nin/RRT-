#include "RRT.hpp"

void RRT::get_rayon_rrt(){

  int nb_case_obs = 0;

  for(int i = 0; i < map.h; i++){

    for(int j = 0; j < map.w; j++){

      if(map.m[i][j] == 1){

        nb_case_obs++;
      }
    }
  }
  if(nb_case_obs == 0){
    rayon_rrt = 30;
    return;
  }
  rayon_rrt = int(3.0*sqrt((float(map.h)*float(map.w))/float(nb_case_obs))) + 1;
  std::cout<<"rayon: "<<rayon_rrt<<std::endl;

  return;
}


std::vector<int> RRT::find_near(std::array<int,2> pos){
  
  std::vector<int> X_near; // va stocker les nodes dans le rayon rayon_rrt
    
  for (size_t i = 0; i < nodes.size(); ++i) {
    // calcul distance
    double dx = nodes[i].x - pos[0];
    double dy = nodes[i].y - pos[1];
    double d = sqrt(dx*dx + dy*dy);

    if (d < float(rayon_rrt)) {
      X_near.push_back(i);
    }
  }
  return X_near;
}

int RRT::find_nearest(std::array<int,2> pos) {
  double minDist = std::numeric_limits<double>::max(); // met la distance initiale minimum au maximum
  int nearest_node = 0; // va stocker la position du node dans la liste des nodes
  
  for (size_t i = 0; i < nodes.size(); ++i) {
    // calcul distance
    double dx = nodes[i].x - pos[0];
    double dy = nodes[i].y - pos[1];
    double d = dx*dx + dy*dy;

    if (d < minDist) {
        minDist = d;
        nearest_node = i;
    }
  }

  if(minDist > 1){
    return nearest_node;
  }
  else{
    return -1;
  }
}

//renvoie -1 si X_near vide
int RRT::find_min_cost(std::vector<int> X_near, std::array<int,2> pos) {

  double min_cost = std::numeric_limits<double>::max(); // met le cout initiale minimum au maximum
  int best_node = -1;
  int pos_x = pos[0]; int pos_y = pos[1];
  std::array<int,2> new_pos = { {pos_x, pos_y} };

  for(int i = 0; i < int(X_near.size()); i++) {
    double dx = nodes[X_near[i]].x - pos_x;
    double dy = nodes[X_near[i]].y - pos_y;
    double cost_to_node = sqrt(dx*dx + dy*dy);
    double cost_path = get_cost_path(X_near[i]);
    double total_cost = cost_path + cost_to_node;

    if(total_cost < min_cost && map.collisionFree(nodes[X_near[i]], new_pos)) {
      best_node = X_near[i];
      min_cost = total_cost;
    }
  }
  return best_node;
}

std::array<int,2> Map::randomPosition() {
  int y = std::rand() % h;
  int x = std::rand() % w;

  return {x, y};
}

bool Map::collisionFree(const RRTNode& start, const std::array<int,2>& final)
{
  if (final.size() < 2) return false;
  if (m.empty() || m[0].empty()) return false;

  auto inside = [&](int x, int y){
    return (x >= 0 && x < w && y >= 0 && y < h);
  };

  int x0 = int(std::round(start.x));
  int y0 = int(std::round(start.y));
  int x1 = int(std::round(final[0]));
  int y1 = int(std::round(final[1]));

  if (!inside(x0,y0) || !inside(x1,y1)) return false;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);

  int dir_x = (x0 < x1) ? 1 : -1;
  int dir_y = (y0 < y1) ? 1 : -1;

  int err = dx - dy;

  while (true) {
    if (!inside(x0,y0)) return false;
    if (m[y0][x0] == 1) return false;

    if (dx != 0 && dy != 0) {
      if (inside(x0 + dir_x, y0) && m[y0][x0 + dir_x] == 1) return false;
      if (inside(x0, y0 + dir_y) && m[y0 + dir_y][x0] == 1) return false;
    }

    if (x0 == x1 && y0 == y1)
    break;

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += dir_x;
    }
    if (e2 < dx) {
      err += dx;
      y0 += dir_y;
    }
  }

return true;
}

std::optional<std::array<int,2>> RRT::steer(RRTNode& node, std::array<int,2>& pos_des) {
  double dx = pos_des[0] - node.x;
  double dy = pos_des[1] - node.y;
  double dist = std::sqrt(dx*dx + dy*dy);

  if (dist == 0)
    return {{node.x, node.y}};

  double step = std::min(double(step_max), dist);

  int new_x = int(std::round(node.x + step * dx / dist));
  int new_y = int(std::round(node.y + step * dy / dist));
  std::array<int,2> new_pos = { {new_x, new_y} };

  if (!map.collisionFree(node, new_pos))
    return std::nullopt; //collision
  return new_pos;
}

bool RRT::near_end(std::array<int,2>& current_pos) {
  double dx = current_pos[0] - q_final[0];
  double dy = current_pos[1] - q_final[1];
  return std::sqrt(dx*dx + dy*dy) < step_max;
}

double RRT::get_cost_path(int goal_node) {

  double cost = 0.0;
  int cur = goal_node;

  while(nodes[cur].parent != -1) {
    int p = nodes[cur].parent;
    double dx = nodes[cur].x - nodes[p].x;
    double dy = nodes[cur].y - nodes[p].y;
    cost += std::sqrt(dx*dx + dy*dy);
    cur = p;
  }
  return cost;
}

std::vector<std::array<int,2>> RRT::extract_path(int goal_index_node){

  std::vector<std::array<int,2>> path;
  int parent = goal_index_node;

  while(parent != -1){
    RRTNode n = nodes[parent];
    path.push_back({int(n.x), int(n.y)});
    parent = n.parent;
  }

  //on ajoute le premier
  std::array<int,2> point = { {int(nodes[0].x), int(nodes[0].y)}};
  path.push_back(point);

  std::reverse(path.begin(), path.end());
  return path;
}

void RRT::rewire(const std::vector<int>& X_near, int ind_new_node)
{
  double cost_new = get_cost_path(ind_new_node);

  for(int idx : X_near){
    if(idx == 0) continue;
    if(idx == ind_new_node) continue;

    std::array<int,2> pos_idx = {int(nodes[idx].x), int(nodes[idx].y)};

    if(!map.collisionFree(nodes[ind_new_node], pos_idx))
      continue;

    double cost_idx = get_cost_path(idx);
    double dx = nodes[idx].x - nodes[ind_new_node].x;
    double dy = nodes[idx].y - nodes[ind_new_node].y;
    double cand = cost_new + std::sqrt(dx*dx + dy*dy);

    if(cand < cost_idx){
      nodes[idx].parent = ind_new_node;
    }
  }
}


bool Map::checkCollisionPath(const std::vector<std::array<int,2>>& path) {

  if (m.empty() || m[0].empty()) return false;
  if (path.size() < 2) return false;

  int H = (int)m.size();
  int W = (int)m[0].size();


  auto inside = [&](int x, int y){
    return (x >= 0 && x < W && y >= 0 && y < H);
  };

  // Vérifie les points eux-mêmes + obstacles
  for (const auto& p : path) {
    if (p.size() < 2) return false;
    int x = p[0], y = p[1];
    if (!inside(x, y)) return false;
    if (m[y][x] == 1) return false;  // point sur obstacle
  }

  // Vérifie chaque liaison consécutive
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    RRTNode start;
    start.x = path[i][0];
    start.y = path[i][1];

    std::array<int,2> goal = {path[i+1][0], path[i+1][1] };

    if (!collisionFree(start, goal)) {
      std::cerr << "Collision segment " << i
          << " : (" << path[i][0] << "," << path[i][1] << ") -> ("
          << path[i+1][0] << "," << path[i+1][1] << ")\n";

      return false;
    }
  }

  return true;
}

std::vector<std::vector<int>> load_map(std::string filename) {
  std::ifstream file;
  file.open(filename);

  std::vector<std::vector<int>> map;

  std::string line;
  while(std::getline(file, line)) {        
      std::istringstream line_stream(line);
      int case_;
      std::vector<int> line_map;

      while(line_stream >> case_) {
        line_map.push_back(case_);
      }

      map.push_back(line_map);
  }

  return map;
}

std::vector<std::array<int,2>> RRT::find_path(int iterations_max){

  std::vector<std::array<int,2>> final_path;

  for(int i = 0;  i < iterations_max; i++){

    std::array<int,2> q_rand = map.randomPosition();

    //int nearest_node = find_nearest(q_rand);
    int nearest_node = kd_tree.findNearest(q_rand);
    if(nearest_node == -1){ continue;}

    auto new_pos_opt = steer(nodes[nearest_node], q_rand);
    if(!new_pos_opt) continue; //obstacle
    auto new_pos = *new_pos_opt;

    std::vector<int> X_near = kd_tree.findNear(new_pos, rayon_rrt*rayon_rrt);
    int best_node = find_min_cost(X_near, new_pos);
    if (best_node == -1) best_node = nearest_node; //si X_near vide

    RRTNode new_node = {new_pos[0], new_pos[1], best_node};

    nodes.push_back(new_node);
    int ind_new_node = int(nodes.size()) - 1;
    kd_tree.insert(new_pos, ind_new_node);
  
    rewire(X_near, ind_new_node);

    if (near_end(new_pos) && map.collisionFree(new_node, q_final)){

        //std::vector<int> X_near_final = find_near(q_final);
        std::vector<int> X_near_final = kd_tree.findNear(q_final, rayon_rrt*rayon_rrt);
        int best_node_final = find_min_cost(X_near_final, q_final);
        if (best_node_final == -1) best_node_final = nearest_node; //si X_near vide

        RRTNode final_node = {q_final[0], q_final[1], best_node_final};

        nodes.push_back(final_node);
        int ind_final_node = int(nodes.size()) - 1;
        kd_tree.insert(q_final, ind_final_node);
        rewire(X_near_final, ind_final_node);

        // ---- smoothing / optimization: continue 1000 iters ----
        const int smooth_iters = 5000;
        std::cout<<"Path found, smoothing..."<<std::endl;

        for (int k = 0; k < smooth_iters; ++k) {
            std::array<int,2> q_rand2 = map.randomPosition();

            int nearest2 = kd_tree.findNearest(q_rand2);
            if (nearest2 == -1) continue;

            auto new_pos2_opt = steer(nodes[nearest2], q_rand2);
            if (!new_pos2_opt) continue;
            auto new_pos2 = *new_pos2_opt;

            auto X_near2 = kd_tree.findNear(new_pos2, rayon_rrt*rayon_rrt);
            int best2 = find_min_cost(X_near2, new_pos2);
            if (best2 == -1) best2 = nearest2;

            RRTNode n2 = {new_pos2[0], new_pos2[1], best2};
            nodes.push_back(n2);
            int ind2 = (int)nodes.size() - 1;
            kd_tree.insert(new_pos2, ind2);

            rewire(X_near2, ind2);

            //rewire le goal pour profiter des nouveaux noeuds
            auto X_goal = kd_tree.findNear(q_final, rayon_rrt*rayon_rrt);
            rewire(X_goal, ind_final_node);
        }
        
        final_path = extract_path(ind_final_node);
        return final_path;
    }
  }

  //recherche du noeud le plus proche
  int nn_reached = kd_tree.findNearest(q_final);
  double dx = nodes[nn_reached].x - q_final[0];
  double dy = nodes[nn_reached].y - q_final[1];
  double dist = std::sqrt(dx*dx + dy*dy);
  bool obstacle = map.collisionFree(nodes[nn_reached], q_final);
  std::cout << "Goal non atteint.\n";
  std::cout << "Nearest node distance = " << dist << " (" << dist / step_max << " x step_max)\n";
  if(obstacle){
    std::cout << "Q_final NON-atteignable en ligne droite (obstacle).\n";
  }else{
    std::cout << "Q_final atteignable en ligne droite.\n";
  }
  std::cout << "Augmentation des itérations nécessaire"<<std::endl;
  final_path = extract_path(nn_reached);
  return final_path;
}


void RRT::Benchmark(int Iterations){

  std::cout<<"Benchmarking"<<std::endl;

  for(int i = 0;  i < Iterations; i++){
    std::array<int,2> q_rand = map.randomPosition();

    //int nearest_node = find_nearest(q_rand);
    int nearest_node = kd_tree.findNearest(q_rand);
    if(nearest_node == -1){ continue;}

    auto new_pos_opt = steer(nodes[nearest_node], q_rand);
    if(!new_pos_opt) continue; //obstacle
    auto new_pos = *new_pos_opt;

    //std::vector<int> X_near = find_near(new_pos);
    std::vector<int> X_near = kd_tree.findNear(new_pos, rayon_rrt*rayon_rrt);
    int best_node = find_min_cost(X_near, new_pos);

    RRTNode new_node = {new_pos[0], new_pos[1], best_node};

    nodes.push_back(new_node);
    int ind_new_node = int(nodes.size()) - 1;
    kd_tree.insert(new_pos, ind_new_node);
  
    rewire(X_near, ind_new_node);
  }
  return;
}


void RRT::Benchmark_noKD(int Iterations){
  for(int i = 0;  i < Iterations; i++){

    std::array<int,2> q_rand = map.randomPosition();

    int nearest_node = find_nearest(q_rand);
    if(nearest_node == -1){ continue;}

    auto new_pos_opt = steer(nodes[nearest_node], q_rand);
    if(!new_pos_opt) continue; //obstacle
    auto new_pos = *new_pos_opt;

    std::vector<int> X_near = find_near(new_pos);
    int best_node = find_min_cost(X_near, new_pos);

    RRTNode new_node = {new_pos[0], new_pos[1], best_node};

    nodes.push_back(new_node);
    int ind_new_node = int(nodes.size()) - 1;

    rewire(X_near, ind_new_node);
  }
}