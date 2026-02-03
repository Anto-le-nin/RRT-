#include <cstdio>
#include <vector>
#include "math.h"
#include <cstdlib>
#include <fstream>
#include <iostream>

#include "RRT.hpp" // pour l'objet RRT

#include "RRT_visualizer.hpp"
void drawLine(std::vector<std::vector<int>>& img, int x0, int y0, int x1, int y1, int value)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int dir_x = (x0 < x1) ? 1 : -1;
  int dir_y = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    if (x0 >= 0 && x0 < int(img[0].size()) && y0 >= 0 && y0 < int(img.size()))
      img[y0][x0] = value;

    if (x0 == x1 && y0 == y1) break;

    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += dir_x; }
    if (e2 <  dx) { err += dx; y0 += dir_y; }
  }
}

void saveRRTImage(const std::vector<std::vector<int>>& map, const RRT& tree, const std::optional<std::vector<std::array<int,2>>>& path, const std::string& filename)
{
  int h = map.size();
  int w = map[0].size();

  std::vector<std::vector<int>> img(h, std::vector<int>(w, 255));

  // obstacles
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      if (map[y][x] == 1)
          img[y][x] = 0;

  // arbre RRT
  for (int i = 1; i < int(tree.nodes.size()); i++) {
    const auto& n = tree.nodes[i];
    const auto& p = tree.nodes[n.parent];
    drawLine(img, int(p.x), int(p.y), int(n.x), int(n.y), 100);
  }

  // noeuds
  for (const auto& n : tree.nodes)
      img[int(n.y)][int(n.x)] = 200;

  // start
  img[int(tree.nodes[0].y)][int(tree.nodes[0].x)] = 150;

  // chemin final
  if (path && path->size() >= 2) {
    for (size_t i = 1; i < path->size(); i++) {
      drawLine(img,  (*path)[i-1][0], (*path)[i-1][1], (*path)[i][0], (*path)[i][1], 200);
    }
  }
  
  std::ofstream out(filename);
  out << "P3\n" << w << " " << h << "\n255\n";

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int v = img[y][x];
      if (v == 0)        out << "0 0 0 ";
      else if (v == 100) out << "0 0 255 ";
      else if (v == 200) out << "255 0 0 ";
      else if (v == 150) out << "0 255 0 ";
      else               out << "255 255 255 ";
    }
    out << "\n";
  }
}

