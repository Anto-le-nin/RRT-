void saveRRTImage(const std::vector<std::vector<int>>& map, const RRT& tree, const std::optional<std::vector<std::array<int,2>>>& path, const std::string& filename);
void drawLine(std::vector<std::vector<int>>& img, int x0, int y0, int x1, int y1, int value);
