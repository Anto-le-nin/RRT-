CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17

INCLUDES = -I./externals//kd_tree/kdtree/include

TARGET = main
SRC = main.cpp RRT.cpp RRT_visualizer.cpp

all:
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET)
