// C++ Program to find Dijkstra's shortest path using
// priority_queue in STL
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
using namespace std;

// iPair ==> Integer Pair
typedef pair<int, int> iPair;

// This class represents a directed graph using
// adjacency list representation
class Graph {
    int V; // No. of vertices
    int direct = 0;     // Current direction
    int move = 0;       // MOve direction
    int cur_pos = 0;    // Current position of the AGV
    int next_pos = 0;   // Next position of the AGV  
    bool done = 0;
    bool intersect = 0;
    // In a weighted graph, we need to store vertex
    // and weight pair for every edge
    vector<iPair>* adj;
    
public:
    Graph(int V); // Constructor

    // function to add an edge to graph
    void addEdge(int u, int v, int w);
    // Function to print the path to the node
    void printPath(int cur, vector<int> parents, vector<int>& path);
    // prints shortest path from s
    pair<int, vector<int>> Dijkstra(int src, int dest);
    // Function to get the optimal shortest path between many destinations
    pair<vector<int>, vector<int>> optimalPath(int src, vector<int> dests);
    // Function for the navigation of the AGV
    void navigationAGV(void);
    void intersectionExecution(void);
};