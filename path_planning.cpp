#include "path_planning.h"

// Allocates memory for adjacency list
Graph::Graph(int V)
{
    this->V = V;
    adj = new vector<iPair>[V];
}

void Graph::addEdge(int u, int v, int w)
{
    adj[u].push_back(make_pair(v, w));
    adj[v].push_back(make_pair(u, w));
}

void Graph::printPath(int cur, vector<int> parents, vector<int>& path)
{
    if(cur == -1)
        return;
    printPath(parents[cur], parents, path);
    path.push_back(cur);
}

// Prints shortest paths from src to all other vertices
pair<int, vector<int>> Graph::Dijkstra(int src, int dest)
{
    // Create a priority queue to store vertices
    priority_queue<iPair, vector<iPair>, greater<iPair> > pq;
    pair<int, vector<int>> node;
    // Create a vector for distances and initialize all distances as infinite (INF)
    vector<int> dist(V, INT_MAX);
    // Initiate a vector to include all the node on the path
    vector<int> parents(V);
    vector<int> path;
    // Insert source itself in priority queue and initialize its distance as 0.
    pq.push(make_pair(0, src));
    dist[src] = 0;
    // Initiate the parents of the src node
    parents[src] = -1;
    /* Looping till priority queue becomes empty (or all
    distances are not finalized) */
    while (! pq.empty()) {
        // The first vertex in pair is the minimum distance vertex, extract it from priority queue.
        int u = pq.top().second;
        pq.pop();

        // 'i' is used to get all adjacent vertices of a
        // vertex
        for (auto i = adj[u].begin(); i != adj[u].end(); ++i) {
            // Get vertex label and weight of current
            // adjacent of u.
            int v = (*i).first;
            int weight = (*i).second;

            // If there is shorted path to v through u.
            if (dist[v] > dist[u] + weight) {
                // Updating distance of v
                dist[v] = dist[u] + weight;
                // Updating the path of the node
                parents[v] = u;
                pq.push(make_pair(dist[v], v));
            }
        }
    }
    // Print shortest distances stored in dist[]
    // cout << "Vertex\t\tDistance from Source" << endl;
    // for (int i = 0; i < V; ++i)
    //     cout << i << "\t\t\t" << dist[i]<<endl;
    printPath(dest, parents, path);
    // for (auto i = path.cbegin(); i != path.cend(); ++i) 
    //     cout << *i << " ";
    node.first = dist[dest];
    node.second = path;
    return node;
}

pair<vector<int>, vector<int>> Graph::optimalPath(int src, vector<int> dests)
{
    int start = src;
    int num = dests.size();
    vector<int> posts(num);
    vector<int> paths;
    pair<vector<int>, vector<int>> optimal_path;
    for(int i = 0; i < num; ++i)
    {
        int cur_dist = 10000;
        int cur_dest = 0;
        vector<int> cur_path;
        for(int j = 0; j < num - i; ++j)
        {
            pair<int, vector<int>> node = Dijkstra(start, dests[j]);
            if(node.first < cur_dist)
            {
                cur_dist = node.first;
                cur_dest = dests[j];
                cur_path.assign(node.second.begin(), node.second.end());
                cout<<"Dist"<<node.first<<"\n";
            }
        }
        cout<<"Dest"<<cur_dest<<"\n";
        posts[i] = cur_dest;
        // Delete the beginning of the other paths to avoid duplicate
        if(i != 0)
            cur_path.erase(cur_path.begin());
        paths.insert(paths.end(), cur_path.begin(), cur_path.end());
        // Update source 
        start = cur_dest;
        // Delete the current destination from the list
        auto it = find(dests.begin(), dests.end(), cur_dest);
        if(it != dests.end())
            dests.erase(it);
    }
    optimal_path.first = posts;
    optimal_path.second = paths;
    return optimal_path;
}

// Driver's code
int main()
{
    // create the graph given in above figure
    int V = 9;
    Graph g(V);

    // making above shown graph
    g.addEdge(0, 1, 4);
    g.addEdge(1, 0, 4);

    g.addEdge(0, 7, 8);
    g.addEdge(7, 0, 8);

    g.addEdge(1, 2, 8);
    g.addEdge(2, 1, 8);

    g.addEdge(1, 7, 11);
    g.addEdge(7, 1, 11);

    g.addEdge(2, 3, 7);
    g.addEdge(3, 2, 7);

    g.addEdge(2, 8, 2);
    g.addEdge(8, 2, 2);

    g.addEdge(2, 5, 4);
    g.addEdge(5, 2, 4);

    g.addEdge(3, 4, 9);
    g.addEdge(4, 3, 9);

    g.addEdge(3, 5, 14);
    g.addEdge(5, 3, 14);

    g.addEdge(4, 5, 10);
    g.addEdge(5, 4, 10);

    g.addEdge(5, 6, 2);
    g.addEdge(6, 5, 2);

    g.addEdge(6, 7, 1);
    g.addEdge(7, 6, 1);

    g.addEdge(6, 8, 6);
    g.addEdge(8, 6, 6);

    g.addEdge(7, 8, 7);
    g.addEdge(8, 7, 7);


    // Function call
    // pair<int, vector<int>> s_path = g.Dijkstra(0, 8);
    // cout<<"Dest: "<<s_path.first<<"\n";
    // for (auto i = s_path.second.cbegin(); i != s_path.second.cend(); ++i) 
    //     cout << *i << " ";
    vector<int> destinations{8, 4, 1};
    pair<vector<int>, vector<int>> opt_path = g.optimalPath(0 , destinations);
    for (auto i = opt_path.first.cbegin(); i != opt_path.first.cend(); ++i) 
        cout << *i << " ";
    cout<<endl;
    for (auto i = opt_path.second.cbegin(); i != opt_path.second.cend(); ++i) 
        cout << *i << " ";
    return 0;
}