#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <map>
#include <algorithm>
#include <climits>
#include <string>
#include <set>
#include <iomanip>
#include <float.h> 

using namespace std;

struct City {
    string name;
    vector<pair<string, int>> neighbors;
    int terminal_delay;
};

class Graph {
private:
    map<string, City> cities;
    bool isGraph1;

public:
    Graph(bool isGraph1) : isGraph1(isGraph1) {}

    void addCity(const string& name) {
        cities[name] = { name, {}, isGraph1 ? 20 : 60 };
    }

    void addEdge(const string& city1, const string& city2, int distance) {
        cities[city1].neighbors.push_back(make_pair(city2, distance));
        cities[city2].neighbors.push_back(make_pair(city1, distance));
    }

    pair<double, int> calculateTimeAndDistance(const string& from, const string& to, const string& start_city, bool include_delay = true) {
        int distance = 0;
        for (const auto& neighbor : cities[from].neighbors) {
            if (neighbor.first == to) {
                distance = neighbor.second;
                break;
            }
        }

        double travel_time = distance / 100.0;
        double delay_time = (include_delay && from != start_city) ? cities[from].terminal_delay / 60.0 : 0;

        return make_pair(travel_time + delay_time, distance);
    }

    pair<vector<string>, pair<double, int>> bfs(const string& start, const string& end) {
        map<string, bool> visited;
        map<string, string> parent;
        queue<string> q;

        q.push(start);
        visited[start] = true;

        while (!q.empty()) {
            string current = q.front();
            q.pop();

            if (current == end) break;

            for (const auto& neighbor : cities[current].neighbors) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    parent[neighbor.first] = current;
                    q.push(neighbor.first);
                }
            }
        }

        vector<string> path;
        double total_time = 0.0;
        int total_distance = 0;
        if (visited[end]) {
            string current = end;
            while (current != start) {
                path.push_back(current);
                pair<double, int> metrics = calculateTimeAndDistance(parent[current], current, start);
                total_time += metrics.first;
                total_distance += metrics.second;
                current = parent[current];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
        }

        return make_pair(path, make_pair(total_time, total_distance));
    }

    pair<vector<string>, pair<double, int>> dfs(const string& start, const string& end) {
        map<string, bool> visited;
        map<string, string> parent;
        stack<string> s;

        s.push(start);
        visited[start] = true;

        while (!s.empty()) {
            string current = s.top();
            s.pop();

            if (current == end) break;

            for (const auto& neighbor : cities[current].neighbors) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    parent[neighbor.first] = current;
                    s.push(neighbor.first);
                }
            }
        }

        vector<string> path;
        double total_time = 0.0;
        int total_distance = 0;
        if (visited[end]) {
            string current = end;
            while (current != start) {
                path.push_back(current);
                pair<double, int> metrics = calculateTimeAndDistance(parent[current], current, start);
                total_time += metrics.first;
                total_distance += metrics.second;
                current = parent[current];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
        }

        return make_pair(path, make_pair(total_time, total_distance));
    }

    pair<vector<string>, pair<double, int>> findShortestPath(const string& start, const string& end) {
        map<string, double> dist;
        map<string, int> total_dist;
        map<string, string> parent;
        set<pair<double, string>> pq;

        for (const auto& city : cities) {
            dist[city.first] = DBL_MAX;
            total_dist[city.first] = 0;
        }

        dist[start] = 0.0;
        pq.insert(make_pair(0.0, start));

        while (!pq.empty()) {
            pair<double, string> current = *pq.begin();
            pq.erase(pq.begin());
            double curr_dist = current.first;
            string curr_city = current.second;

            for (const auto& neighbor : cities[curr_city].neighbors) {
                string next_city = neighbor.first;
                int weight = neighbor.second;
                double time = weight / 100.0;
                double delay = (curr_city != start) ? cities[curr_city].terminal_delay / 60.0 : 0;
                if (curr_dist + time + delay < dist[next_city]) {
                    pq.erase(make_pair(dist[next_city], next_city));
                    dist[next_city] = curr_dist + time + delay;
                    total_dist[next_city] = total_dist[curr_city] + weight;
                    parent[next_city] = curr_city;
                    pq.insert(make_pair(dist[next_city], next_city));
                }
            }
        }

        vector<string> path;
        string current = end;
        while (current != start && parent.find(current) != parent.end()) {
            path.push_back(current);
            current = parent[current];
        }
        if (current == start) {
            path.push_back(start);
            reverse(path.begin(), path.end());
        }
        else {
            path.clear();
        }

        return make_pair(path, make_pair(dist[end], total_dist[end]));
    }
};

Graph createGraph(bool isGraph1) {
    Graph graph(isGraph1);
    vector<string> cities = { "Tabriz", "Tehran", "Arak", "Isfahan", "Semnan", "Mashhad", "Zahedan", "Birjand", "Ahvaz" };

    for (const auto& city : cities) {
        graph.addCity(city);
    }

    graph.addEdge("Tabriz", "Tehran", 150);
    graph.addEdge("Tabriz", "Arak", 200);
    graph.addEdge("Tabriz", "Ahvaz", 1200);
    graph.addEdge("Arak", "Semnan", 600);
    graph.addEdge("Arak", "Isfahan", 200);
    graph.addEdge("Isfahan", "Tehran", 500);
    graph.addEdge("Isfahan", "Zahedan", 1000);
    graph.addEdge("Isfahan", "Ahvaz", 700);
    graph.addEdge("Tehran", "Semnan", 1200);
    graph.addEdge("Mashhad", "Birjand", 400);
    graph.addEdge("Mashhad", "Tehran", 1200);
    graph.addEdge("Mashhad", "Semnan", 700);
    graph.addEdge("Semnan", "Zahedan", 1200);
    graph.addEdge("Zahedan", "Birjand", 300);

    return graph;
}

void printPath(const vector<string>& path, const pair<double, int>& metrics) {
    for (size_t i = 0; i < path.size(); ++i) {
        cout << path[i];
        if (i != path.size() - 1) {
            cout << " -> ";
        }
    }
    cout << "\nDistance: " << metrics.second << " km, Time: " << fixed << setprecision(2) << metrics.first << " hours\n";
}

int main() {
    cout << "Graph Traversal Algorithms Project\n";
    cout << "=================================\n";

    int mapChoice;
    cout << "1. Map with 20 min terminal delay\n";
    cout << "2. Map with 60 min terminal delay\n";
    cout << "Select map (1 or 2): ";
    cin >> mapChoice;

    Graph graph = createGraph(mapChoice == 1);

    int option;
    cout << "\n1. Find shortest path\n";
    cout << "2. Find two different paths\n";
    cout << "3. Find path with waypoint\n";
    cout << "Select option: ";
    cin >> option;

    string start, end, waypoint;

    if (option == 1) {
        cout << "Start city: ";
        cin >> start;
        cout << "End city: ";
        cin >> end;

        pair<vector<string>, pair<double, int>> result = graph.findShortestPath(start, end);

        cout << "\nShortest path: ";
        printPath(result.first, result.second);
    }
    else if (option == 2) {
        cout << "Start city: ";
        cin >> start;
        cout << "End city: ";
        cin >> end;

        pair<vector<string>, pair<double, int>> bfs_result = graph.bfs(start, end);
        pair<vector<string>, pair<double, int>> dfs_result = graph.dfs(start, end);

        cout << "\nBFS path: ";
        printPath(bfs_result.first, bfs_result.second);

        cout << "\nDFS path: ";
        printPath(dfs_result.first, dfs_result.second);
    }
    else if (option == 3) {
        cout << "Start city: ";
        cin >> start;
        cout << "Waypoint city: ";
        cin >> waypoint;
        cout << "End city: ";
        cin >> end;

        pair<vector<string>, pair<double, int>> path1 = graph.bfs(start, waypoint);
        pair<vector<string>, pair<double, int>> path2 = graph.bfs(waypoint, end);

        vector<string> full_path = path1.first;
        full_path.insert(full_path.end(), path2.first.begin() + 1, path2.first.end());

        double total_time = path1.second.first + path2.second.first;
        int total_distance = path1.second.second + path2.second.second;

        cout << "\nPath with waypoint: ";
        printPath(full_path, make_pair(total_time, total_distance));
    }

    return 0;
}
