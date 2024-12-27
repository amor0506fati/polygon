#include <vector>
#include <math.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <stack>
#include <limits>
#include <chrono>

struct Node {
    double lon, lat; 
    std::vector<std::pair<Node*, double>> neighbors; 
};

struct Graph {
    std::vector<Node*> nodes; 
    std::unordered_map<std::string, Node*> node_map; 

    // Method to load the graph from a file
    void load_from_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Unable to open file " << filename << std::endl;
            return;
        }
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string node_data;
            std::getline(ss, node_data, ':');
            double lon1, lat1;
            parse_coordinates(node_data, lon1, lat1);
            std::string main_key = coordinates_to_key(lon1, lat1);
            Node* main_node = get_or_create_node(lon1, lat1);
            std::string neighbors_data;
            while (std::getline(ss, neighbors_data, ';')) {
                double lon2, lat2, weight;
                parse_neighbor(neighbors_data, lon2, lat2, weight);

                Node* neighbor_node = get_or_create_node(lon2, lat2);
                main_node->neighbors.emplace_back(neighbor_node, weight);
            }
        } file.close();
    }

 
    Node* find_closest_node(double lat, double lon) {
        double min_distance = std::numeric_limits<double>::max();
        Node* node_found = nullptr;

        for (auto node : nodes) {
            double distance = std::sqrt(std::pow(node->lat - lat, 2) + std::pow(node->lon - lon, 2));
            if (distance < min_distance) {
                node_found = node;
                min_distance = distance;
            }
        }

        return node_found;
    }


    std::vector<Node*> bfs(Node* start, Node* goal) {
        std::unordered_map<Node*, Node*> came_from;
        std::queue<Node*> frontier;
        std::vector<Node*> visited_nodes;
        frontier.push(start);
        came_from[start] = nullptr;

        while (!frontier.empty()) {
            Node* current = frontier.front();
            frontier.pop();

            visited_nodes.push_back(current);

            if (current == goal) break;

            for (auto& neighbor : current->neighbors) {
                if (came_from.find(neighbor.first) == came_from.end()) {
                    frontier.push(neighbor.first);
                    came_from[neighbor.first] = current;
                }
            }
        }

        //print_visited_nodes(visited_nodes);
        return reconstruct_path(came_from, start, goal);
    }

    
    std::vector<Node*> dfs(Node* start, Node* goal) {
        std::unordered_map<Node*, Node*> came_from;
        std::stack<Node*> frontier;
        std::vector<Node*> visited_nodes;

        frontier.push(start);
        came_from[start] = nullptr;
        while (!frontier.empty()) {
            Node* current = frontier.top();
            frontier.pop();

            visited_nodes.push_back(current);

            if (current == goal) break;

            for (auto& neighbor : current->neighbors) {
                if (came_from.find(neighbor.first) == came_from.end()) {
                    frontier.push(neighbor.first);
                    came_from[neighbor.first] = current;
                }
            }
        }

        //print_visited_nodes(visited_nodes);
        return reconstruct_path(came_from, start, goal);
    }

    std::vector<Node*> dijkstra(Node* start, Node* goal) {
        std::unordered_map<Node*, double> cost_so_far;
        std::unordered_map<Node*, Node*> came_from;
        std::priority_queue<std::pair<double, Node*>, std::vector<std::pair<double, Node*>>, std::greater<>> frontier;
        std::vector<Node*> visited_nodes;

        frontier.emplace(0.0, start);
        cost_so_far[start] = 0.0;
        came_from[start] = nullptr;
        while (!frontier.empty()) {
            Node* current = frontier.top().second;
            frontier.pop();
            visited_nodes.push_back(current);
            if (current == goal) break;
            for (auto& neighbor : current->neighbors) {
                double new_cost = cost_so_far[current] + neighbor.second;
                if (cost_so_far.find(neighbor.first) == cost_so_far.end() || new_cost < cost_so_far[neighbor.first]) {
                    cost_so_far[neighbor.first] = new_cost;
                    frontier.emplace(new_cost, neighbor.first);
                    came_from[neighbor.first] = current;
                }
            }
        }
        //print_visited_nodes(visited_nodes);
        return reconstruct_path(came_from, start, goal);
    }

 
    std::vector<Node*> a_star(Node* start, Node* goal) {
        std::unordered_map<Node*, double> cost_so_far;
        std::unordered_map<Node*, Node*> came_from;
        std::priority_queue<std::pair<double, Node*>, std::vector<std::pair<double, Node*>>, std::greater<>> frontier;
        std::vector<Node*> visited_nodes;

        frontier.emplace(0.0, start);
        cost_so_far[start] = 0.0;
        came_from[start] = nullptr;
        while (!frontier.empty()) {
            Node* current = frontier.top().second;
            frontier.pop();
            visited_nodes.push_back(current);
            if (current == goal) break;
            for (auto& neighbor : current->neighbors) {
                double new_cost = cost_so_far[current] + neighbor.second;
                double heuristic = heuristic_cost(neighbor.first, goal);
                double priority = new_cost + heuristic;
                if (cost_so_far.find(neighbor.first) == cost_so_far.end() || new_cost < cost_so_far[neighbor.first]) {
                    cost_so_far[neighbor.first] = new_cost;
                    frontier.emplace(priority, neighbor.first);
                    came_from[neighbor.first] = current;
                }
            }
        }

        //print_visited_nodes(visited_nodes);
        return reconstruct_path(came_from, start, goal);
    }

    // Helper function to measure the execution time of a function
    template <typename Func>
    void measure_execution_time(Func func, const std::string& label) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << label << ": " << elapsed.count() << " seconds" << std::endl;
    }

private:
    // Function to reconstruct the path from the start node to the goal node
    std::vector<Node*> reconstruct_path(std::unordered_map<Node*, Node*>& came_from, Node* start, Node* goal) {
        std::vector<Node*> path;
        Node* current = goal;

        while (current != nullptr) {
            path.push_back(current);
            current = came_from[current];
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    // Heuristic function for A* (Euclidean distance)
    double heuristic_cost(Node* a, Node* b) {
        return std::sqrt(std::pow(a->lat - b->lat, 2) + std::pow(a->lon - b->lon, 2));
    }

    void parse_coordinates(const std::string& data, double& lon, double& lat) {
        std::stringstream ss(data);
        std::string lon_str, lat_str;
        std::getline(ss, lon_str, ',');
        std::getline(ss, lat_str, ',');
        lon = std::stod(lon_str);
        lat = std::stod(lat_str);
    }
   
    void parse_neighbor(const std::string& data, double& lon, double& lat, double& weight) {
        std::stringstream ss(data);
        std::string lon_str, lat_str, weight_str;
        std::getline(ss, lon_str, ',');
        std::getline(ss, lat_str, ',');
        std::getline(ss, weight_str, ',');
        lon = std::stod(lon_str);
        lat = std::stod(lat_str);
        weight = std::stod(weight_str);
    }

 
    std::string coordinates_to_key(double lon, double lat) {
        return std::to_string(lon) + "," + std::to_string(lat);
    }

    Node* get_or_create_node(double lon, double lat) {
        std::string key = coordinates_to_key(lon, lat);
        if (node_map.find(key) == node_map.end()) {
            Node* new_node = new Node{ lon, lat, {} };
            nodes.push_back(new_node);
            node_map[key] = new_node;
        }
        return node_map[key];
    }

    //// Function to print visited nodes
    //void print_visited_nodes(const std::vector<Node*>& visited_nodes) {
    //    std::cout << "Visited nodes:" << std::endl;
    //    for (const auto& node : visited_nodes) {
    //        std::cout << "(" << node->lon << ", " << node->lat << ")" << ';';
    //    }
    //}
};

int main() {
    Graph graph;
    graph.load_from_file("C:\\Users\\admin\\Downloads\\spb_graph (1).txt");

    // Coordinates for finding the closest nodes
    double lat1 = 59.926778, lon1 = 30.337716; // Ломоносова.9 ИТМО
    double lat2 = 59.962661, lon2 = 30.32422; // Большая Монетная. 25 

    // Find the closest nodes to the given coordinates
    Node* start_node = graph.find_closest_node(lat1, lon1);
    Node* goal_node = graph.find_closest_node(lat2, lon2);

    if (start_node && goal_node) {
        std::cout << "Found closest nodes:" << std::endl;
        std::cout << "Start: (" << start_node->lon << ", " << start_node->lat << ")" << std::endl;
        std::cout << "Goal: (" << goal_node->lon << ", " << goal_node->lat << ")" << std::endl;

        // Measure and print the execution time of BFS, DFS, Dijkstra, and A* algorithms
        graph.measure_execution_time([&]() {
            auto path = graph.bfs(start_node, goal_node);
            std::cout << "BFS found path of length: " << path.size() << std::endl;
            }, "BFS");

        graph.measure_execution_time([&]() {
            auto path = graph.dfs(start_node, goal_node);
            std::cout << "DFS found path of length: " << path.size() << std::endl;
            }, "DFS");

        graph.measure_execution_time([&]() {
            auto path = graph.dijkstra(start_node, goal_node);
            std::cout << "Dijkstra found path of length: " << path.size() << std::endl;
            }, "Dijkstra");

        graph.measure_execution_time([&]() {
            auto path = graph.a_star(start_node, goal_node);
            std::cout << "A* found path of length: " << path.size() << std::endl;
            }, "A*");
    }
    else {
        std::cerr << "Error: Could not find the closest nodes" << std::endl;
    }

    return 0;
}

