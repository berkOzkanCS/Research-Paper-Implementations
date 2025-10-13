#define _USE_MATH_DEFINES

#include <iostream>
#include <fstream>
#include <sstream>     
#include <cmath>
#include <vector>
#include <cstdlib>     
#include <random>
#include <algorithm>
#include <limits>
#include <array>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

extern "C" {
    #include "Dubins-Curves/include/dubins.h"
}

const double INF = std::numeric_limits<double>::max();

struct State{
    double x, y, z, phi;
};

struct Threat {
    double x, y, z;
    double a; 
    double b;
    double c; 
};

using Graph = boost::adjacency_list<
    boost::listS,
    boost::vecS,
    boost::undirectedS,
    boost::no_property,
    boost::property<boost::edge_weight_t, double>
>;
using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
using EdgeWeightMap = boost::property_map<Graph, boost::edge_weight_t>::type;

std::vector<State> gen_wp(const State start, const int* bounds, const int num, const std::vector<Threat>& threats, const State& tgtCoords, const double THRESHOLD, const double R_MIN, const double MAX_GAMMA);
Graph gen_graph(const std::vector<State>& samples, double threshold, double Rmin, double max_gamma, const std::vector<Threat>& threats);
std::vector<State> fetch_waypoints(Vertex start_idx, Vertex goal_idx, const std::vector<Vertex>& predecessor_map, const std::vector<State>& all_samples);

std::vector<State> genSamples(const int* bounds, const int num, const std::vector<Threat>& threats);
std::vector<State> genGoalSamples(const State& tgtCoords, int num_goals, const int* bounds);
bool is_safe(const State& s, const std::vector<Threat>& threats);
bool is_path_safe(const State& q0, const State& q1, double Rmin, const std::vector<Threat>& threats);

void export_data_and_plot_with_python(
    const std::vector<Threat>& threats, 
    const std::vector<State>& all_samples, 
    const Graph& graph, 
    const std::vector<State>& final_path
);

int main() {
    State start{400.0, 400.0, 4000.0, 0.0};
    const int bounds_array[6] = {0,5000,0,5000,0,500};
    const int* bounds = bounds_array;
    const int num_samples = 2500;

    const int NUMBER_OF_THREATS = 25;
    std::vector<Threat> threats;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> coordinates(500.0, 4500.0);
    std::uniform_real_distribution<> axes_dist(300.0, 750.0);

    for (int i = 0; i < NUMBER_OF_THREATS; ++i) {
        Threat t;
        t.x = coordinates(gen);
        t.y = coordinates(gen);
        t.z = coordinates(gen);
        t.a = axes_dist(gen);
        t.b = axes_dist(gen);
        t.c = axes_dist(gen);
        threats.push_back(t);
    }

    State tgtCoords{4800.0, 4800.0, 4800.0, 0.0};
    const double R_MIN = 30.0;
    const double THRESHOLD = 1000.0;
    const double MAX_GAMMA = 15.0 * (M_PI / 180.0);

    std::vector<State> all_samples = genSamples(bounds, num_samples, threats);
    all_samples.insert(all_samples.begin(), start);

    std::vector<State> goal_samples = genGoalSamples(tgtCoords, 10, bounds);
    all_samples.insert(all_samples.end(), goal_samples.begin(), goal_samples.end());
    std::vector<State> goals = goal_samples;

    Graph final_graph = gen_graph(all_samples, THRESHOLD, R_MIN, MAX_GAMMA, threats);

    std::vector<State> path = gen_wp(start, bounds, num_samples, threats, tgtCoords, THRESHOLD, R_MIN, MAX_GAMMA);

    if (!path.empty()) {
        std::cout << "Path found with " << path.size() << " waypoints. Plotting with Matplotlib..." << std::endl;
    } else {
        std::cerr << "Path planning failed. Check constraints and sampling density." << std::endl;
    }
    export_data_and_plot_with_python(threats, all_samples, final_graph, path);

    return 0;
}


std::string format_data_for_python(
    const std::vector<Threat>& threats, 
    const std::vector<State>& all_samples, 
    const Graph& graph, 
    const std::vector<State>& final_path) {
    std::stringstream ss;
    
    ss << "THREATS:\n";
    for (const auto& t : threats) {
        ss << t.x << "," << t.y << "," << t.z << "," << t.a << "," << t.b << "," << t.c << "\n";
    }
    ss << "END_THREATS\n";

    ss << "EDGES:\n";
    boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei) {
        ss << boost::source(*ei, graph) << "," << boost::target(*ei, graph) << "\n";
    }
    ss << "END_EDGES\n";

    ss << "SAMPLES:\n";
    for (const auto& s : all_samples) {
        ss << s.x << "," << s.y << "," << s.z << "\n";
    }
    ss << "END_SAMPLES\n";

    ss << "PATH:\n";
    for (const auto& s : final_path) {
        ss << s.x << "," << s.y << "," << s.z << "\n";
    }
    ss << "END_PATH\n";
    
    return ss.str();
}

void export_data_and_plot_with_python(
    const std::vector<Threat>& threats, 
    const std::vector<State>& all_samples, 
    const Graph& graph, 
    const std::vector<State>& final_path) 
{
    std::string output_data = format_data_for_python(threats, all_samples, graph, final_path);
    std::ofstream outfile("prm_data.txt");
    outfile << output_data;
    outfile.close();

    int result = system("python3 plot_prm.py prm_data.txt"); 
    
    if (result != 0) {
        std::cerr << "WARNING: Python plotting script failed to execute. "
                  << "Check if 'python3 plot_prm.py' runs correctly, and if Matplotlib is installed." << std::endl;
    }
}


std::vector<State> gen_wp(const State start, const int* bounds, const int num, const std::vector<Threat>& threats, const State& tgtCoords, const double THRESHOLD, const double R_MIN, const double MAX_GAMMA) {
    std::vector<State> samples = genSamples(bounds, num, threats);
    samples.insert(samples.begin(), start);
    Vertex start_vertex_idx = 0;

    std::vector<State> goal_samples = genGoalSamples(tgtCoords, 10, bounds);
    Vertex first_goal_idx = samples.size();
    samples.insert(samples.end(), goal_samples.begin(), goal_samples.end());

    std::vector<Vertex> goal_indices;
    for (Vertex i = first_goal_idx; i < samples.size(); ++i) {
        goal_indices.push_back(i);
    }


    Graph graph = gen_graph(samples, THRESHOLD, R_MIN, MAX_GAMMA, threats);

//
    // EdgeWeightMap weight_map = boost::get(boost::edge_weight, graph);

    // std::cout << "Graph: " << boost::num_vertices(graph) << " vertices, "
    //           << boost::num_edges(graph) << " edges\n";

    // boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    // for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei) {
    //     Vertex u = boost::source(*ei, graph);
    //     Vertex v = boost::target(*ei, graph);
    //     double w = weight_map[*ei];
    //     std::cout << "Edge: " << u << " -> " << v << " [weight=" << w << "]\n";
    // }
//

    std::vector<double> distances(samples.size());
    std::vector<Vertex> predecessors(samples.size());
    
    std::fill(distances.begin(), distances.end(), INF);
    distances[start_vertex_idx] = 0;

    try {
        boost::dijkstra_shortest_paths(graph, start_vertex_idx,
            boost::distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, graph)))
            .predecessor_map(boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, graph)))
            .weight_map(boost::get(boost::edge_weight, graph))
        );
    } catch (const boost::bad_graph& e) {
        std::cerr << "Boost Graph Error in Dijkstra: " << e.what() << std::endl;
        return {};
    }

    Vertex closest_goal_idx = boost::graph_traits<Graph>::null_vertex();
    double min_dist = INF;

    for (Vertex goal_idx : goal_indices) {
        // std::cout << "dist[" << goal_idx << "]: " << distances[goal_idx] << "\n";
        if (distances[goal_idx] < min_dist) {
            min_dist = distances[goal_idx];
            closest_goal_idx = goal_idx;
        }
    }

    if (closest_goal_idx == boost::graph_traits<Graph>::null_vertex() || min_dist == INF) {
        std::cout << "returning in line 265\n";
        std::cout << min_dist;
        if (closest_goal_idx == boost::graph_traits<Graph>::null_vertex()) {
            std::cout << "fuck, its true!\n";
        } else {
            std::cout << "NOT true!\n";
        }
        return {};
    }

    std::cout << "Start vertex index: " << start_vertex_idx << "\n";
    std::cout << "Goal vertex index: " << closest_goal_idx << "\n";

    std::cout << "Predecessor map:\n";
    for (size_t i = 0; i < predecessors.size(); ++i) {
        std::cout << "Vertex " << i << " -> " << predecessors[i] << "\n";
    }
    std::cout << "Samples passed to fetch_waypoints:\n";
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& s = samples[i];
        std::cout << "Index " << i 
                << ": x=" << s.x 
                << ", y=" << s.y 
                << ", z=" << s.z 
                << ", phi=" << s.phi 
                << "\n";
    }
    return fetch_waypoints(start_vertex_idx, closest_goal_idx, predecessors, samples);
}

std::vector<State> fetch_waypoints(Vertex start_idx, Vertex goal_idx,
                                 const std::vector<Vertex>& predecessor_map,
                                 const std::vector<State>& all_samples)
{
    std::vector<State> waypoints;
    Vertex curr = goal_idx;

    while (curr != start_idx && curr != predecessor_map[curr]) {
        waypoints.push_back(all_samples[curr]);
        curr = predecessor_map[curr];
    }

    if (curr == start_idx) {
        waypoints.push_back(all_samples[start_idx]);
    }
    
    std::reverse(waypoints.begin(), waypoints.end());
    return waypoints;
}

Graph gen_graph(const std::vector<State>& samples, double threshold, double Rmin, double max_gamma, const std::vector<Threat>& threats) {

    Graph graph(samples.size());
    EdgeWeightMap weight_map = boost::get(boost::edge_weight, graph);
    int num_samples = samples.size();

    for (int i = 0; i < num_samples; ++i) {
        for (int j = i + 1; j < num_samples; ++j) {
            const State& wi = samples[i];
            const State& wj = samples[j];

            double start_state[3] = {wi.x, wi.y, wi.phi};
            double end_state[3] = {wj.x, wj.y, wj.phi};

            DubinsPath path;
            int ret = dubins_shortest_path(&path, start_state, end_state, Rmin);
            
            if (ret != EDUBOK) continue; 
            
            double path_len = dubins_path_length(&path);

            if (path_len < threshold && path_len > 2.0 * Rmin) {
                double delta_z = std::abs(wi.z - wj.z);
                double gamma = std::atan2(delta_z, path_len); 

                if (gamma <= max_gamma) {
                    if (is_path_safe(wi, wj, Rmin, threats)) {
                        
                        boost::graph_traits<Graph>::edge_descriptor e;
                        bool inserted;
                        
                        boost::tie(e, inserted) = boost::add_edge(i, j, graph);
                        weight_map[e] = path_len;
                    }
                }
            }
        }
    }
    return graph;
}

bool is_path_safe(const State& q0, const State& q1, double Rmin, const std::vector<Threat>& threats) {
    double start_state[3] = {q0.x, q0.y, q0.phi};
    double end_state[3] = {q1.x, q1.y, q1.phi};

    DubinsPath path;
    int ret = dubins_shortest_path(&path, start_state, end_state, Rmin);
    
    if (ret != EDUBOK) return false;

    double path_len = dubins_path_length(&path);

    const double step_size = 5.0; 
    int num_steps = static_cast<int>(path_len / step_size);

    for (int i = 0; i <= num_steps; ++i) {
        double t = i * step_size;
        double q[3];
        
        if (t > path_len) {
            t = path_len;
        }

        dubins_path_sample(&path, t, q);

        double z_interp = q0.z + (q1.z - q0.z) * (t / path_len);

        State current_state = {q[0], q[1], z_interp, q[2]};

        if (!is_safe(current_state, threats)) {
            return false;
        }
    }
    return true;
}

bool is_safe(const State& s, const std::vector<Threat>& threats) {
    for (const auto& t : threats) {
        double normalized_distance_sq = 
            pow((s.x - t.x) / t.a, 2) +
            pow((s.y - t.y) / t.b, 2) +
            pow((s.z - t.z) / t.c, 2);

        if (normalized_distance_sq < 1.0) {
            return false;
        }
    }
    return true;
}

std::vector<State> genSamples(const int* bounds, const int num, const std::vector<Threat>& threats) {
    std::vector<State> samples;
    std::random_device rd;  
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> coordinatesXY(100.0, 5000.0); 
    std::uniform_real_distribution<> coordinatesZ(50.0, 4500.0); 
    std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);

    for (int i = 0; i < num; i++) {
        State s;
        s.x = coordinatesXY(gen);
        s.y = coordinatesXY(gen);
        s.z = coordinatesZ(gen); 
        s.phi = dist_phi(gen);  
        
        if (is_safe(s, threats)) {
            std::cout << "x: " << s.x << ", y: " << s.y << ", z: " << s.z << "\n";
            samples.push_back(s);
        }
    }
    return samples;
} 

std::vector<State> genGoalSamples(const State& tgtCoords, int num_goals, const int* bounds) {
    std::vector<State> goals;
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> dist_x(tgtCoords.x - 100.0, tgtCoords.x + 100.0);
    std::uniform_real_distribution<> dist_y(tgtCoords.y - 100.0, tgtCoords.y + 100.0);
    std::uniform_real_distribution<> dist_z(bounds[4], bounds[5]);
    std::uniform_real_distribution<> dist_phi(-M_PI, M_PI);

    for (int i = 0; i < num_goals; ++i) {
        goals.push_back({
            dist_x(gen),
            dist_y(gen),
            dist_z(gen),
            dist_phi(gen)
        });
    }
    return goals;
}