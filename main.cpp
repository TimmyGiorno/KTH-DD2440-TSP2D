#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <random>
#include <chrono>
#include <limits>

using namespace std;

// Construct point.
// Example: 95.0129 61.5432.
struct Point {
    double x, y;
};

// Construct distance matrix.
// The distance is computed as the Euclidean distance 
// between the two points, rounded to the nearest integer.
class DistanceMatrix {
private:
    vector<vector<uint32_t>> distances;
    
public:
    DistanceMatrix(const vector<Point>& points) {
        int n = points.size();
        distances.resize(n, vector<uint32_t>(n));
        
        for (int i = 0; i < n; i++) {
            distances[i][i] = 0;
            for (int j = i + 1; j < n; j++) {
                uint32_t dist = calcDistance(points[i], points[j]);
                distances[i][j] = distances[j][i] = dist;
            }
        }
    }
    
    inline uint32_t getDistance(int i, int j) const {
        return distances[i][j];
    }

    int getSize() const {
        return distances.size();
    }
    
private:
    static uint32_t calcDistance(const Point& p1, const Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return round(sqrt(dx * dx + dy * dy));
    }
};


class KNearestNeighbors {
public:
    KNearestNeighbors(const DistanceMatrix& distances, int k) {
        int n = distances.getSize();
        k = min(k, n - 1); 
        neighbors.resize(n, vector<uint16_t>(k + 1));

        for (int i = 0; i < n; i++) {
            vector<pair<uint32_t, uint16_t>> dist_pairs;
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    dist_pairs.emplace_back(distances.getDistance(i, j), j);
                }
            }
            sort(dist_pairs.begin(), dist_pairs.end());

            for (int j = 0; j < k; j++) {
                neighbors[i][j] = dist_pairs[j].second;
            }
            
            neighbors[i][k] = i;
        }
    }

    const vector<uint16_t>& operator[](int index) const {
        return neighbors[index];
    }

private:
    vector<vector<uint16_t>> neighbors;
};


// Calculate tour length.
// The tour is represented as a vector of node indices.
// The length is computed as the sum of the distances between 
// consecutive nodes in the tour.
uint64_t calculateTourLength(const vector<uint16_t>& tour, const DistanceMatrix& distances) {
    uint64_t length = 0;
    int n = tour.size();
    for (int i = 0; i < n; i++) {
        length += distances.getDistance(tour[i], tour[(i + 1) % n]);
    }
    return length;
}

// Greedy construction of initial tour.
vector<uint16_t> greedyConstruction(const vector<Point> points, const DistanceMatrix& distances) {
    int n = points.size();
    vector<uint16_t> tour(n);
    vector<bool> used(n, false);
    
    int start = 0;
    tour[0] = start;
    used[start] = true;
    
    for (int i = 1; i < n; i++) {
        int best_next = -1;
        int min_dist = numeric_limits<int>::max();
        
        for (int j = 0; j < n; j++) {
            if (!used[j]) {
                int dist = distances.getDistance(tour[i-1], j);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_next = j;
                }
            }
        }
        
        tour[i] = best_next;
        used[best_next] = true;
    }
    
    return tour;
}

// 2-Opt optimization.
void twoOpt(vector<uint16_t>& tour, const DistanceMatrix& distances, const KNearestNeighbors& neighbor) {
    int n = tour.size();
    bool locallyOptimal = false;

    while (!locallyOptimal) {
        locallyOptimal = true;

        for (int u_i = 0; u_i < n - 1; ++u_i) {
            uint16_t u = tour[u_i];
            uint16_t v = tour[u_i + 1];

            // Iterate over the neighbors of u
            for (int k = 0; k < neighbor[u].size(); ++k) {
                uint16_t w = neighbor[u][k];
                int w_i = find(tour.begin(), tour.end(), w) - tour.begin();
                if (w_i == u_i || w_i == (u_i + 1) % n) continue; // Skip adjacent edges

                uint16_t z = tour[(w_i + 1) % n];

                // Calculate the current and new distances
                uint32_t curr_dist = distances.getDistance(u, v) + distances.getDistance(w, z);
                uint32_t new_dist = distances.getDistance(u, w) + distances.getDistance(v, z);

                if (new_dist < curr_dist) {
                    // Perform the swap
                    reverse(tour.begin() + u_i + 1, tour.begin() + w_i + 1);
                    locallyOptimal = false;
                    break;
                }
            }
        }
    }
}

vector<uint16_t> solveTSP(const vector<Point>& points, double time_limit_seconds) {
    const auto start_time = chrono::steady_clock::now();
    
    DistanceMatrix distances(points);
    
    KNearestNeighbors kNeighbors(distances, 20);
    
    vector<uint16_t> tour = greedyConstruction(points, distances);
    
    bool improved = true;
    while (improved) {
        auto current_time = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::seconds>(current_time - start_time).count() 
            >= time_limit_seconds) {
            break;
        }
        
        vector<uint16_t> prev_tour = tour;
        
        twoOpt(tour, distances, kNeighbors);
        
        if (calculateTourLength(tour, distances) >= calculateTourLength(prev_tour, distances)) {
            tour = prev_tour; 
            improved = false; 
        }
    }
    
    return tour;
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(nullptr);
    
    int n;
    cin >> n;
    
    vector<Point> points(n);
    for (int i = 0; i < n; i++) {
        cin >> points[i].x >> points[i].y;
    }
    
    const double time_limit_seconds = 1.9;
    
    vector<uint16_t> tour = solveTSP(points, time_limit_seconds);
    
    for (int i = 0; i < n; i++) {
        cout << tour[i] << endl;
    }
    
    return 0;
}