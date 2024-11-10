#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace std;

struct Point {
    double x, y;
};

int calcDistance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return round(sqrt(dx * dx + dy * dy));
}

vector<int> greedyTSP(const vector<Point>& points) {
    int n = points.size();
    vector<int> tour(n);
    vector<bool> used(n, false);
    
    tour[0] = 0;
    used[0] = true;
    
    for (int i = 1; i < n; i++) {
        int best = -1;
        for (int j = 0; j < n; j++) {
            if (!used[j]) {
                if (best == -1 || 
                    calcDistance(points[tour[i-1]], points[j]) < 
                    calcDistance(points[tour[i-1]], points[best])) {
                    best = j;
                }
            }
        }
        tour[i] = best;
        used[best] = true;
    }
    
    return tour;
}

int main() {

    int n;
    cin >> n;
    
    vector<Point> points(n);
    for (int i = 0; i < n; i++) {
        cin >> points[i].x >> points[i].y;
    }
    
    vector<int> tour = greedyTSP(points);
    
    for (int i = 0; i < n; i++) {
        cout << tour[i] << endl;
    }
    
    return 0;
}