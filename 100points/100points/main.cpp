#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>
#include <queue>

struct Point 
{
    double x, y;
};

struct Edge 
{
    int to;
    double cost;
};

using Graph = std::vector<std::vector<Edge>>;

double distance(const Point& a, const Point& b) 
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

Graph createGraph(const std::vector<Point>& points) 
{
    int n = points.size();
    Graph graph(n);

    for (int i = 0; i < n; ++i) 
    {
        std::vector<std::pair<double, int>> distances;

        for (int j = 0; j < n; ++j) 
        {
            if (i != j) 
            {
                distances.emplace_back(distance(points[i], points[j]), j);
            }
        }

        std::sort(distances.begin(), distances.end());

        int connections = std::min(6, static_cast<int>(distances.size()));
        for (int k = 0; k < connections; ++k) 
        {
            graph[i].push_back({ distances[k].second, distances[k].first });
        }
    }

    return graph;
}

std::vector<double> dijkstra(const Graph& graph, int start)
{
    size_t n = graph.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    dist[start] = 0;

    using PII = std::pair<double, int>;
    std::priority_queue<PII, std::vector<PII>, std::greater<PII>> pq;
    pq.push({ 0, start });

    while (!pq.empty()) 
    {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u]) continue;

        for (const auto& edge : graph[u]) 
        {
            int v = edge.to;
            double new_dist = dist[u] + edge.cost;

            if (new_dist < dist[v]) 
            {
                dist[v] = new_dist;
                pq.push({ new_dist, v });
            }
        }
    }

    return dist;
}

int main() {
    setlocale(0, "");
    const int num_points = 100;
    const double M_PI = 3.14;
    const double radius = 10.0;
    std::vector<Point> points(num_points);

    std::default_random_engine gen(std::random_device{}());;
    std::uniform_real_distribution<> dis(0.0, radius);

    for (int i = 0; i < num_points; ++i) 
    {
        double angle = dis(gen) * 2 * M_PI;
        double r = dis(gen);
        points[i] = { r * cos(angle), r * sin(angle) };
    }

    Graph graph = createGraph(points);

    int destination;
    std::cout << "Введите номер точки назначения (0-" << num_points - 1 << "): ";
    std::cin >> destination;

    std::vector<double> costs = dijkstra(graph, 0);

    std::cout << "Минимальная стоимость пути до точки " << destination << ": "
        << costs[destination] * 10 << " USD" << std::endl;

    return 0;
}

//1 - Генерация случайных точек внутри окружности
//2 - Создание графа, где каждая точка соединена с соседними(от 2 до 6 соединений)
//3 - Решение написаное с помощью алгоритма Дейкстры для нахождения минимального пути от начальной точки до конечной.
//4 - Вывод стоимости пути.