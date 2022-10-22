#include "util.hpp"

Point mean(PointList points)
{
    Point mean;

    for (Point p : points)
    {
        mean.first += p.first;
        mean.second += p.second;
    }

    int n = points.size();
    mean.first = mean.first / n;
    mean.second = mean.second / n;
    return mean;
}
