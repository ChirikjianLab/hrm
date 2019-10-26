#include "include/DistanceMetric.h"

double vectorEuclidean(const std::vector<double>& v1,
                       const std::vector<double>& v2) {
    std::vector<double> diff;
    for (std::size_t i = 0; i < v1.size(); ++i) {
        diff.push_back(v1[i] - v2[i]);
    }
    return std::sqrt(
        std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0));
}
