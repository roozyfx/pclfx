#pragma once
#include "common.hpp"
#include <memory>

class Filter
{
public:
    Filter(std::shared_ptr<PointCloud> pointCloud);

    void outlierRemoval();
    std::shared_ptr<PointCloud> voxelGridFilter();

    void stdDev(const double stdDev) { _standardDeviation = stdDev; }
    void kMean(const uint kMean) { _kMean = kMean; }

    void vg_x(double x) { _vg_x = x; }
    void vg_y(double y) { _vg_y = y; }
    void vg_z(double z) { _vg_z = z; }

    std::shared_ptr<PointCloud> sorInliers() const { return _sor_inliers; }

    std::shared_ptr<PointCloud> sorOutliers() const { return _sor_outliers; }

private:
    std::shared_ptr<PointCloud> _pc;
    std::shared_ptr<PointCloud> _sor_inliers{std::make_shared<PointCloud>()};
    std::shared_ptr<PointCloud> _sor_outliers{std::make_shared<PointCloud>()};
    double _standardDeviation;
    uint _kMean;
    double _vg_x;
    double _vg_y;
    double _vg_z;
};
