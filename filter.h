#pragma once
#include "common.hpp"
#include <memory>

class Filter
{
public:
    Filter(std::shared_ptr<PointCloud> point_cloud)
        : _pc{point_cloud}
    {}
    Filter(std::shared_ptr<PointCloud> point_cloud,
           const double stdDev,
           const uint kMean,
           const double voxel_grid_x,
           const double voxel_grid_y,
           const double voxel_grid_z)
        : _pc{point_cloud}
        , _standard_deviation{stdDev}
        , _k_mean{kMean}
        , _vg_x{voxel_grid_x}
        , _vg_y{voxel_grid_y}
        , _vg_z{voxel_grid_z}
    {}
    Filter(std::shared_ptr<PointCloud> point_cloud,
           const double voxel_grid_x,
           const double voxel_grid_y,
           const double voxel_grid_z)
        : _pc{point_cloud}
        , _vg_x{voxel_grid_x}
        , _vg_y{voxel_grid_y}
        , _vg_z{voxel_grid_z}
    {}
    Filter(std::shared_ptr<PointCloud> point_cloud, const double std_dev, const uint k_mean)
        : _pc{point_cloud}
        , _standard_deviation{std_dev}
        , _k_mean{k_mean}
    {}

    void OutlierRemoval();
    std::shared_ptr<PointCloud> VoxelGridFilter();

    void StdDev(const double std_dev) { _standard_deviation = std_dev; }
    void KMean(const uint k_mean) { _k_mean = k_mean; }

    void Vg_x(double x) { _vg_x = x; }
    void Vg_y(double y) { _vg_y = y; }
    void Vg_z(double z) { _vg_z = z; }

    std::shared_ptr<PointCloud> SorInliers() const { return _sor_inliers; }

    std::shared_ptr<PointCloud> SorOutliers() const { return _sor_outliers; }

private:
    std::shared_ptr<PointCloud> _pc;
    std::shared_ptr<PointCloud> _sor_inliers{std::make_shared<PointCloud>()};
    std::shared_ptr<PointCloud> _sor_outliers{std::make_shared<PointCloud>()};
    double _standard_deviation;
    uint _k_mean;
    double _vg_x;
    double _vg_y;
    double _vg_z;
};
