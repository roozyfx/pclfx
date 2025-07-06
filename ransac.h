#pragma once
#include "common.hpp"

class Ransac
{
public:
    Ransac(std::shared_ptr<PointCloud> point_cloud);
    void RansacSegmentation();
    inline std::shared_ptr<_PointCloud<pcl::PointXYZRGB>> GetResult() { return _result; }

private:
    std::shared_ptr<PointCloud> _pc;
    std::shared_ptr<_PointCloud<pcl::PointXYZRGB>> _result{
        std::make_shared<_PointCloud<pcl::PointXYZRGB>>()};
};
