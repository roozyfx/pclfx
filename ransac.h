#pragma once
#include "common.hpp"

class Ransac
{
public:
    Ransac(std::shared_ptr<PointCloud> pointCloud);
    void ransacSegmentation();
    inline std::shared_ptr<_PointCloud<pcl::PointXYZRGB>> getResult() { return _result; }

private:
    std::shared_ptr<PointCloud> _pc;
    std::shared_ptr<_PointCloud<pcl::PointXYZRGB>> _result{
        std::make_shared<_PointCloud<pcl::PointXYZRGB>>()};
};
