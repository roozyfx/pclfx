#pragma once
// Point Cloud Library
#include <memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointXYZ;
// typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT;

template<typename PT>
struct _PointCloud
{
    pcl::PointCloud<PT>::Ptr cloud{std::make_shared<pcl::PointCloud<PT>>()};
    std::string id{"cloud"};
};
typedef _PointCloud<PointT> PointCloud;

//@todo: make a better solution for coloring, e.g. user can change via UI
namespace Utils {
inline void xyz2xyzrgb(std::shared_ptr<const _PointCloud<PointXYZ>> in,
                       std::shared_ptr<_PointCloud<pcl::PointXYZRGB>> out,
                       const uint8_t r,
                       const uint8_t g,
                       const uint8_t b)
{
    out->id = in->id;
    for (auto idx = 0; idx < in->cloud->size(); ++idx) {
        pcl::PointXYZRGB temp;
        temp.x = (*in->cloud)[idx].x;
        temp.y = (*in->cloud)[idx].y;
        temp.z = (*in->cloud)[idx].z;
        temp.r = r;
        temp.g = g;
        temp.b = b;
        out->cloud->push_back(temp);
    }
}

} //namespace Utils
