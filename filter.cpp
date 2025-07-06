#include "filter.h"
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

using std::cout, std::endl;

void Filter::OutlierRemoval()
{
    cout << "Std Dev: " << _standard_deviation << endl;
    cout << "K Mean: " << _k_mean << endl;
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<Point> sor;

    sor.setInputCloud(_pc->cloud);

    sor.setMeanK(_k_mean);
    sor.setStddevMulThresh(_standard_deviation);

    sor.filter(*_sor_inliers->cloud);
    _sor_inliers->id = "sor_inliers";

    sor.setNegative(true);
    sor.filter(*_sor_outliers->cloud);
    _sor_outliers->id = "sor_outliers";
}

std::shared_ptr<PointCloud> Filter::VoxelGridFilter()
{
    // Create the filtering object
    pcl::VoxelGrid<Point> vg;
    vg.setInputCloud(_pc->cloud);
    vg.setLeafSize(_vg_x, _vg_y, _vg_z);
    std::shared_ptr<PointCloud> vg_result{std::make_shared<PointCloud>()};
    vg.filter(*vg_result->cloud);
    vg_result->id = "Voxel Grid";

    return vg_result;
}
