#include "filter.h"
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

using std::cout, std::endl;

void Filter::outlierRemoval()
{
    cout << "Std Dev: " << _standardDeviation << endl;
    cout << "K Mean: " << _kMean << endl;
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<Point> sor;

    sor.setInputCloud(_pc->cloud);

    sor.setMeanK(_kMean);
    sor.setStddevMulThresh(_standardDeviation);

    sor.filter(*_sor_inliers->cloud);
    _sor_inliers->id = "sor_inliers";

    sor.setNegative(true);
    sor.filter(*_sor_outliers->cloud);
    _sor_outliers->id = "sor_outliers";
}

std::shared_ptr<PointCloud> Filter::voxelGridFilter()
{
    // Create the filtering object
    pcl::VoxelGrid<Point> vg;
    vg.setInputCloud(_pc->cloud);
    vg.setLeafSize(_vg_x, _vg_y, _vg_z);
    std::shared_ptr<PointCloud> vgResult{std::make_shared<PointCloud>()};
    vg.filter(*vgResult->cloud);
    vgResult->id = "Voxel Grid";

    return vgResult;
}
