#include "ransac.h"
#include "common.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

Ransac::Ransac(std::shared_ptr<PointCloud> pointCloud)
    : _pc{pointCloud}
{}

void Ransac::ransacSegmentation()
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(_pc->cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1]
              << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

    pcl::ExtractIndices<PointT> extract;
    std::shared_ptr<PointCloud> extractedCloud{std::make_shared<PointCloud>()};

    extract.setInputCloud(_pc->cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // Extract only the selected points
    extract.filter(*extractedCloud->cloud);
    extractedCloud->id = "extractedCloud";

    Utils::xyz2xyzrgb(extractedCloud, _result, 40, 200, 50);
}
