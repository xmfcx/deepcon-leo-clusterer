#ifndef PCL_STUFF_H
#define PCL_STUFF_H

#include <iostream>
#include <vector>
#include <tuple>

#include <pcl/point_cloud.h>

//ground removing
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>

//euclidean clustering
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

//downsampling
#include <pcl/filters/voxel_grid.h>

class PclStuff {
public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  static Cloud::Ptr
  Downsample(const Cloud::ConstPtr &cloud_in, float leaf_size);

  static Cloud::Ptr
  GroundRemover(Cloud::ConstPtr cloud_in, float treshold = 0.2);

  static std::tuple<
    Cloud::Ptr,
    Cloud::Ptr,
    std::vector<float>,
    std::vector<float>,
    std::vector<float>>
  MiniClusterer(Cloud::Ptr cloud_in,
                float tolerance, int points_min, int points_max,
                float x_max, float y_max, float z_max,
                float x_min, float y_min, float z_min);

};


#endif //PCL_STUFF_H
