#include "PclStuff.h"

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;


Cloud::Ptr PclStuff::Downsample(const Cloud::ConstPtr &cloud_in,
                                float leaf_size) {
  Cloud::Ptr cloud_result(new Cloud());

  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  grid.setInputCloud(cloud_in);
  grid.filter(*cloud_result);

  return cloud_result;
}

Cloud::Ptr
PclStuff::GroundRemover(Cloud::ConstPtr cloud_in, float treshold) {
  Cloud::Ptr cloud_groundless(new Cloud);

  pcl::SACSegmentation<Point> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(treshold);
  seg.setInputCloud(cloud_in);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    std::cout
      << "Could not estimate a planar model for the given dataset."
      << std::endl;
    *cloud_groundless = *cloud_in;
    return cloud_groundless;
  }

  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_groundless);
  return cloud_groundless;
}

std::tuple<Cloud::Ptr, Cloud::Ptr, std::vector<float>, std::vector<float>, std::vector<float>>
PclStuff::MiniClusterer(Cloud::Ptr cloud_in,
                        float tolerance, int points_min, int points_max,
                        float x_max, float y_max, float z_max,
                        float x_min, float y_min, float z_min) {
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(points_min);
  ec.setMaxClusterSize(points_max);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_in);
  ec.extract(cluster_indices);

  Cloud::Ptr cloud_cluster(new Cloud);
  cloud_cluster->is_dense = true;
  cloud_cluster->width = 0;
  cloud_cluster->height = 1;
  Cloud::Ptr centroids(new Cloud);
  std::vector<float> vector_length_x;
  std::vector<float> vector_length_y;
  std::vector<float> vector_length_z;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it) {


    pcl::PointXYZ loww(std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max());
    pcl::PointXYZ high(std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest());


    pcl::PointXYZ sum(0, 0, 0);
    int cur_cluster_size = 0;
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      Point colorful_point(j * 0.05);
      colorful_point.x = cloud_in->points[*pit].x;
      colorful_point.y = cloud_in->points[*pit].y;
      colorful_point.z = cloud_in->points[*pit].z;


      if (cloud_in->points[*pit].x < loww.x) {
        loww.x = cloud_in->points[*pit].x;
      }
      if (cloud_in->points[*pit].y < loww.y) {
        loww.y = cloud_in->points[*pit].y;
      }
      if (cloud_in->points[*pit].z < loww.z) {
        loww.z = cloud_in->points[*pit].z;
      }

      if (cloud_in->points[*pit].x > high.x) {
        high.x = cloud_in->points[*pit].x;
      }
      if (cloud_in->points[*pit].y > high.y) {
        high.y = cloud_in->points[*pit].y;
      }
      if (cloud_in->points[*pit].z > high.z) {
        high.z = cloud_in->points[*pit].z;
      }


      cloud_cluster->points.push_back(colorful_point);

      sum.x += cloud_in->points[*pit].x;
      sum.y += cloud_in->points[*pit].y;
      sum.z += cloud_in->points[*pit].z;
      cur_cluster_size++;
    }

    j++;
    pcl::PointXYZ size(fabsf(high.x - loww.x),
                       fabsf(high.y - loww.y),
                       fabsf(high.z - loww.z));
    Point centroid;
    centroid.x = sum.x / cur_cluster_size;
    centroid.y = sum.y / cur_cluster_size;
    centroid.z = sum.z / cur_cluster_size;

    if (size.x > x_max
        || size.x < x_min
        || size.y > y_max
        || size.y < y_min
        || size.z > z_max
        || size.z < z_min
      )
      continue;

    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      Point colorful_point;
      colorful_point.x = cloud_in->points[*pit].x;
      colorful_point.y = cloud_in->points[*pit].y;
      colorful_point.z = cloud_in->points[*pit].z;
    }

    centroids->points.push_back(centroid);
    vector_length_x.push_back(size.x);
    vector_length_y.push_back(size.y);
    vector_length_z.push_back(size.z);
  }
  cloud_cluster->width = cloud_cluster->points.size();

  return std::make_tuple(cloud_cluster, centroids,
                         vector_length_x, vector_length_y, vector_length_z);
}

