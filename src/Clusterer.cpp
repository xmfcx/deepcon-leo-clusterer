#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "PclStuff.h"
#include <ros/spinner.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Subscriber sub_velodyne_points;

ros::Publisher pub_cloud_raw;
ros::Publisher pub_cloud_downsampled;
ros::Publisher pub_cloud_groundless;
ros::Publisher pub_clusters;
ros::Publisher pub_cluster_centroids;
ros::Publisher pub_markers;

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

void VisualizeResult(const Cloud::Ptr &centroids,
                     const std::vector<float> &vector_length_x,
                     const std::vector<float> &vector_length_y,
                     const std::vector<float> &vector_length_z);


void
PublishCloud(const Cloud::ConstPtr &cloud_in, const ros::Publisher &publisher) {
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*cloud_in, msg_cloud);
  msg_cloud.header.stamp = ros::Time::now();
  msg_cloud.header.frame_id = "velodyne";
  publisher.publish(msg_cloud);
}

void LaserCloudCallBack(const sensor_msgs::PointCloud2ConstPtr &msg_cloud) {
  Cloud::Ptr cloud_in(new Cloud);
  pcl::fromROSMsg(*msg_cloud, *cloud_in);
  PublishCloud(cloud_in, pub_cloud_raw);

  Cloud::Ptr cloud_ds = PclStuff::Downsample(cloud_in, 0.1f);
  PublishCloud(cloud_ds, pub_cloud_downsampled);

  Cloud::Ptr cloud_groundless = PclStuff::GroundRemover(cloud_ds, 0.2f);
  PublishCloud(cloud_groundless, pub_cloud_groundless);

  Cloud::Ptr centroids;
  Cloud::Ptr cloud_cluster;
  std::vector<float> vec_lengths_x;
  std::vector<float> vec_lengths_y;
  std::vector<float> vec_lengths_z;
  std::tie(cloud_cluster, centroids,
           vec_lengths_x, vec_lengths_y, vec_lengths_z) =
    PclStuff::MiniClusterer(cloud_groundless, 0.8, 3, 8000,
                            0.8, 0.8, 1.2,
                            0.01, 0.01, 0.01);

  PublishCloud(cloud_cluster, pub_clusters);
  PublishCloud(centroids, pub_cluster_centroids);

  VisualizeResult(centroids,
                  vec_lengths_x, vec_lengths_y, vec_lengths_z);

}

void VisualizeResult(const Cloud::Ptr &centroids,
                     const std::vector<float> &vector_length_x,
                     const std::vector<float> &vector_length_y,
                     const std::vector<float> &vector_length_z) {
  visualization_msgs::MarkerArray marker_array = visualization_msgs::MarkerArray();

  for (int i = 0; i < 100; ++i) {
    visualization_msgs::Marker marker = visualization_msgs::Marker();
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "markerss" + std::to_string(i);
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    if (i >= centroids->points.size()) {
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.color.a = 0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker_array.markers.push_back(marker);
      continue;
    }

    marker.pose.position.x = centroids->points[i].x;
    marker.pose.position.y = centroids->points[i].y;
    marker.pose.position.z = centroids->points[i].z;

    marker.scale.x = vector_length_x[i];
    marker.scale.y = vector_length_y[i];
    marker.scale.z = vector_length_z[i];


    marker.color.a = 0.4; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    marker_array.markers.push_back(marker);
  }
  pub_markers.publish(marker_array);
}


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "clusterer_node");
  ros::NodeHandle nh;
  sub_velodyne_points = nh.subscribe("/velodyne_points", 1,
                                     &LaserCloudCallBack);

  pub_cloud_raw = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_raw", 1);
  pub_cloud_downsampled = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_downsampled", 1);
  pub_cloud_groundless = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_groundless", 1);
  pub_clusters = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_clusters", 1);
  pub_cluster_centroids = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_cluster_centroids", 1);
  pub_markers = nh.advertise<visualization_msgs::MarkerArray>
    ("/markers_detections", 1);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
