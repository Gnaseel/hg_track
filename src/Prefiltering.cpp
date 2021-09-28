#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include "std_msgs/Float32.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include "cmath"
#include "typeinfo"
using namespace std;
using PointT = pcl::PointXYZI;

double minX=0;
double minY=0;
double minZ=0;
double maxX=0;
double maxY=0;
double maxZ=0;
double dist=0;

ros::Publisher filtered_pub, clustered_pub, cons_pub;
class Obs{
public:
  float minx=100;
  float maxx=-100;
  float miny=100;
  float maxy=-100;
  bool selected=false;
};
void cluster_callback(sensor_msgs::PointCloud2 *msg){
    //------------------------------------- Type Convert ------------------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg,*pCloud);
    pcl::PointCloud<pcl::PointXYZI>::iterator iter;

    int temp[150]={0};
    Obs obs[150];
    geometry_msgs::PoseArray poses;
    poses.header.frame_id = "map";
    geometry_msgs::PoseArray pub_poses;
    poses.header.frame_id = "map";
    geometry_msgs::PoseArray conPoses;
    conPoses.header.frame_id = "map";

    double mean_x=0;
    double mean_y=0;

    for(iter = pCloud->begin();iter!=pCloud->end();iter++){

      // int idx =int(iter->intensity); 
      // if (obs[idx].maxx < iter->x) obs[idx].maxx=iter->x; 
      // if (obs[idx].maxy < iter->y) obs[idx].maxy=iter->y; 
      // if (obs[idx].minx > iter->x) obs[idx].minx=iter->x; 
      // if (obs[idx].miny > iter->y) obs[idx].miny=iter->y;
      // obs[idx].selected=true;

      double dist = sqrt((iter->x)*(iter->x) + (iter->y)*(iter->y));
      if(dist>8.0) continue;

      if(temp[int(iter->intensity)] == 0){
        temp[int(iter->intensity)] = 1;

        geometry_msgs::Pose pose;
        pose.position.x=iter->x;
        pose.position.y=iter->y;
        mean_x += iter->x;
        mean_y += iter->y;
        poses.poses.push_back(pose);
      }
        // if(conlist.findIdx(iter->intensity) == -1){\
        //     Con newCon(iter->x, iter->y, 0.3, iter->intensity);
        //     conlist.addCon(newCon);
        // }
    }

    mean_x /= poses.poses.size()+0.0001;
    mean_y /= poses.poses.size()+0.0001;

    for(int i=0; i<poses.poses.size();i++){
      double mean_dist = sqrt((mean_x-poses.poses[i].position.x)*(mean_x-poses.poses[i].position.x) + (mean_y-poses.poses[i].position.y)*(mean_y-poses.poses[i].position.y));
      double dist = sqrt((poses.poses[i].position.x)*(poses.poses[i].position.x) + (poses.poses[i].position.y)*(poses.poses[i].position.y));

      if(mean_dist > 4 && dist > 5)continue;
      // if(dist<0.8 && poses.poses[i].position.x < -0.5 && abs(poses.poses[i].position.y) < 0.5) continue;
      pub_poses.poses.push_back(poses.poses[i]);
      
    }

    // print("SIZE {}".format(len(poses.poses)))
    // print("{} / {}".format(mean_x, mean_y))

    // for pose in poses.poses:
    //     dist = getDist(mean_x, mean_y, pose.position.x, pose.position.y)
    //     print("{} / {}   -------- {}".format(pose.position.x, pose.position.y, dist))
    //     if  dist < 5:
    //         pub_poses.poses.append(pose)


    cons_pub.publish(pub_poses);
}
pcl::PCLPointCloud2 cloud_cb(pcl::PCLPointCloud2ConstPtr input)
{
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    pcl::fromPCLPointCloud2(*input,*cloud_filtered);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ECE;
    ECE.setClusterTolerance(0.6); // m unit, please edit here
    ECE.setMinClusterSize(1); // 몇 개부터 한 군집?
    ECE.setMaxClusterSize(3000); // 몇 개까지 한 군집?
    ECE.setSearchMethod(tree);
    ECE.setInputCloud(cloud_filtered);
    ECE.extract(cluster_indices);
    pcl::PCLPointCloud2 output;
    int index = 0;
    pcl::PointCloud<PointT> TotalCloud;
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, index++)
    {
      int size = it->indices.size();
      if (size > 30) continue;
      if (size == 1) continue;
      PointT pt2;
      pcl::PointCloud<PointT> clster_Cloud;
      bool toggle = true;
      for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
        PointT pt = cloud_filtered->points[*pit];
        pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
        pt2.intensity = (float)(index);
        if (pt.z > -0.1){
          toggle=false;
          break;
        }
        clster_Cloud.push_back(pt2);
      }
      if(toggle) TotalCloud+=clster_Cloud;
      // cout<<"SIZE = "<<size<<endl;
      // cout<<"DIST = "<<sqrt(pt2.x*pt2.x + pt2.y*pt2.y)<<endl;
    }
    pcl::toPCLPointCloud2(TotalCloud, output);
    return output;

}
//--------------------- FILTER APPLY----------------
pcl::PCLPointCloud2 roi_filter(pcl::PCLPointCloud2ConstPtr cloudPtr)
{
   pcl::PCLPointCloud2 output;
   pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
   cropFilter.setInputCloud(cloudPtr);
   cropFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 0)); // x, y, z, min (m)
   cropFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 0));
   cropFilter.filter(output);
   return output;
}

pcl::PCLPointCloud2 voxelGrid(pcl::PCLPointCloud2ConstPtr cloudPtr)
{
  pcl::PCLPointCloud2 output;
  pcl::VoxelGrid<pcl::PCLPointCloud2> VG;
  VG.setInputCloud(cloudPtr);
  VG.setLeafSize(0.1f, 0.1f, 0.1f);
  VG.filter(output);
  return output;
}


void applyFilter(const sensor_msgs::PointCloud2ConstPtr& msg){

    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2* source = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*msg, *source);
    pcl::PCLPointCloud2* cloud= new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    *cloud = *source;
    *cloud = roi_filter(cloudPtr);  //ROI 필터 적용
    // *cloud = voxelGrid(cloudPtr);   //VoxelGrid 적용
    pcl_conversions::fromPCL(*cloud, output); 
    output.header.frame_id = "velodyne";
    filtered_pub.publish(output);

    *cloud = cloud_cb(cloudPtr); // apply clustering
    // *cloud = segmentation(cloudPtr);

    pcl_conversions::fromPCL(*cloud, output); 
    output.header.frame_id = "velodyne";
    clustered_pub.publish(output);
    cluster_callback(&output);
    // obstacle_detect(*cloud);        // 물체 
}
void getParameter(ros::NodeHandle nh){
  nh.getParam("minX",minX);
  nh.getParam("minY",minY);
  nh.getParam("minZ",minZ);
  nh.getParam("maxX",maxX);
  nh.getParam("maxY",maxY);
  nh.getParam("maxZ",maxZ);
}
//-------------------------------------------------------
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lidar_Prefiltering");
  ros::NodeHandle nh;
  ros::Subscriber vlp_sub = nh.subscribe("/velodyne_points", 10, applyFilter); // raw data callback (doing preprocessing)
  filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_filtered", 100); // transmit processed output
  clustered_pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_clustered", 100); // transmit processed output
  cons_pub = nh.advertise<geometry_msgs::PoseArray> ("/lane/cons", 100); // transmit processed output
    
  getParameter(nh);

  ros::spin();
}