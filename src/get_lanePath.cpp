#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>


#include <con.cpp>
#include <cmath>
using namespace std;

ros::Publisher con_path_pub;
ros::Publisher car_path_pub;
ros::Publisher middle_path_pub;

ros::Publisher target_velo_pub;
ros::Publisher obs_path_pub;
ros::Publisher car_angle_pub;
ros::Publisher car_speed_pub;

ConList conlist;
bool detected=false;

void con_path_publish(){
    // if(conlist.conPath.poses.size() != 0)
    // con_path_pub.publish(conlist.conPath);
    // if(conlist.carPath.poses.size() != 0)
    // car_path_pub.publish(conlist.carPath);
    // if(conlist.middlePath.poses.size() != 0)
    middle_path_pub.publish(conlist.middlePath);
}

int angle_publish(){
    float yaw =0;
    if(conlist.middlePath.poses.size()==0 || conlist.middlePath.poses.size()==1) {
        // cout<<"NONE"<<endl;

        cout<<"LEFT_NUM "<<conlist.left_con_num<<endl;
        cout<<"RIGHT_NUM "<<conlist.right_con_num<<endl;
        int sum = conlist.left_con_num+conlist.right_con_num;
        cout<<"SUMSUMSUMSUMSUM  "<<sum<<endl;
    system("clear");

        if(sum > 1) yaw = -28;
        else if(sum < -1) yaw = 28;
        else yaw=0;
        return yaw;
        
        }
    cout<<"SIZE = "<<conlist.middlePath.poses.size()<<endl;
    cout<<"COORD "<<conlist.middlePath.poses[0].pose.position.x<<" "<<conlist.middlePath.poses[0].pose.position.y<<endl;
    cout<<"COORD "<<conlist.middlePath.poses[1].pose.position.x<<" "<<conlist.middlePath.poses[1].pose.position.y<<endl;

    // float yaw = atan2(conlist.carPath.poses[0].pose.position.y,conlist.carPath.poses[0].pose.position.x)*180/M_PI;
    double dist =0;
    if(conlist.middlePath.poses.size() == 2){
        yaw= atan2(
        (conlist.middlePath.poses[0].pose.position.y+conlist.middlePath.poses[1].pose.position.y)*0.5 ,
        (conlist.middlePath.poses[0].pose.position.x+conlist.middlePath.poses[1].pose.position.x)*0.5
        )*180/M_PI;

        double navX = (conlist.middlePath.poses[0].pose.position.x+conlist.middlePath.poses[1].pose.position.x)*0.5;
        double navY = (conlist.middlePath.poses[0].pose.position.y+conlist.middlePath.poses[1].pose.position.y)*0.5;
        dist = sqrt(navX*navX+navY*navY);
        if(dist > 2){
            // yaw = yaw*0.8;
        }
    }
    yaw =-yaw*2;  
    int car_yaw = yaw*71.4;
    cout<<setprecision(6)<<"YAW = "<<yaw<<endl;
    cout<<"DIST = "<<dist<<endl;
    cout<<"CAR YAW = "<<car_yaw<<endl;

    std_msgs::Float32 yaw_msg;
    yaw_msg.data = yaw;
    if(detected)
    yaw_msg.data = 2000;
    car_angle_pub.publish(yaw_msg);
    std_msgs::UInt16 speed_msg;
    speed_msg.data = 6;
    if(abs(yaw)> 70 )
    speed_msg.data = 5;
    car_speed_pub.publish(speed_msg);

    system("clear");
    return 0;
}
void obs_callback(const std_msgs::Bool msg){
    detected = msg.data;
}
void cluster_callback(const sensor_msgs::PointCloud2ConstPtr& msg){

    //------------------------------------- Type Convert ------------------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg,*pCloud);
    pcl::PointCloud<pcl::PointXYZI>::iterator iter;

    conlist.clear();
    for(iter = pCloud->begin();iter!=pCloud->end();iter++){
        if(conlist.findIdx(iter->intensity) == -1){\
            Con newCon(iter->x, iter->y, 0.3, iter->intensity);
            conlist.addCon(newCon);
        }
    }
}

int main (int argc, char** argv){

    //--------------------------- INIT --------------------------------------------------
    ros::init(argc,argv, "get_lanePath");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/velodyne_clustered",10,cluster_callback);
    con_path_pub = nh.advertise<nav_msgs::Path>("/hg_lane/Path",1);
    car_path_pub = nh.advertise<nav_msgs::Path>("/planning/path",1);
    middle_path_pub = nh.advertise<nav_msgs::Path>("/hg_lane/middle_path",1);

    target_velo_pub = nh.advertise<std_msgs::Int16>("/max_v",10);
    obs_path_pub = nh.advertise<std_msgs::Bool>("/planning/obs",100);
    car_angle_pub = nh.advertise<std_msgs::Float32>("/control/angle", 1);
    car_speed_pub = nh.advertise<std_msgs::UInt16>("/control/accel",1);


    std_msgs::Int16 velo_msg;
    velo_msg.data = 4;
    std_msgs::Bool path_msg;
    path_msg.data = true;

    ros::Rate r(40);
    conlist.init(nh);
    //----------------------------------------------------------------------------------

    while(ros::ok()){
        target_velo_pub.publish(velo_msg);
        obs_path_pub.publish(path_msg);
        ros::spinOnce();
        r.sleep();
        conlist.sorting();
        conlist.getPath();
        conlist.getPath2();
        con_path_publish();
        angle_publish();
    }
    ros::shutdown();
    return 0;
}