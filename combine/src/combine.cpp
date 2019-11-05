#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <visualization_msgs/Marker.h>

//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>

//#include <pcl/segmentation/conditional_euclidean_clustering.h>

//#include <pcl/common/common.h>
//#include <pcl/segmentation/extract_clusters.h>

//#include <pcl/search/organized.h>
//#include <pcl/search/kdtree.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <string>
#include <vector>

#define LEAF_SIZE 0.1 

class Combine{
public:
    
	Combine();
	~Combine();

private:
	ros::Subscriber sub_lidar_;
    ros::Subscriber sub_carmer_;
    //ros::Publisher  pub_loaction_;
    ros::Publisher marker_pub;
    
    //外参,旋转矩阵R
    const double Matrix[3][3] = {{-0.006223, -0.99955, 0.029640}, {0.074689, -0.029418, -1.007846}, {1.008262, -0.008343, -0.074473}};
    //内参
    const double CameraMat[3][3] = {{0.002853, 0, -0.891156}, {0, 0.002858, -0.653638}, {0, 0, 1}};
    //外参,平移矩阵t
    const double v_coor_trans[3] = {1.024, -0.039, -0.936};
    
    std_msgs::Header point_cloud_header_;
	
	//std::vector<std::string> cam_label;
    std::vector<float> cam_X;
    std::vector<float> cam_Y;
    
    std::vector<std::string> lidar_label;
    std::vector<float> lidar_X;
    std::vector<float> lidar_Y;
    std::vector<float> lidar_Z;
	
	ros::NodeHandle nh;
	
	int cntt = 0;
	
	void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);
	void visual_marker(float x, float y, float z);
	void lidar_pre(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);
	void transfrom_coor(std::vector<float> &cam_x, std::vector<float> &cam_y);
	void locat_pub(darknet_ros_msgs::BoundingBoxes msg_cam);
	
};

Combine::Combine(){
	//点云预处理
	sub_lidar_ = nh.subscribe("/velodyne_points", 5, &Combine::lidar_pre, this);
	sub_carmer_ = nh.subscribe("/darknet_ros/bounding_boxes", 5, &Combine::locat_pub, this);
	
	marker_pub = nh.advertise<visualization_msgs::Marker>("visual_marker", 1);
}

Combine::~Combine(){}

void Combine::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
}

void Combine::visual_marker(float x, float y, float z){
	visualization_msgs::Marker marker;
    marker.header.frame_id = "/velodyne";
    marker.header.stamp = ros::Time::now();
	
	marker.ns = "basic_shapes";
 	marker.id = 0;
 	
 	marker.type = visualization_msgs::Marker::CUBE;
 	
 	marker.action = visualization_msgs::Marker::ADD;
 	
 	marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

	marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;
    
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration(0.5);
    
    marker_pub.publish(marker);
}

void Combine::lidar_pre(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr){
	//cntt++;
	//std::cout << cntt << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    point_cloud_header_ = in_cloud_ptr->header;
    
    /*for(int i = 0; i < in_cloud_ptr.points.size(); i++){
    	if(in_cloud_ptr.points[i].x < 0){
    		
    	}
    }*/
    
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    voxel_grid_filer(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);
    //lidar_X，lidar_Y由transfrom_coor计算得到
    float x_offset, y_offset;
    if(lidar_X.size()){
		for(int j = 0; j < lidar_X.size(); j++){
			for(int i = 0; i < filtered_pc_ptr->points.size(); i++){
				x_offset = filtered_pc_ptr->points[i].y - lidar_X[j];
				y_offset = filtered_pc_ptr->points[i].z - lidar_Y[j] - 1;
				if(fabs(x_offset) < 0.2 && fabs(y_offset) < 0.5 && filtered_pc_ptr->points[i].x > 3){
					std::cout << "lidar_X: " << lidar_X[j] << std::endl;
					lidar_X[j] = filtered_pc_ptr->points[i].x;
					lidar_Y[j] = filtered_pc_ptr->points[i].y;
					lidar_Z[j] = filtered_pc_ptr->points[i].z;
					visual_marker(lidar_X[j], lidar_Y[j], lidar_Z[j]);
					//std::cout << "time:" << cntt << " lidar_X:" << lidar_X[j] << " lidar_Y:" << lidar_Y[j] << " lidar_Z:" << lidar_Z[j] << std::endl;
					break;
				}
			}
		}
	}
	std::cout << "cntt: " << cntt << std::endl;
}

void Combine::transfrom_coor(std::vector<float> &cam_x, std::vector<float> &cam_y){
	lidar_X.clear();
   	lidar_Y.clear();
   	lidar_Z.clear();
	tf::Vector3 v_coor, v_temp, v_temp1, v_temp2;
	for(int i = 0; i < cam_x.size(); i++){
		cntt++;
		v_coor[0] = cam_x[i];
		v_coor[1] = cam_y[i];
		v_coor[2] = 1;
		//内参计算
		for (int m = 0; m < 3; m++)
        {
            for (int k = 0; k < 3; k++)
                v_temp[m] += CameraMat[m][k] * v_coor[k];
        }
        //外参计算
        v_temp1[0] = v_temp[0] - v_coor_trans[0];
        v_temp1[1] = v_temp[1] - v_coor_trans[1];
        v_temp1[2] = v_temp[2] - v_coor_trans[2];              
        for (int m = 0; m < 3; m++)
        {
            for (int k = 0; k < 3; k++)
                v_temp2[m] += Matrix[m][k] * v_temp1[k];
        }
        lidar_X.push_back(v_temp2[0]);
        lidar_Y.push_back(v_temp2[1]);
        lidar_Z.push_back(v_temp2[2]);
        std::cout << "time:" << cntt << ' ' << i << ' '<< " lidar_X:" << lidar_X[i] << " lidar_Y:" << lidar_Y[i] << std::endl;
	}
}

void Combine::locat_pub(darknet_ros_msgs::BoundingBoxes msg_cam){
    if(msg_cam.bounding_boxes.size()>0){
        //cam_label.clear();
        cam_X.clear();
    	cam_Y.clear();
    	for(int i=0;i<msg_cam.bounding_boxes.size();i++){
        	if(msg_cam.bounding_boxes[i].Class == "car"){
        		//cam_label.push_back(msg_cam.bounding_boxes[i].Class);
        		cam_X.push_back((msg_cam.bounding_boxes[i].xmax + msg_cam.bounding_boxes[i].xmin) / 2);
        		cam_Y.push_back((msg_cam.bounding_boxes[i].ymax + msg_cam.bounding_boxes[i].ymin) / 2);
      		}
      	}
    }
    transfrom_coor(cam_X, cam_Y);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "combine");
	Combine core;
	ros::spin();
	return 0;
}


