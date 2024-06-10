#pragma once
// c++
#include <iostream>
#include <math.h>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// eigen 
#include <Eigen/Core>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ceres/ceres.h>
#include <chrono>

#include "cloudMap.h"

enum LID_TYPE{AVIA = 1, VELO = 2, OUST = 3, ROBO = 4};
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum Edge_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct extraElement
{
	double range;
	double distance;
	double angle[2];
	double intersect;
	Edge_jump edge_jump[2];
	Feature feature_type;

	extraElement()
	{
		range = 0;
		edge_jump[Prev] = Nr_nor;
		edge_jump[Next] = Nr_nor;
		feature_type = Nor;
		intersect = 2;
	}
};

namespace velodyne_ros {
	struct EIGEN_ALIGN16 Point {
	 	PCL_ADD_POINT4D;
	 	float intensity;
	 	float time;
	 	uint16_t ring;
	 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace robosense_ros
{
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        uint8_t intensity;
        uint16_t ring = 0;
        double timestamp = 0;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint8_t, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

namespace ouster_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint32_t, t, t)
    (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)
    (uint16_t, ambient, ambient)
    (uint32_t, range, range)
)

class cloudProcessing
{
private:

	int lidar_type;
	int N_SCANS;

	int Horizon_SCANS;
	float ang_res_x;
	float ang_res_y;
	float ang_bottom;
	int ground_scan_id;

	float sensor_mount_angle;
	float segment_theta;
	int segment_valid_point_num;
	int segment_valid_line_num;
	float segment_alpha_x;
	float segment_alpha_y;

	int SCAN_RATE;
	int time_unit;
	double time_unit_scale;
	double blind;

	Eigen::Matrix3d R_imu_lidar;
	Eigen::Vector3d t_imu_lidar;

	bool given_offset_time;

	bool given_ring;

	cv::Mat range_mat;
    cv::Mat label_mat;
    cv::Mat ground_mat;
    int label_count;

    point3D nan_point;
    std::vector<point3D> full_cloud;

    std::vector<std::pair<int8_t, int8_t>> neighbor_iterator;

    uint16_t *all_pushed_id_x;
    uint16_t *all_pushed_id_y;

    uint16_t *queue_id_x;
    uint16_t *queue_id_y;

	int point_filter_num;

	int sweep_id;

	double delta_cut_time;

	std::vector<pcl::PointCloud<pcl::PointXYZINormal>> scan_cloud;
	std::vector<std::vector<extraElement>> v_extra_elem;

	// function
	void ousterHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, double &dt_offset);
	void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, double &dt_offset);
	void robosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, double &dt_offset);

	double plane_parameters[4] = {-20,500, 80,-70};

	void resetParameters();

	void groundRemoval();

	void cloudSegmentation(std::vector<point3D> &v_cloud_out);

	void labelComponents(int row, int col);

public:

	cloudProcessing();

	void setLidarType(int para);
	void setNumScans(int para);

	void setHorizonScans(int para);
	void setAngResX(float para);
	void setAngResY(float para);
	void setAngBottom(float para);
	void setGroundScanInd(int para);

	void setScanRate(int para);
	void setTimeUnit(int para);
	void setBlind(double para);

	void setExtrinR(Eigen::Matrix3d &R);
	void setExtrinT(Eigen::Vector3d &t);

	void setUseFeature(bool para);
	void setPointFilterNum(int para);

	int getLidarType() {return lidar_type;}

	bool isPointTimeEnable() {return given_offset_time;}

	void process(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<point3D> &v_cloud_out, double &dt_offset);

	void initialization();
};