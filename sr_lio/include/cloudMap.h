#pragma once
// c++
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>

// eigen 
#include <Eigen/Core>
#include <Eigen/Dense>

// robin_map
#include <tsl/robin_map.h>

struct point3D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d raw_point;
    Eigen::Vector3d point;
    Eigen::Vector3d imu_point;
    double alpha_time = 0.0;
    double relative_time = 0.0;
    double timestamp = 0.0;
    int index_frame = -1;

    // 2024.04.19 yzk
    int label = 0;
    double range = -1.0;
    bool is_dynamic = false;
    bool is_undecided = false;
    int num_decided = 0;
    int num_out_view = 0;
    // 2024.04.19 yzk

    point3D() = default;
};

// 2024.04.19 yzk
class globalPoint
{
private:
    Eigen::Vector3f position;

    short label;
    bool is_dynamic;

public:

    globalPoint();

    void setPosition(Eigen::Vector3d &position_);

    void setLabel(int label_);

    void setDynamic(bool is_dynamic_);

    Eigen::Vector3d getPosition();

    int getLabel();

    bool isDynamic();
};
// 2024.04.19 yzk

// 2023.08.07 yzk
struct planeParam {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d raw_point;
    Eigen::Vector3d norm_vector;
    Eigen::Matrix<double, 1, 6> jacobians;
    double norm_offset;
    double distance = 0.0;
    double weight = 1.0;

    planeParam() = default;
};
// 2023.08.07 yzk

// 2023.08.12 yzk
struct imuState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp;

    Eigen::Vector3d un_acc;
    Eigen::Vector3d un_gyr;
    Eigen::Vector3d trans;
    Eigen::Quaterniond quat;
    Eigen::Vector3d vel;

    imuState() = default;
};
// 2023.08.12 yzk

// 2024.04.19 yzk
struct voxel {

    voxel() = default;

    voxel(short x, short y, short z) : x(x), y(y), z(z) {}

    bool operator==(const voxel &vox) const { return x == vox.x && y == vox.y && z == vox.z; }

    inline bool operator<(const voxel &vox) const {
        return x < vox.x || (x == vox.x && y < vox.y) || (x == vox.x && y == vox.y && z < vox.z);
    }

    inline static voxel coordinates(globalPoint &point, double voxel_size) {
        return {short(point.getPosition().x() / voxel_size),
                short(point.getPosition().y() / voxel_size),
                short(point.getPosition().z() / voxel_size)};
    }

    short x;
    short y;
    short z;
};

struct voxelBlock {

    explicit voxelBlock(int num_points_ = 20) : num_points(num_points_) { points.reserve(num_points_); }

    std::vector<globalPoint> points;

    bool IsFull() const { return num_points == points.size(); }

    void AddPoint(const globalPoint &point) {
        assert(num_points > points.size());
        points.push_back(point);
    }

    inline int NumPoints() const { return points.size(); }

    inline int Capacity() { return num_points; }

private:
    int num_points;
};
// 2024.04.19 yzk

typedef tsl::robin_map<voxel, voxelBlock> voxelHashMap;

namespace std {

    template<> struct hash<voxel> {
        std::size_t operator()(const voxel &vox) const
        {
            const size_t kP1 = 73856093;
            const size_t kP2 = 19349669;
            const size_t kP3 = 83492791;
            return vox.x * kP1 + vox.y * kP2 + vox.z * kP3;
        }
    };
}