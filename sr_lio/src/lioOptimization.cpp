#include "lioOptimization.h"

cloudFrame::cloudFrame(std::vector<point3D> &point_frame_, state *p_state_)
{
    point_frame.insert(point_frame.end(), point_frame_.begin(), point_frame_.end());

    p_state = p_state_;
}

cloudFrame::cloudFrame(cloudFrame *p_cloud_frame)
{
    time_sweep_begin = p_cloud_frame->time_sweep_begin;
    time_sweep_end = p_cloud_frame->time_sweep_end;

    id = p_cloud_frame->id;
    frame_id = p_cloud_frame->frame_id;

    p_state = p_cloud_frame->p_state;

    point_frame.insert(point_frame.end(), p_cloud_frame->point_frame.begin(), p_cloud_frame->point_frame.end());

    offset_begin = p_cloud_frame->offset_begin;
    offset_end = p_cloud_frame->offset_end;
    dt_offset = p_cloud_frame->dt_offset;
}

void cloudFrame::release()
{
    std::vector<point3D>().swap(point_frame);

    if(p_state != nullptr)
        p_state->release();

    delete p_state;

    p_state = nullptr;
}

estimationSummary::estimationSummary()
{

}

void estimationSummary::release()
{

}

lioOptimization::lioOptimization()
{
	allocateMemory();

    readParameters();

    initialValue();

    pub_cloud_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_current", 2);
    pub_cloud_world = nh.advertise<sensor_msgs::PointCloud2>("/cloud_global_map", 2);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/Odometry_after_opt", 5);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 5);

    sub_cloud_ori = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 20, &lioOptimization::standardCloudHandler, this);
    sub_imu_ori = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, &lioOptimization::imuHandler, this);

    path.header.stamp = ros::Time::now();
    path.header.frame_id ="camera_init";
    points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());

    //options.recordParameters();
}

void lioOptimization::readParameters()
{	
    int para_int;
    double para_double;
    float para_float;
    bool para_bool;
    std::string str_temp;

    // common
    nh.param<std::string>("common/lidar_topic", lidar_topic, "/points_raw");
	nh.param<std::string>("common/imu_topic", imu_topic, "/imu_raw");
    nh.param<int>("common/point_filter_num", para_int, 1);  cloud_pro->setPointFilterNum(para_int);
    nh.param<std::vector<double>>("common/gravity_acc", v_G, std::vector<double>());
    nh.param<bool>("debug_output", debug_output, false);
    nh.param<std::string>("output_path", output_path, "");

    // LiDAR parameter
    nh.param<int>("lidar_parameter/lidar_type", para_int, AVIA);  cloud_pro->setLidarType(para_int);
    nh.param<int>("lidar_parameter/N_SCANS", para_int, 16);  cloud_pro->setNumScans(para_int);
    nh.param<int>("lidar_parameter/Horizon_SCANS", para_int, 1800);  cloud_pro->setHorizonScans(para_int);
    nh.param<float>("lidar_parameter/ang_res_x", para_float, 0.2);  cloud_pro->setAngResX(para_float);
    nh.param<float>("lidar_parameter/ang_res_y", para_float, 1.33);  cloud_pro->setAngResY(para_float);
    nh.param<float>("lidar_parameter/ang_bottom", para_float, 30.67);  cloud_pro->setAngBottom(para_float);
    nh.param<int>("lidar_parameter/ground_scan_id", para_int, 20);  cloud_pro->setGroundScanInd(para_int);

    nh.param<int>("lidar_parameter/SCAN_RATE", para_int, 10);  cloud_pro->setScanRate(para_int);
    nh.param<int>("lidar_parameter/time_unit", para_int, US);  cloud_pro->setTimeUnit(para_int);
    nh.param<double>("lidar_parameter/blind", para_double, 0.01);  cloud_pro->setBlind(para_double);
    nh.param<float>("lidar_parameter/det_range", det_range, 300.f);
    nh.param<double>("lidar_parameter/fov_degree", fov_deg, 180);

    // IMU parameter
    nh.param<double>("imu_parameter/acc_cov", para_double, 0.1);  eskf_pro->setAccCov(para_double);
    nh.param<double>("imu_parameter/gyr_cov", para_double, 0.1);  eskf_pro->setGyrCov(para_double);
    nh.param<double>("imu_parameter/b_acc_cov", para_double, 0.0001);  eskf_pro->setBiasAccCov(para_double);
    nh.param<double>("imu_parameter/b_gyr_cov", para_double, 0.0001);  eskf_pro->setBiasGyrCov(para_double);

    nh.param<bool>("imu_parameter/time_diff_enable", time_diff_enable, false);

    // extrinsic parameter
    nh.param<bool>("extrinsic_parameter/extrinsic_enable", extrin_enable, true);
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_t", v_extrin_t, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_parameter/extrinsic_R", v_extrin_R, std::vector<double>());

    // state estimation parameters
    nh.param<double>("odometry_options/init_voxel_size", options.init_voxel_size, 0.2);
    nh.param<double>("odometry_options/init_sample_voxel_size", options.init_sample_voxel_size, 1.0);
    nh.param<int>("odometry_options/init_num_frames", options.init_num_frames, 20);
    nh.param<double>("odometry_options/voxel_size", options.voxel_size, 0.5);
    nh.param<double>("odometry_options/sample_voxel_size", options.sample_voxel_size, 1.5);
    nh.param<double>("odometry_options/max_distance", options.max_distance, 100.0);
    nh.param<int>("odometry_options/max_num_points_in_voxel", options.max_num_points_in_voxel, 20);
    nh.param<double>("odometry_options/min_distance_points", options.min_distance_points, 0.1);
    nh.param<double>("odometry_options/distance_error_threshold", options.distance_error_threshold, 5.0);

    nh.param<std::string>("odometry_options/motion_compensation", str_temp, "CONSTANT_VELOCITY");
    if(str_temp == "IMU") options.motion_compensation = IMU;
    else if(str_temp == "CONSTANT_VELOCITY") options.motion_compensation = CONSTANT_VELOCITY;
    else std::cout << "The `motion_compensation` " << str_temp << " is not supported." << std::endl;

    nh.param<std::string>("odometry_options/initialization", str_temp, "INIT_IMU");
    if(str_temp == "INIT_IMU") options.initialization = INIT_IMU;
    else if(str_temp == "INIT_CONSTANT_VELOCITY") options.initialization = INIT_CONSTANT_VELOCITY;
    else std::cout << "The `state_initialization` " << str_temp << " is not supported." << std::endl;


    icpOptions optimize_options;
    nh.param<int>("icp_options/threshold_voxel_occupancy", options.optimize_options.threshold_voxel_occupancy, 1);
    nh.param<double>("icp_options/size_voxel_map", options.optimize_options.size_voxel_map, 1.0);
    nh.param<int>("icp_options/num_iters_icp", options.optimize_options.num_iters_icp, 5);
    nh.param<int>("icp_options/min_number_neighbors", options.optimize_options.min_number_neighbors, 20);
    nh.param<int>("icp_options/voxel_neighborhood", options.optimize_options.voxel_neighborhood, 1);
    nh.param<double>("icp_options/power_planarity", options.optimize_options.power_planarity, 2.0);
    nh.param<bool>("icp_options/estimate_normal_from_neighborhood", options.optimize_options.estimate_normal_from_neighborhood, true);
    nh.param<int>("icp_options/max_number_neighbors", options.optimize_options.max_number_neighbors, 20);
    nh.param<double>("icp_options/max_dist_to_plane_icp", options.optimize_options.max_dist_to_plane_icp, 0.3);
    nh.param<double>("icp_options/threshold_orientation_norm", options.optimize_options.threshold_orientation_norm, 0.0001);
    nh.param<double>("icp_options/threshold_translation_norm", options.optimize_options.threshold_translation_norm, 0.001);
    nh.param<int>("icp_options/max_num_residuals", options.optimize_options.max_num_residuals, -1);
    nh.param<int>("icp_options/min_num_residuals", options.optimize_options.min_num_residuals, 100);
    nh.param<int>("icp_options/num_closest_neighbors", options.optimize_options.num_closest_neighbors, 1);
    nh.param<double>("icp_options/weight_alpha", options.optimize_options.weight_alpha, 0.9);
    nh.param<double>("icp_options/weight_neighborhood", options.optimize_options.weight_neighborhood, 0.1);
    nh.param<bool>("icp_options/debug_print", options.optimize_options.debug_print, true);
    nh.param<bool>("icp_options/debug_viz", options.optimize_options.debug_viz, false);
}

void lioOptimization::allocateMemory()
{
    cloud_pro = new cloudProcessing();
    eskf_pro = new eskfEstimator();

    down_cloud_body.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    down_cloud_world.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
}

void lioOptimization::initialValue()
{
    laser_point_cov = 0.001;

    G = vec3FromArray(v_G);
    G_norm = G.norm();
    R_imu_lidar = mat33FromArray(v_extrin_R);
    t_imu_lidar = vec3FromArray(v_extrin_t);

    cloud_pro->setExtrinR(R_imu_lidar);
    cloud_pro->setExtrinT(t_imu_lidar);

    dt_sum = 0;

    last_time_lidar = -1.0;
    last_time_imu = -1.0;
    last_time_frame = -1.0;
    current_time = -1.0;

    index_frame = 1;

    fov_deg = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);

    options.optimize_options.init_num_frames = options.init_num_frames;

    cloud_pro->initialization();

    // loop closing
    loop_map.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pub_loop_map = nh.advertise<sensor_msgs::PointCloud2>("/loop_map", 2);
    // loop closing
}

void lioOptimization::addPointToMap(voxelHashMap &map, point3D &point3d, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points, cloudFrame* p_frame)
{
    globalPoint point;
    point.setPosition(point3d.point);
    point.setLabel(point3d.label);
    point.setDynamic(point3d.is_dynamic);

    short kx = static_cast<short>(point.getPosition().x() / voxel_size);
    short ky = static_cast<short>(point.getPosition().y() / voxel_size);
    short kz = static_cast<short>(point.getPosition().z() / voxel_size);

    voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

    if(search != map.end())
    {
        auto &voxel_block = (search.value());

        if(!voxel_block.IsFull()) {
            double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
            for (int i(0); i < voxel_block.NumPoints(); ++i)
            {
                auto &_point = voxel_block.points[i];
                double sq_dist = (_point.getPosition() - point.getPosition()).squaredNorm();
                if (sq_dist < sq_dist_min_to_points)
                {
                    sq_dist_min_to_points = sq_dist;
                }
            }
            if(sq_dist_min_to_points > (min_distance_points * min_distance_points))
            {
                if(min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                {
                    voxel_block.AddPoint(point);
                    addPointToPcl(points_world, point);
                }
            }
        }
    }
    else
    {
        if(min_num_points <= 0){
            voxelBlock block(max_num_points_in_voxel);
            block.AddPoint(point);
            map[voxel(kx, ky, kz)] = std::move(block);
        }

    }
}

void lioOptimization::addPointsToMap(voxelHashMap &map, cloudFrame* p_frame, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points)
{
    for (auto &point: p_frame->point_frame)
    {
        if (point.is_dynamic == false)
        {
            addPointToMap(map, point, voxel_size, max_num_points_in_voxel, min_distance_points, min_num_points, p_frame);
            
            if (point.is_undecided == false)
                addPointToTrueMap(voxel_map_true, point, voxel_size, max_num_points_in_voxel, min_distance_points, min_num_points);
        }
    }
    publishCLoudWorld(pub_cloud_world, points_world, p_frame);
    points_world->clear();
}

void lioOptimization::removePointsFarFromLocation(voxelHashMap &map, const Eigen::Vector3d &location, double distance)
{
    std::vector<voxel> voxels_to_erase;
    for (auto &pair: map) {
        globalPoint global_point = pair.second.points[0];
        Eigen::Vector3d pt = global_point.getPosition();

        if ((pt - location).squaredNorm() > (distance * distance)) {
            voxels_to_erase.push_back(pair.first);
        }
    }
    for (auto &vox: voxels_to_erase)
        map.erase(vox);

    std::vector<voxel>().swap(voxels_to_erase);
}

size_t lioOptimization::mapSize(const voxelHashMap &map)
{
    size_t map_size(0);
    for (auto &itr_voxel_map: map) {
        map_size += (itr_voxel_map.second).NumPoints();
    }
    return map_size;
}

void lioOptimization::standardCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    double sample_size = index_frame < options.init_num_frames ? options.init_voxel_size : options.voxel_size;

    assert(msg->header.stamp.toSec() > last_time_lidar);
    
    std::vector<point3D> v_point_cloud;
    double dt_offset;

    cloud_pro->process(msg, v_point_cloud, dt_offset);

    boost::mt19937_64 g;
    std::shuffle(v_point_cloud.begin(), v_point_cloud.end(), g);

    subSampleFrame(v_point_cloud, sample_size);

    std::shuffle(v_point_cloud.begin(), v_point_cloud.end(), g);

    lidar_buffer.push(v_point_cloud);
    time_buffer.push(std::make_pair(msg->header.stamp.toSec(), dt_offset / (double)1000.0));

    assert(msg->header.stamp.toSec() > last_time_lidar);
    last_time_lidar = msg->header.stamp.toSec();
}

void lioOptimization::imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu::Ptr msg_temp(new sensor_msgs::Imu(*msg));

    if (abs(time_diff) > 0.1 && time_diff_enable)
    {
        msg_temp->header.stamp = ros::Time().fromSec(time_diff + msg->header.stamp.toSec());
    }

    assert(msg_temp->header.stamp.toSec() > last_time_imu);

    imu_buffer.push(msg_temp);

    assert(msg_temp->header.stamp.toSec() > last_time_imu);
    last_time_imu = msg_temp->header.stamp.toSec();
}

std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<point3D>>, std::pair<double, double>>> lioOptimization::getMeasurements()
{
    std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<point3D>>, std::pair<double, double>>> measurements;

    while (true)
    {
        if(imu_buffer.size() < 60 || lidar_buffer.size() < 2 || time_buffer.size() < 2)
            return measurements;

        if (!(imu_buffer.back()->header.stamp.toSec() > time_buffer.front().first + time_buffer.front().second))
        {
            return measurements;
        }

        if (!(imu_buffer.front()->header.stamp.toSec() < time_buffer.front().first + time_buffer.front().second))
        {
            time_buffer.pop();

            std::vector<point3D>().swap(lidar_buffer.front());
            assert(lidar_buffer.front().size() == 0);
            lidar_buffer.pop();

            continue;
        }

        double timestamp = time_buffer.front().first + time_buffer.front().second;
        double timestamp_begin = time_buffer.front().first;
        double timestamp_offset = time_buffer.front().second;
        time_buffer.pop();

        if (fabs(timestamp_begin - time_buffer.front().first) > 1e-5)
        {
            if (time_buffer.front().first - timestamp_begin - timestamp_offset > 1e-5)
            {
                timestamp_offset = time_buffer.front().first - timestamp_begin;
                timestamp = timestamp_begin + timestamp_offset;
            }
            else if (time_buffer.front().first - timestamp_begin - timestamp_offset < 1e-5)
            {
                timestamp_offset = time_buffer.front().first - timestamp_begin;
                timestamp = timestamp_begin + timestamp_offset;
            }
        }

        std::vector<point3D> v_point_cloud = lidar_buffer.front();

        std::vector<point3D>().swap(lidar_buffer.front());
        assert(lidar_buffer.front().size() == 0);
        lidar_buffer.pop();

        std::vector<sensor_msgs::ImuConstPtr> imu_measurements;
        while (imu_buffer.front()->header.stamp.toSec() < timestamp)
        {
            imu_measurements.emplace_back(imu_buffer.front());
            imu_buffer.pop();
        }

        imu_measurements.emplace_back(imu_buffer.front());

        if (imu_measurements.empty())
            ROS_WARN("no imu between two sweeps");

        measurements.emplace_back(std::make_pair(imu_measurements, v_point_cloud), std::make_pair(timestamp_begin, timestamp_offset));
        break;
    }
    return measurements;
}

void lioOptimization::makePointTimestamp(std::vector<point3D> &sweep, double time_sweep_begin, double time_sweep_end)
{
    if(cloud_pro->isPointTimeEnable())
    {
        return;
    }
    else
    {
        double delta_t = time_sweep_end - time_sweep_begin;

        std::vector<point3D>::iterator iter = sweep.begin();

        while (iter != sweep.end())
        {
            if((*iter).timestamp > time_sweep_end) iter = sweep.erase(iter);
            else if((*iter).timestamp < time_sweep_begin) iter = sweep.erase(iter);
            else
            {
                (*iter).relative_time = (*iter).timestamp - time_sweep_begin;
                (*iter).alpha_time = (*iter).relative_time / delta_t;
                (*iter).relative_time = (*iter).relative_time * 1000.0;
                iter++;
            }
        }
    }
}

cloudFrame* lioOptimization::buildFrame(std::vector<point3D> &const_frame, state *cur_state, double timestamp_begin, double timestamp_offset)
{
    std::vector<point3D> frame(const_frame);

    double offset_begin = 0;
    double offset_end = timestamp_offset;

    double dt_offset = 0;

    if (index_frame > 1)
        dt_offset -= timestamp_begin - all_cloud_frame.back()->time_sweep_end;

    makePointTimestamp(frame, timestamp_begin, timestamp_begin + timestamp_offset);

    if (index_frame <= 2) {
        for (auto &point_temp : frame)
            point_temp.alpha_time = 1.0;
    }

    if (options.motion_compensation == CONSTANT_VELOCITY)
        distortFrameByConstant(frame, imu_states, timestamp_begin, R_imu_lidar, t_imu_lidar);
    else if (options.motion_compensation == IMU)
        distortFrameByImu(frame, imu_states, timestamp_begin, R_imu_lidar, t_imu_lidar);

    transformAllImuPoint(frame, imu_states, R_imu_lidar, t_imu_lidar);

    if (index_frame > 2) {
        for (auto &point_temp: frame) {
            transformPoint(point_temp, cur_state->rotation, cur_state->translation, R_imu_lidar, t_imu_lidar);
        }
    }
    else
    {
        for (auto &point_temp: frame) {
            Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
            Eigen::Vector3d t_zero = Eigen::Vector3d::Zero();
            transformPoint(point_temp, q_identity, t_zero, R_imu_lidar, t_imu_lidar);
        }
    }

    cloudFrame *p_frame = new cloudFrame(const_frame, cur_state);
    p_frame->time_sweep_begin = timestamp_begin;
    p_frame->time_sweep_end = timestamp_begin + timestamp_offset;
    p_frame->offset_begin = offset_begin;
    p_frame->offset_end = offset_end;
    p_frame->dt_offset = dt_offset;
    p_frame->id = all_cloud_frame.size();
    p_frame->frame_id = index_frame;

    all_cloud_frame.push_back(p_frame);

    return p_frame;
}

void lioOptimization::stateInitialization(state *cur_state)
{
    if (index_frame <= 2)
    {
        cur_state->rotation = Eigen::Quaterniond::Identity();
        cur_state->translation = Eigen::Vector3d::Zero();
    }
    else if (index_frame == 3)
    {
        if (options.initialization == INIT_CONSTANT_VELOCITY)
        {
            Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

            Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                         all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                         (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

            cur_state->rotation = q_next_end;
            cur_state->translation = t_next_end;
        }
        else if (options.initialization == INIT_IMU)
        {
            if (initial_flag)
            {
                cur_state->rotation = eskf_pro->getRotation();
                cur_state->translation = eskf_pro->getTranslation();
            }
            else
            {
                Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

                Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                             all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                             (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

                cur_state->rotation = q_next_end;
                cur_state->translation = t_next_end;
            }
        }
        else
        {
            cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
            cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
        }
    }
    else
    {
        if (options.initialization == INIT_CONSTANT_VELOCITY)
        {
            Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                    all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

            Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                         all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                         (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                         all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

            cur_state->rotation = q_next_end;
            cur_state->translation = t_next_end;
        }
        else if (options.initialization == INIT_IMU)
        {
            if (initial_flag)
            {
                cur_state->rotation = eskf_pro->getRotation();
                cur_state->translation = eskf_pro->getTranslation();
            }
            else
            {
                Eigen::Quaterniond q_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                        all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;

                Eigen::Vector3d t_next_end = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation + 
                                             all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation * 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->rotation.inverse() * 
                                             (all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation - 
                                             all_cloud_frame[all_cloud_frame.size() - 2]->p_state->translation);

                cur_state->rotation = q_next_end;
                cur_state->translation = t_next_end;
            }
        }
        else
        {
            cur_state->rotation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
            cur_state->translation = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
        }
    }
}

optimizeSummary lioOptimization::stateEstimation(cloudFrame *p_frame)
{
    icpOptions optimize_options = options.optimize_options;
    const double kSizeVoxelInitSample = options.voxel_size;

    const double kSizeVoxelMap = optimize_options.size_voxel_map;
    const double kMinDistancePoints = options.min_distance_points;
    const int kMaxNumPointsInVoxel = options.max_num_points_in_voxel;

    optimizeSummary optimize_summary;

    if(p_frame->frame_id > 1)
    {
        bool good_enough_registration = false;
        double sample_voxel_size = p_frame->frame_id < options.init_num_frames ? options.init_sample_voxel_size : options.sample_voxel_size;
        double min_voxel_size = std::min(options.init_voxel_size, options.voxel_size);

        optimize_summary = optimize(p_frame, optimize_options, sample_voxel_size);

        if(!optimize_summary.success)
        {
            return optimize_summary;
        }
    }
    else
    {
        p_frame->p_state->translation = eskf_pro->getTranslation();
        p_frame->p_state->rotation = eskf_pro->getRotation();
        p_frame->p_state->velocity = eskf_pro->getVelocity();
        p_frame->p_state->ba = eskf_pro->getBa();
        p_frame->p_state->bg = eskf_pro->getBg();
        G = eskf_pro->getGravity();
        G_norm = G.norm();
    }

    // loop closing
    pubLocalMap(p_frame);
    // loop closing

    addPointsToMap(voxel_map, p_frame, kSizeVoxelMap, kMaxNumPointsInVoxel, kMinDistancePoints);
  
    decidePoints(optimize_options, p_frame);
    addDecidedPointsToTrueMap(kSizeVoxelMap, kMaxNumPointsInVoxel, kMinDistancePoints);

    const double kMaxDistance = options.max_distance;
    const Eigen::Vector3d location = p_frame->p_state->translation;

    removePointsFarFromLocation(voxel_map, location, kMaxDistance);

    return optimize_summary;
}

void lioOptimization::addPointToTrueMap(voxelHashMap &map, point3D &point3d, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points)
{
    globalPoint point;
    point.setPosition(point3d.point);
    point.setLabel(point3d.label);
    point.setDynamic(point3d.is_dynamic);

    short kx = static_cast<short>(point.getPosition().x() / voxel_size);
    short ky = static_cast<short>(point.getPosition().y() / voxel_size);
    short kz = static_cast<short>(point.getPosition().z() / voxel_size);

    voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

    if(search != map.end())
    {
        auto &voxel_block = (search.value());

        if(!voxel_block.IsFull()) {
            double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
            for (int i(0); i < voxel_block.NumPoints(); ++i)
            {
                auto &_point = voxel_block.points[i];
                double sq_dist = (_point.getPosition() - point.getPosition()).squaredNorm();
                if (sq_dist < sq_dist_min_to_points)
                {
                    sq_dist_min_to_points = sq_dist;
                }
            }
            if(sq_dist_min_to_points > (min_distance_points * min_distance_points))
            {
                if(min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                {
                    voxel_block.AddPoint(point);
                    points_vec_true.push_back(&voxel_block.points.back());
                    // addPointToPcl(points_world, point);
                }
            }
        }
    }
    else
    {
        if(min_num_points <= 0){
            voxelBlock block(max_num_points_in_voxel);
            block.AddPoint(point);
            map[voxel(kx, ky, kz)] = std::move(block);
            points_vec_true.push_back(&map[voxel(kx, ky, kz)].points.back());
            // addPointToPcl(points_world, point);
        }

    }
}

void lioOptimization::decidePoints(const icpOptions &cur_icp_options, cloudFrame *p_frame)
{
    const short nb_voxels_visited = p_frame->frame_id < cur_icp_options.init_num_frames ? 2 : cur_icp_options.voxel_neighborhood;
    const int kMinNumNeighbors = cur_icp_options.min_number_neighbors;
    const int kThresholdCapacity = p_frame->frame_id < cur_icp_options.init_num_frames ? 1 : cur_icp_options.threshold_voxel_occupancy;
    const double voxel_size = options.optimize_options.size_voxel_map;

    std::vector<point3D>::iterator iter = points_undecided.begin();

    while (iter != points_undecided.end())
    {
        Eigen::Vector3d distance = (*iter).point - p_frame->p_state->translation;

        if (distance.norm() < 30.0)
        {
            short kx = static_cast<short>((*iter).point.x() / voxel_size);
            short ky = static_cast<short>((*iter).point.y() / voxel_size);
            short kz = static_cast<short>((*iter).point.z() / voxel_size);

            voxelHashMap::iterator search = voxel_map_true.find(voxel(kx, ky, kz));

            if(search != voxel_map_true.end())
            {
                auto &voxel_block = (search.value());

                if(voxel_block.NumPoints() > 5)
                {
                    if ((*iter).label > 0)
                    {
                        int num_ground = 0;

                        for (int i = 0; i < voxel_block.NumPoints(); i++)
                            if (voxel_block.points[i].getLabel() == 0)
                                num_ground++;

                        double ratio_ground = (double)num_ground / (double)voxel_block.NumPoints();

                        if (ratio_ground > 0.1)
                        {
                            (*iter).is_undecided = false;
                            (*iter).is_dynamic = true;
                            iter = points_undecided.erase(iter);
                            continue;
                        }
                        else
                        {
                            (*iter).is_undecided = false;
                        }
                    }
                }
                else
                {
                    (*iter).num_decided++;
                    if ((*iter).num_decided >= 1)
                    {
                        (*iter).is_undecided = false;
                        (*iter).is_dynamic = true;
                        iter = points_undecided.erase(iter);
                        continue;
                    }
                }
            }
            else
            {
                (*iter).num_decided++;
                if ((*iter).num_decided >= 2)
                {
                    (*iter).is_undecided = false;
                }
            }
        }
        else
        {
            (*iter).num_out_view++;
            if ((*iter).num_out_view >= 5)
            {
                (*iter).is_undecided = false;
                iter = points_undecided.erase(iter);
                continue;
            }
        }
        iter++;
    }
}

void lioOptimization::addDecidedPointsToTrueMap(double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points)
{
    std::vector<point3D>::iterator iter = points_undecided.begin();

    while (iter != points_undecided.end())
    {
        if ((*iter).is_undecided == false)
        {
            assert((*iter).is_dynamic == false);
            addPointToTrueMap(voxel_map_true, (*iter), voxel_size, max_num_points_in_voxel, min_distance_points, min_num_points);
            iter = points_undecided.erase(iter);
            continue;
        }
        iter++;
    }
}

void lioOptimization::process(std::vector<point3D> &const_frame, double timestamp_begin, double timestamp_offset)
{
    state *cur_state = new state();

    stateInitialization(cur_state);

    cloudFrame *p_frame = buildFrame(const_frame, cur_state, timestamp_begin, timestamp_offset);

    optimizeSummary summary = stateEstimation(p_frame);

    std::cout << "after solution: " << std::endl;
    std::cout << "rotation: " << p_frame->p_state->rotation.x() << " " << p_frame->p_state->rotation.y() << " " 
              << p_frame->p_state->rotation.z() << " " << p_frame->p_state->rotation.w() << std::endl;
    std::cout << "translation: " << p_frame->p_state->translation.x() << " " << p_frame->p_state->translation.y() << " " << p_frame->p_state->translation.z() << std::endl;
    std::cout << "gravity = " << G.x() << " " << G.y() << " " << G.z() << std::endl;

    dt_sum = 0;

    publish_odometry(pub_odom, p_frame);
    publish_path(pub_path, p_frame);   

    if(debug_output)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr p_cloud_temp;
        p_cloud_temp.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
        point3DtoPCL(p_frame->point_frame, p_cloud_temp);

        std::string pcd_path(output_path + "/cloud_frame/" + std::to_string(index_frame) + std::string(".pcd"));
        saveCutCloud(pcd_path, p_cloud_temp);
    }

    int num_remove = 0;

    if (initial_flag)
    {
        if (index_frame > 1)
        {
            while (all_cloud_frame.size() > 2)
            {
                recordSinglePose(all_cloud_frame[0]);
                all_cloud_frame[0]->release();
                all_cloud_frame.erase(all_cloud_frame.begin());
                num_remove++;
            }
            assert(all_cloud_frame.size() == 2);
        }
    }
    else
    {
        while (all_cloud_frame.size() > options.num_for_initialization)
        {
            recordSinglePose(all_cloud_frame[0]);
            all_cloud_frame[0]->release();
            all_cloud_frame.erase(all_cloud_frame.begin());
            num_remove++;
        }
    }
    

    for(int i = 0; i < all_cloud_frame.size(); i++)
        all_cloud_frame[i]->id = all_cloud_frame[i]->id - num_remove;
}

void lioOptimization::recordSinglePose(cloudFrame *p_frame)
{
    std::ofstream foutC(std::string(output_path + "/pose.txt"), std::ios::app);

    foutC.setf(std::ios::scientific, std::ios::floatfield);
    foutC.precision(6);

    foutC << std::fixed << p_frame->time_sweep_end << " ";
    foutC << p_frame->p_state->translation.x() << " " << p_frame->p_state->translation.y() << " " << p_frame->p_state->translation.z() << " ";
    foutC << p_frame->p_state->rotation.x() << " " << p_frame->p_state->rotation.y() << " " << p_frame->p_state->rotation.z() << " " << p_frame->p_state->rotation.w();
    foutC << std::endl; 

    foutC.close();

    if (initial_flag)
    {
        std::ofstream foutC2(std::string(output_path + "/velocity.txt"), std::ios::app);

        foutC2.setf(std::ios::scientific, std::ios::floatfield);
        foutC2.precision(6);

        foutC2 << std::fixed << p_frame->time_sweep_end << " ";
        foutC2 << p_frame->p_state->velocity.x() << " " << p_frame->p_state->velocity.y() << " " << p_frame->p_state->velocity.z();
        foutC2 << std::endl; 

        foutC2.close();

        std::ofstream foutC3(std::string(output_path + "/bias.txt"), std::ios::app);

        foutC3.setf(std::ios::scientific, std::ios::floatfield);
        foutC3.precision(6);

        foutC3 << std::fixed << p_frame->time_sweep_end << " ";
        foutC3 << p_frame->p_state->ba.x() << " " << p_frame->p_state->ba.y() << " " << p_frame->p_state->ba.z() << " ";
        foutC3 << p_frame->p_state->bg.x() << " " << p_frame->p_state->bg.y() << " " << p_frame->p_state->bg.z();
        foutC3 << std::endl; 

        foutC3.close();
    }
}

void lioOptimization::set_posestamp(geometry_msgs::PoseStamped &body_pose_out,cloudFrame *p_frame)
{
    body_pose_out.pose.position.x = p_frame->p_state->translation.x();
    body_pose_out.pose.position.y = p_frame->p_state->translation.y();
    body_pose_out.pose.position.z = p_frame->p_state->translation.z();
    
    body_pose_out.pose.orientation.x = p_frame->p_state->rotation.x();
    body_pose_out.pose.orientation.y = p_frame->p_state->rotation.y();
    body_pose_out.pose.orientation.z = p_frame->p_state->rotation.z();
    body_pose_out.pose.orientation.w = p_frame->p_state->rotation.w();
}

void lioOptimization::publish_path(ros::Publisher pub_path,cloudFrame *p_frame)
{
    set_posestamp(msg_body_pose,p_frame);
    msg_body_pose.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    msg_body_pose.header.frame_id = "camera_init";

    static int i = 0;
    i++;
    if (i % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pub_path.publish(path);
    }
}

void lioOptimization::publishCLoudWorld(ros::Publisher &pub_cloud_world, pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, cloudFrame* p_frame)
{
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_points, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    laserCloudmsg.header.frame_id = "camera_init";
    pub_cloud_world.publish(laserCloudmsg);
}

void lioOptimization::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, globalPoint& point)
{
    pcl::PointXYZI cloudTemp;
    
    cloudTemp.x = point.getPosition().x();
    cloudTemp.y = point.getPosition().y();
    cloudTemp.z = point.getPosition().z();

    if (point.getLabel() == 0)
        cloudTemp.intensity = 10;
    else
        cloudTemp.intensity = 50/*50 * (cloudTemp.z - p_frame->p_state->translation.z())*/;
    // cloudTemp.intensity = (double)point.getLabel();
    pcl_points->points.push_back(cloudTemp);
}


void lioOptimization::publish_odometry(const ros::Publisher & pubOdomAftMapped, cloudFrame *p_frame)
{
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(p_frame->p_state->rotation.z(), -p_frame->p_state->rotation.x(), -p_frame->p_state->rotation.y());

    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(p_frame->time_sweep_end);
    odomAftMapped.pose.pose.orientation.x = p_frame->p_state->rotation.x();
    odomAftMapped.pose.pose.orientation.y = p_frame->p_state->rotation.y();
    odomAftMapped.pose.pose.orientation.z = p_frame->p_state->rotation.z();
    odomAftMapped.pose.pose.orientation.w = p_frame->p_state->rotation.w();
    odomAftMapped.pose.pose.position.x = p_frame->p_state->translation.x();
    odomAftMapped.pose.pose.position.y = p_frame->p_state->translation.y();
    odomAftMapped.pose.pose.position.z = p_frame->p_state->translation.z();
    pubOdomAftMapped.publish(odomAftMapped);

    laserOdometryTrans.frame_id_ = "/camera_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom";
    laserOdometryTrans.stamp_ = ros::Time().fromSec(p_frame->time_sweep_end);
    laserOdometryTrans.setRotation(tf::Quaternion(p_frame->p_state->rotation.x(), 
                                                  p_frame->p_state->rotation.y(), 
                                                  p_frame->p_state->rotation.z(), 
                                                  p_frame->p_state->rotation.w()));
    laserOdometryTrans.setOrigin(tf::Vector3(p_frame->p_state->translation.x(), 
                                             p_frame->p_state->translation.y(), 
                                             p_frame->p_state->translation.z()));
    tfBroadcaster.sendTransform(laserOdometryTrans);
}

void lioOptimization::run()
{
    std::vector<std::pair<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<point3D>>, std::pair<double, double>>> measurements = getMeasurements();

    if(measurements.size() == 0) return;

    for (auto &measurement : measurements)
    {
        auto cut_sweep = measurement.first.second;

        double time_frame = measurement.second.first + measurement.second.second;
        double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;

        if (!initial_flag)
        {
            for (auto &imu_msg : measurement.first.first)
            {
                double time_imu = imu_msg->header.stamp.toSec();

                if (time_imu <= time_frame)
                { 
                    if(current_time < 0)
                        current_time = measurement.second.first;

                    current_time = time_imu;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    
                    imu_meas.emplace_back(current_time, std::make_pair(Eigen::Vector3d(rx, ry, rz), Eigen::Vector3d(dx, dy, dz)));
                }
                else
                {
                    double dt_1 = time_frame - current_time;
                    double dt_2 = time_imu - time_frame;
                    current_time = time_frame;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;

                    imu_meas.emplace_back(current_time, std::make_pair(Eigen::Vector3d(rx, ry, rz), Eigen::Vector3d(dx, dy, dz)));
                }
            }
            eskf_pro->tryInit(imu_meas);
            imu_meas.clear();

            last_time_frame = time_frame;

            std::vector<point3D>().swap(measurement.first.second);

            continue;
        }

        if (initial_flag)
        {
            imuState imu_state_temp;

            imu_state_temp.timestamp = current_time;

            imu_state_temp.un_acc = eskf_pro->getRotation().toRotationMatrix() * (eskf_pro->getLastAcc() - eskf_pro->getBa());
            imu_state_temp.un_gyr = eskf_pro->getLastGyr() - eskf_pro->getBg();
            imu_state_temp.trans = eskf_pro->getTranslation();
            imu_state_temp.quat = eskf_pro->getRotation();
            imu_state_temp.vel = eskf_pro->getVelocity();

            imu_states.push_back(imu_state_temp);
        }

        for (auto &imu_msg : measurement.first.first)
        {
            double time_imu = imu_msg->header.stamp.toSec();

            if (time_imu <= time_frame)
            { 
                if(current_time < 0)
                    current_time = measurement.second.first;
                double dt = time_imu - current_time;

                if(dt < -1e-6) continue;
                assert(dt >= 0);
                current_time = time_imu;
                dx = imu_msg->linear_acceleration.x;
                dy = imu_msg->linear_acceleration.y;
                dz = imu_msg->linear_acceleration.z;
                rx = imu_msg->angular_velocity.x;
                ry = imu_msg->angular_velocity.y;
                rz = imu_msg->angular_velocity.z;
                
                imuState imu_state_temp;

                imu_state_temp.timestamp = current_time;

                imu_state_temp.un_acc = eskf_pro->getRotation().toRotationMatrix() * (0.5 * (eskf_pro->getLastAcc() + Eigen::Vector3d(dx, dy, dz)) - eskf_pro->getBa());
                imu_state_temp.un_gyr = 0.5 * (eskf_pro->getLastGyr() + Eigen::Vector3d(rx, ry, rz)) - eskf_pro->getBg();

                dt_sum = dt_sum + dt;
                eskf_pro->predict(dt, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));

                imu_state_temp.trans = eskf_pro->getTranslation();
                imu_state_temp.quat = eskf_pro->getRotation();
                imu_state_temp.vel = eskf_pro->getVelocity();

                imu_states.push_back(imu_state_temp);
            }
            else
            {
                double dt_1 = time_frame - current_time;
                double dt_2 = time_imu - time_frame;
                current_time = time_frame;
                assert(dt_1 >= 0);
                assert(dt_2 >= 0);
                assert(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                rz = w1 * rz + w2 * imu_msg->angular_velocity.z;

                imuState imu_state_temp;

                imu_state_temp.timestamp = current_time;

                imu_state_temp.un_acc = eskf_pro->getRotation().toRotationMatrix() * (0.5 * (eskf_pro->getLastAcc() + Eigen::Vector3d(dx, dy, dz)) - eskf_pro->getBa());
                imu_state_temp.un_gyr = 0.5 * (eskf_pro->getLastGyr() + Eigen::Vector3d(rx, ry, rz)) - eskf_pro->getBg();

                dt_sum = dt_sum + dt_1;
                eskf_pro->predict(dt_1, Eigen::Vector3d(dx, dy, dz), Eigen::Vector3d(rx, ry, rz));

                imu_state_temp.trans = eskf_pro->getTranslation();
                imu_state_temp.quat = eskf_pro->getRotation();
                imu_state_temp.vel = eskf_pro->getVelocity();

                imu_states.push_back(imu_state_temp);
            }
        }

        process(cut_sweep, measurement.second.first, measurement.second.second);

        imu_states.clear();
        
        last_time_frame = time_frame;
        index_frame++;

        std::vector<point3D>().swap(measurement.first.second);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_optimization");
    ros::Time::init();
    
    lioOptimization LIO;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        LIO.run();

        rate.sleep();
    }

    return 0;
}

// loop closing
void lioOptimization::pubLocalMap(cloudFrame* p_frame)
{
    for (auto &point: p_frame->point_frame)
    {
        // if (point.is_dynamic == false)
        // {
            globalPoint global_point;
            global_point.setPosition(point.raw_point);
            global_point.setLabel(point.label);
            global_point.setDynamic(point.is_dynamic);
            addPointToPcl(loop_map, global_point);
        // }
    }
    publishCLoudWorld(pub_loop_map, loop_map, p_frame);
    loop_map->clear();
}
// loop closing