#include "fast_lio_sam/main.h"
#include "fast_lio_sam/utilities.hpp"

pose_pcd::pose_pcd(const nav_msgs::msg::Odometry &odom_in, const sensor_msgs::msg::PointCloud2 &pcd_in, const int &idx_in)
{
  tf2::Quaternion q_(odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y, odom_in.pose.pose.orientation.z, odom_in.pose.pose.orientation.w);
  tf2::Matrix3x3 m_(q_);
  pose_eig(0, 0) = m_[0][0]; pose_eig(0, 1) = m_[0][1]; pose_eig(0, 2) = m_[0][2];
  pose_eig(1, 0) = m_[1][0]; pose_eig(1, 1) = m_[1][1]; pose_eig(1, 2) = m_[1][2];
  pose_eig(2, 0) = m_[2][0]; pose_eig(2, 1) = m_[2][1]; pose_eig(2, 2) = m_[2][2];
  pose_eig(0, 3) = odom_in.pose.pose.position.x;
  pose_eig(1, 3) = odom_in.pose.pose.position.y;
  pose_eig(2, 3) = odom_in.pose.pose.position.z;
  pose_corrected_eig = pose_eig;
  pcl::PointCloud<pcl::PointXYZI> tmp_pcd_;
  pcl::fromROSMsg(pcd_in, tmp_pcd_);
  pcd = tf_pcd(tmp_pcd_, pose_eig.inverse()); //FAST-LIO publish data in world frame, so save it in LiDAR frame
  timestamp = rclcpp::Time(odom_in.header.stamp).seconds();
  idx = idx_in;
}

FAST_LIO_SAM_CLASS::FAST_LIO_SAM_CLASS(const rclcpp::Node::SharedPtr node_ptr) : m_node_ptr(node_ptr)
{
  ////// ROS params
  // temp vars
  double loop_update_hz_, vis_hz_;
  // get params
  m_node_ptr->declare_parameter<string>("map_frame", "map");
  m_node_ptr->declare_parameter<double>("keyframe_threshold", 1.0);
  m_node_ptr->declare_parameter<double>("loop_detection_radius", 15.0);
  m_node_ptr->declare_parameter<double>("loop_detection_timediff_threshold", 10.0);
  m_node_ptr->declare_parameter<double>("icp_score_threshold", 10.0);
  m_node_ptr->declare_parameter<int>("subkeyframes_number", 5);
  m_node_ptr->declare_parameter<double>("loop_update_hz", 1.0);
  m_node_ptr->declare_parameter<double>("vis_hz", 0.5);

  m_node_ptr->declare_parameter<bool>("result.save_map_bag", false);
  m_node_ptr->declare_parameter<bool>("result.save_map_pcd", false);

  m_node_ptr->get_parameter("map_frame", m_map_frame);
  m_node_ptr->get_parameter("keyframe_threshold", m_keyframe_thr);
  m_node_ptr->get_parameter("loop_detection_radius", m_loop_det_radi);
  m_node_ptr->get_parameter("loop_detection_timediff_threshold", m_loop_det_tdiff_thr);
  m_node_ptr->get_parameter("icp_score_threshold", m_icp_score_thr);
  m_node_ptr->get_parameter("subkeyframes_number", m_sub_key_num);
  m_node_ptr->get_parameter("loop_update_hz", loop_update_hz_);
  m_node_ptr->get_parameter("vis_hz", vis_hz_);

  m_node_ptr->get_parameter("result.save_map_bag", m_save_map_bag);
  m_node_ptr->get_parameter("result.save_map_pcd", m_save_map_pcd);

  ////// GTSAM init
  gtsam::ISAM2Params isam_params_;
  isam_params_.relinearizeThreshold = 0.01;
  isam_params_.relinearizeSkip = 1;
  m_isam_handler = std::make_shared<gtsam::ISAM2>(isam_params_);
  ////// loop init
  m_voxelgrid.setLeafSize(0.3, 0.3, 0.3);
  m_voxelgrid_vis.setLeafSize(0.2, 0.2, 0.2);
  m_icp.setMaxCorrespondenceDistance(m_loop_det_radi*2.0);
  m_icp.setTransformationEpsilon(1e-2);
  m_icp.setEuclideanFitnessEpsilon(1e-2);
  m_icp.setMaximumIterations(100);
  m_icp.setRANSACIterations(0);

  ////// ROS things
  m_odom_path.header.frame_id = m_map_frame;
  m_corrected_path.header.frame_id = m_map_frame;
  m_package_path = ament_index_cpp::get_package_share_directory("fast_lio_sam");
  // publishers
  m_odom_pub = m_node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>("/ori_odom", rclcpp::QoS(10).reliable().transient_local());
  m_path_pub = m_node_ptr->create_publisher<nav_msgs::msg::Path>("/ori_path", rclcpp::QoS(10).reliable().transient_local());
  m_corrected_odom_pub = m_node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_odom", rclcpp::QoS(10).reliable().transient_local());
  m_corrected_path_pub = m_node_ptr->create_publisher<nav_msgs::msg::Path>("/corrected_path", rclcpp::QoS(10).reliable().transient_local());
  m_corrected_pcd_map_pub = m_node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_map", rclcpp::QoS(10).reliable().transient_local());
  m_corrected_current_pcd_pub = m_node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_current_pcd", rclcpp::QoS(10).reliable().transient_local());
  m_loop_detection_pub = m_node_ptr->create_publisher<visualization_msgs::msg::Marker>("/loop_detection", rclcpp::QoS(10).reliable().transient_local());
  m_realtime_pose_pub = m_node_ptr->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_stamped", rclcpp::QoS(10).reliable().transient_local());
  m_debug_src_pub = m_node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>("/src", rclcpp::QoS(10).reliable().transient_local());
  m_debug_dst_pub = m_node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>("/dst", rclcpp::QoS(10).reliable().transient_local());
  m_debug_aligned_pub = m_node_ptr->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned", rclcpp::QoS(10).reliable().transient_local());
  // subscribers
  m_callback_group = m_node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = m_callback_group;

  m_sub_odom = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(m_node_ptr, "/Odometry", rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opt);
  m_sub_pcd = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(m_node_ptr, "/cloud_registered", rclcpp::QoS(10).reliable().get_rmw_qos_profile(), sub_opt);
  m_sub_odom_pcd_sync = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *m_sub_odom, *m_sub_pcd);
  m_sub_odom_pcd_sync->registerCallback(std::bind(&FAST_LIO_SAM_CLASS::odom_pcd_cb, this, std::placeholders::_1, std::placeholders::_2));
  // Timers at the end
  m_loop_timer = rclcpp::create_timer(m_node_ptr, m_node_ptr->get_clock(), rclcpp::Duration::from_seconds(1/loop_update_hz_), std::bind(&FAST_LIO_SAM_CLASS::loop_timer_func, this), m_callback_group);
  m_vis_timer = rclcpp::create_timer(m_node_ptr, m_node_ptr->get_clock(), rclcpp::Duration::from_seconds(1/vis_hz_), std::bind(&FAST_LIO_SAM_CLASS::vis_timer_func, this), m_callback_group);

  m_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(m_node_ptr);
  
  RCLCPP_WARN(m_node_ptr->get_logger(), "Main class, starting node...");
}

FAST_LIO_SAM_CLASS::~FAST_LIO_SAM_CLASS()
{
  // save map
  if (m_save_map_bag)
  {
    auto bag_ = std::make_unique<rosbag2_cpp::Writer>();
    bag_->open(m_package_path+"/result_bag");
    {
      lock_guard<mutex> lock(m_keyframes_mutex);
      for (int i = 0; i < m_keyframes.size(); ++i)
      {
        rclcpp::Time time_(static_cast<int64_t>(m_keyframes[i].timestamp * 1e9));
        bag_->write(pcl_to_pcl_ros(m_keyframes[i].pcd, m_map_frame), "/keyframe_pcd", time_);
        bag_->write(pose_eig_to_pose_stamped(m_keyframes[i].pose_corrected_eig), "/keyframe_pose", time_);
      }
    }
    bag_->close();

    std::cout << "\033[36;1mResult saved in .bag format!!!\033[0m" << std::endl;
  }
  if (m_save_map_pcd)
  {
    pcl::PointCloud<pcl::PointXYZI> corrected_map_;
    {
      lock_guard<mutex> lock(m_keyframes_mutex);
      for (int i = 0; i < m_keyframes.size(); ++i)
      {
        corrected_map_ += tf_pcd(m_keyframes[i].pcd, m_keyframes[i].pose_corrected_eig);
      }
    }
    pcl::io::savePCDFileASCII<pcl::PointXYZI> (m_package_path+"/result.pcd", corrected_map_);
    std::cout << "\033[32;1mResult saved in .pcd format!!!\033[0m" << std::endl;
  }
}

void FAST_LIO_SAM_CLASS::odom_pcd_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg)
{
  Eigen::Matrix4d last_odom_tf_;
  last_odom_tf_ = m_current_frame.pose_eig; //to calculate delta
  m_current_frame = pose_pcd(*odom_msg, *pcd_msg, m_current_keyframe_idx); //to be checked if keyframe or not

  if (!m_init) //// init only once
  {
    //others
    m_keyframes.push_back(m_current_frame);
    update_vis_vars(m_current_frame);
    m_corrected_current_pcd_pub->publish(pcl_to_pcl_ros(m_current_frame.pcd, m_map_frame));
    //graph
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished()); // rad*rad, meter*meter
    m_gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose_eig_to_gtsam_pose(m_current_frame.pose_eig), prior_noise_));
    m_init_esti.insert(m_current_keyframe_idx, pose_eig_to_gtsam_pose(m_current_frame.pose_eig));
    {
      lock_guard<mutex> lock(m_realtime_pose_mutex);
      m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_frame.pose_eig;
      m_current_frame.pose_corrected_eig = m_last_corrected_pose * m_odom_delta;
      geometry_msgs::msg::PoseStamped current_pose_stamped_ = pose_eig_to_pose_stamped(m_current_frame.pose_corrected_eig, m_map_frame);
      m_realtime_pose_pub->publish(current_pose_stamped_);
      tf2::Transform transform_;
      transform_.setOrigin(tf2::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
      transform_.setRotation(tf2::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));

      geometry_msgs::msg::TransformStamped trans;
      trans.header.frame_id = m_map_frame;
      trans.header.stamp = odom_msg->header.stamp;
      trans.child_frame_id = "robot";
      trans.transform = tf2::toMsg(transform_);
      m_broadcaster->sendTransform(trans);
    }
    m_current_keyframe_idx++;
    m_init = true;
  }
  else
  {
    //// 1. realtime pose = last corrected odom * delta (last -> current)
    high_resolution_clock::time_point t1_ = high_resolution_clock::now();
    {
      lock_guard<mutex> lock(m_realtime_pose_mutex);
      m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_frame.pose_eig;
      m_current_frame.pose_corrected_eig = m_last_corrected_pose * m_odom_delta;
      geometry_msgs::msg::PoseStamped current_pose_stamped_ = pose_eig_to_pose_stamped(m_current_frame.pose_corrected_eig, m_map_frame);
      m_realtime_pose_pub->publish(current_pose_stamped_);
      tf2::Transform transform_;
      transform_.setOrigin(tf2::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
      transform_.setRotation(tf2::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
      
      geometry_msgs::msg::TransformStamped trans;
      trans.header.frame_id = m_map_frame;
      trans.header.stamp = odom_msg->header.stamp;
      trans.child_frame_id = "robot";
      trans.transform = tf2::toMsg(transform_);
      m_broadcaster->sendTransform(trans);
    }
    // pub current scan in corrected pose frame
    m_corrected_current_pcd_pub->publish(pcl_to_pcl_ros(tf_pcd(m_current_frame.pcd, m_current_frame.pose_corrected_eig), m_map_frame));

    //// 2. check if keyframe
    high_resolution_clock::time_point t2_ = high_resolution_clock::now();
    if (check_if_keyframe(m_current_frame, m_keyframes.back()))
    {
      // 2-2. if so, save
      {
        lock_guard<mutex> lock(m_keyframes_mutex);
        m_keyframes.push_back(m_current_frame);
        m_not_processed_keyframe = m_current_frame; //to check loop in another thread
      }
      // 2-3. if so, add to graph
      gtsam::noiseModel::Diagonal::shared_ptr odom_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished());
      gtsam::Pose3 pose_from_ = pose_eig_to_gtsam_pose(m_keyframes[m_current_keyframe_idx-1].pose_corrected_eig);
      gtsam::Pose3 pose_to_ = pose_eig_to_gtsam_pose(m_current_frame.pose_corrected_eig);
      {
        lock_guard<mutex> lock(m_graph_mutex);
        m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(m_current_keyframe_idx-1, m_current_keyframe_idx, pose_from_.between(pose_to_), odom_noise_));
        m_init_esti.insert(m_current_keyframe_idx, pose_to_);
      }
      m_current_keyframe_idx++;

      //// 3. vis
      high_resolution_clock::time_point t3_ = high_resolution_clock::now();
      {
        lock_guard<mutex> lock(m_vis_mutex);
        update_vis_vars(m_current_frame);
      }

      //// 4. optimize with graph
      high_resolution_clock::time_point t4_ = high_resolution_clock::now();
      // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, m_init_esti).optimize(); // cf. isam.update vs values.LM.optimize
      {
        lock_guard<mutex> lock(m_graph_mutex);
        m_isam_handler->update(m_gtsam_graph, m_init_esti);
        m_isam_handler->update();
        if (m_loop_added_flag) //https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
        {
          m_isam_handler->update();
          m_isam_handler->update();
          m_isam_handler->update();
        }
        m_gtsam_graph.resize(0);
        m_init_esti.clear();
      }

      //// 5. handle corrected results
      // get corrected poses and reset odom delta (for realtime pose pub)
      high_resolution_clock::time_point t5_ = high_resolution_clock::now();
      {
        lock_guard<mutex> lock(m_realtime_pose_mutex);
        m_corrected_esti = m_isam_handler->calculateEstimate();
        m_last_corrected_pose = gtsam_pose_to_pose_eig(m_corrected_esti.at<gtsam::Pose3>(m_corrected_esti.size()-1));
        m_odom_delta = Eigen::Matrix4d::Identity();
      }
      // correct poses in keyframes
      if (m_loop_added_flag)
      {
        lock_guard<mutex> lock(m_keyframes_mutex);
        for (int i = 0; i < m_corrected_esti.size(); ++i)
        {
          m_keyframes[i].pose_corrected_eig = gtsam_pose_to_pose_eig(m_corrected_esti.at<gtsam::Pose3>(i));
        }
        m_loop_added_flag = false;
      }
      high_resolution_clock::time_point t6_ = high_resolution_clock::now();

      RCLCPP_INFO(m_node_ptr->get_logger(), "read: %.1f, key_add: %.1f, vis: %.1f, opt: %.1f, res: %.1f, tot: %.1fms", 
        duration_cast<microseconds>(t2_-t1_).count()/1e3, duration_cast<microseconds>(t3_-t2_).count()/1e3,
        duration_cast<microseconds>(t4_-t3_).count()/1e3, duration_cast<microseconds>(t5_-t4_).count()/1e3,
        duration_cast<microseconds>(t6_-t5_).count()/1e3, duration_cast<microseconds>(t6_-t1_).count()/1e3);
    }
  }
  return;
}

void FAST_LIO_SAM_CLASS::loop_timer_func()
{
  if (!m_init) return;

  //// 1. copy keyframes and not processed keyframes
  high_resolution_clock::time_point t1_ = high_resolution_clock::now();
  pose_pcd not_proc_key_copy_;
  vector<pose_pcd> keyframes_copy_;
  {
    lock_guard<mutex> lock(m_keyframes_mutex);
    keyframes_copy_ = m_keyframes;
    not_proc_key_copy_ = m_not_processed_keyframe;
    m_not_processed_keyframe.processed = true;
  }
  if (not_proc_key_copy_.processed) return; //already processed

  //// 2. detect loop and add to graph
  high_resolution_clock::time_point t2_ = high_resolution_clock::now();
  bool if_loop_occured_ = false;
  // from not_proc_key_copy_ keyframe to old keyframes in threshold radius, get the closest keyframe
  int closest_keyframe_idx_ = get_closest_keyframe_idx(not_proc_key_copy_, keyframes_copy_);
  if (closest_keyframe_idx_ >= 0) //if exists
  {
    // ICP to check loop (from front_keyframe to closest keyframe's neighbor)
    icp_key_to_subkeys(not_proc_key_copy_, closest_keyframe_idx_, keyframes_copy_);
    double score_ = m_icp.getFitnessScore();
    cout << score_ << endl;
    // if matchness score is lower than threshold, (lower is better)
    if(m_icp.hasConverged() && score_ < m_icp_score_thr) // add loop factor
    {
      Eigen::Matrix4d pose_between_eig_ = m_icp.getFinalTransformation().cast<double>();
      gtsam::Pose3 pose_from_ = pose_eig_to_gtsam_pose(pose_between_eig_ * not_proc_key_copy_.pose_corrected_eig);
      gtsam::Pose3 pose_to_ = pose_eig_to_gtsam_pose(keyframes_copy_[closest_keyframe_idx_].pose_corrected_eig);
      gtsam::noiseModel::Diagonal::shared_ptr loop_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << score_, score_, score_, score_, score_, score_).finished());
      {
        lock_guard<mutex> lock(m_graph_mutex);
        m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(not_proc_key_copy_.idx, closest_keyframe_idx_, pose_from_.between(pose_to_), loop_noise_));
      }
      m_loop_idx_pairs.push_back({not_proc_key_copy_.idx, closest_keyframe_idx_}); //for vis
      if_loop_occured_ = true;
    }
  }
  high_resolution_clock::time_point t3_ = high_resolution_clock::now();

  if (if_loop_occured_)
  {
    m_loop_added_flag_vis = true;
    m_loop_added_flag = true;
  }

  RCLCPP_INFO(m_node_ptr->get_logger(), "copy: %.1f, loop: %.1f", 
          duration_cast<microseconds>(t2_-t1_).count()/1e3, duration_cast<microseconds>(t3_-t2_).count()/1e3);

  return;
}

void FAST_LIO_SAM_CLASS::vis_timer_func()
{
  if (!m_init) return;

  high_resolution_clock::time_point tv1_ = high_resolution_clock::now();
  //// 1. if loop closed, correct vis data
  if (m_loop_added_flag_vis) 
  {
    // copy and ready
    gtsam::Values corrected_esti_copy_;
    pcl::PointCloud<pcl::PointXYZ> corrected_odoms_;
    nav_msgs::msg::Path corrected_path_;
    {
      lock_guard<mutex> lock(m_realtime_pose_mutex);
      corrected_esti_copy_ = m_corrected_esti;
    }
    // correct pose and path
    for (int i = 0; i < corrected_esti_copy_.size(); ++i)
    {
      gtsam::Pose3 pose_ = corrected_esti_copy_.at<gtsam::Pose3>(i);
      corrected_odoms_.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
      corrected_path_.poses.push_back(gtsam_pose_to_pose_stamped(pose_, m_map_frame));
    }
    // update vis of loop constraints
    if (!m_loop_idx_pairs.empty())
    {
      m_loop_detection_pub->publish(get_loop_markers(corrected_esti_copy_));
    }
    // update with corrected data
    {
      lock_guard<mutex> lock(m_vis_mutex);
      m_corrected_odoms = corrected_odoms_;
      m_corrected_path.poses = corrected_path_.poses;
    }
    m_loop_added_flag_vis = false;
  }

  //// 2. publish odoms, paths
  {
    lock_guard<mutex> lock(m_vis_mutex);
    m_odom_pub->publish(pcl_to_pcl_ros(m_odoms, m_map_frame));
    m_path_pub->publish(m_odom_path);
    m_corrected_odom_pub->publish(pcl_to_pcl_ros(m_corrected_odoms, m_map_frame));
    m_corrected_path_pub->publish(m_corrected_path);
  }

  //// 3. global map
  if (m_global_map_vis_switch && m_corrected_pcd_map_pub->get_subscription_count() > 0) //save time, only once
  {
    pcl::PointCloud<pcl::PointXYZI> corrected_map_;
    {
      lock_guard<mutex> lock(m_keyframes_mutex);
      for (int i = 0; i < m_keyframes.size(); ++i)
      {
        corrected_map_ += tf_pcd(m_keyframes[i].pcd, m_keyframes[i].pose_corrected_eig);
      }
    }
    voxelize_pcd(m_voxelgrid_vis, corrected_map_);
    m_corrected_pcd_map_pub->publish(pcl_to_pcl_ros(corrected_map_, m_map_frame));
    m_global_map_vis_switch = false;
  }
  if (!m_global_map_vis_switch && m_corrected_pcd_map_pub->get_subscription_count() == 0)
  {
    m_global_map_vis_switch = true;      
  }
  high_resolution_clock::time_point tv2_ = high_resolution_clock::now();
  RCLCPP_INFO(m_node_ptr->get_logger(), "vis: %.1fms", duration_cast<microseconds>(tv2_-tv1_).count()/1e3);
  return;
}

void FAST_LIO_SAM_CLASS::update_vis_vars(const pose_pcd &pose_pcd_in)
{
  m_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  m_corrected_odoms.points.emplace_back(pose_pcd_in.pose_corrected_eig(0, 3), pose_pcd_in.pose_corrected_eig(1, 3), pose_pcd_in.pose_corrected_eig(2, 3));
  m_odom_path.poses.push_back(pose_eig_to_pose_stamped(pose_pcd_in.pose_eig, m_map_frame));
  m_corrected_path.poses.push_back(pose_eig_to_pose_stamped(pose_pcd_in.pose_corrected_eig, m_map_frame));
  return;
}

visualization_msgs::msg::Marker FAST_LIO_SAM_CLASS::get_loop_markers(const gtsam::Values &corrected_esti_in)
{
  visualization_msgs::msg::Marker edges_; edges_.type = 5u;
  edges_.scale.x = 0.12f; edges_.header.frame_id = m_map_frame; edges_.pose.orientation.w = 1.0f;
  edges_.color.r = 1.0f; edges_.color.g = 1.0f; edges_.color.b = 1.0f; edges_.color.a = 1.0f;
  for (int i = 0; i < m_loop_idx_pairs.size(); ++i)
  {
    if (m_loop_idx_pairs[i].first >= corrected_esti_in.size() || m_loop_idx_pairs[i].second >= corrected_esti_in.size()) continue;
    gtsam::Pose3 pose_ = corrected_esti_in.at<gtsam::Pose3>(m_loop_idx_pairs[i].first);
    gtsam::Pose3 pose2_ = corrected_esti_in.at<gtsam::Pose3>(m_loop_idx_pairs[i].second);
    geometry_msgs::msg::Point p_, p2_;
    p_.x = pose_.translation().x(); p_.y = pose_.translation().y(); p_.z = pose_.translation().z();
    p2_.x = pose2_.translation().x(); p2_.y = pose2_.translation().y(); p2_.z = pose2_.translation().z();
    edges_.points.push_back(p_);
    edges_.points.push_back(p2_);
  }
  return edges_;
}

void FAST_LIO_SAM_CLASS::voxelize_pcd(pcl::VoxelGrid<pcl::PointXYZI> &voxelgrid, pcl::PointCloud<pcl::PointXYZI> &pcd_in)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr before_(new pcl::PointCloud<pcl::PointXYZI>);
  *before_ = pcd_in;
  voxelgrid.setInputCloud(before_);
  voxelgrid.filter(pcd_in);
  return;
}

bool FAST_LIO_SAM_CLASS::check_if_keyframe(const pose_pcd &pose_pcd_in, const pose_pcd &latest_pose_pcd)
{
  return m_keyframe_thr < (latest_pose_pcd.pose_corrected_eig.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig.block<3, 1>(0, 3)).norm();
}

int FAST_LIO_SAM_CLASS::get_closest_keyframe_idx(const pose_pcd &front_keyframe, const vector<pose_pcd> &keyframes)
{
  double shortest_distance_ = m_loop_det_radi*3.0;
  int closest_idx_ = -1;
  for (int idx = 0; idx < keyframes.size()-1; ++idx)
  {
    //check if potential loop: close enough in distance, far enough in time
    double tmp_dist_ = (keyframes[idx].pose_corrected_eig.block<3, 1>(0, 3) - front_keyframe.pose_corrected_eig.block<3, 1>(0, 3)).norm();
    if (m_loop_det_radi > tmp_dist_ && m_loop_det_tdiff_thr < (front_keyframe.timestamp - keyframes[idx].timestamp))
    {
      if (tmp_dist_ < shortest_distance_)
      {
        shortest_distance_ = tmp_dist_;
        closest_idx_ = keyframes[idx].idx;
      }
    }
  }
  return closest_idx_;
}

void FAST_LIO_SAM_CLASS::icp_key_to_subkeys(const pose_pcd &front_keyframe, const int &closest_idx, const vector<pose_pcd> &keyframes)
{
	// merge subkeyframes before ICP
  pcl::PointCloud<pcl::PointXYZI> dst_raw_, src_raw_;
  src_raw_ = tf_pcd(front_keyframe.pcd, front_keyframe.pose_corrected_eig);
  for (int i = closest_idx-m_sub_key_num; i < closest_idx+m_sub_key_num+1; ++i)
  {
    if (i>=0 && i < keyframes.size()-1) //if exists
    {
      dst_raw_ += tf_pcd(keyframes[i].pcd, keyframes[i].pose_corrected_eig);
    }
  }
  
  // voxlize pcd
  pcl::PointCloud<pcl::PointXYZI>::Ptr src_(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr dst_(new pcl::PointCloud<pcl::PointXYZI>);
  voxelize_pcd(m_voxelgrid, dst_raw_);
  voxelize_pcd(m_voxelgrid, src_raw_);
  *dst_ = dst_raw_;
  *src_ = src_raw_;

  // then match with ICP
  pcl::PointCloud<pcl::PointXYZI> dummy_;
  m_icp.setInputSource(src_);
  m_icp.setInputTarget(dst_);
  m_icp.align(dummy_);
  m_debug_src_pub->publish(pcl_to_pcl_ros(src_raw_, m_map_frame));
  m_debug_dst_pub->publish(pcl_to_pcl_ros(dst_raw_, m_map_frame));
  m_debug_aligned_pub->publish(pcl_to_pcl_ros(dummy_, m_map_frame));
	return;
}
