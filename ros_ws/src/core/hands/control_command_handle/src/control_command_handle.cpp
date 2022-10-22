#include "control_command_handle.h"

ControlCommandHandle::ControlCommandHandle(ros::NodeHandle &public_nh, ros::NodeHandle &private_nh, dc_server &m_dyn_cfg_server):
    public_nh_(public_nh), private_nh_(private_nh),  
    control_command_handle_visualization_(private_nh) {

    loadParameter(m_dyn_cfg_server);
    setupSubscriber();
    setupPublisher();
    setupServiceClients();

    ros::Rate rate(RATE_);

    srv_centerline_client_.waitForExistence();
    centerline_service::Centerline srv;

    if (srv_centerline_client_.call(srv)) {
        ROS_WARN_STREAM("[CCH] Success loading Centerline");
        centerline_ = srv.response.centerline;
    } else {
        ROS_WARN_STREAM("[CCH] Cannot call centerline from service");
    }

    fixedRateControlCall();
} 

void ControlCommandHandle::loadParameter(dc_server &m_dyn_cfg_server) {

    m_dyn_cfg_server.setCallback([&](control_command_handle::control_command_handle_Config& cfg, uint32_t) {
        speed_ = cfg.Speed;
        speed_mod_ = cfg.SpeedMod;
    });

    EGO_ID_ = ros::this_node::getNamespace();

    if (EGO_ID_ == "/") {
        EGO_ID_ = "";
    }

    ROS_WARN_STREAM("[CCM]: namespace: " << EGO_ID_);

    if (!private_nh_.param<std::string>("topics/subscribe/pose_topic", SUBSCRIBE_POSE_TOPIC_, "")) 
        ROS_WARN_STREAM("[CCH] Did not load pose topic.");
    if (!private_nh_.param<std::string>("topics/subscribe/odom_topic", SUBSCRIBE_ODOM_TOPIC_, "")) 
        ROS_WARN_STREAM("[CCH] Did not load odom topic.");
    if (!private_nh_.param<std::string>("topics/subscribe/obstacle_topic", SUBSCRIBE_OBSTACLE_TOPIC_, ""))
        ROS_WARN_STREAM("[CCH] Did not load obstacle_topic.");        
    if (!private_nh_.param<std::string>("topics/subscribe/scan_topic", SUBSCRIBE_SCAN_TOPIC_, ""))
        ROS_WARN_STREAM("[CCH] Did not load scan_topic.");    
    if (!private_nh_.param<std::string>("topics/subscribe/opp_trajectory_topic", SUBSCRIBE_OPP_TRAJECTORY_TOPIC_, ""))
        ROS_WARN_STREAM("[CCH] Did not load opp_trajectory_topic.");    
    if (!private_nh_.param<std::string>("services/centerline_service", CENTERLINE_SERVICE_, ""))
        ROS_WARN_STREAM("[CCH] Did not load centerline_service.");
    if (!private_nh_.param<std::string>("services/gap_follower_service", GAP_FOLLOWER_SERVICE_, ""))
        ROS_WARN_STREAM("[CCH] Did not load gap_follower_service.");
    if (!private_nh_.param<std::string>("services/pure_pursuit_service", PURE_PURSUIT_SERVICE_, ""))
        ROS_WARN_STREAM("[CCH] Did not load pure_pursuit_service.");
    if (!private_nh_.param<std::string>("services/stanley_controller_service", STANLEY_CONTROLLER_SERVICE_, ""))
        ROS_WARN_STREAM("[CCH] Did not load stanley_controller_service.");
    if (!private_nh_.param<std::string>("services/opp_traj_service", OPP_TRAJ_SERVICE_, ""))
        ROS_WARN_STREAM("[CCH] Did not load opp_traj_service.");
    if (!private_nh_.param<std::string>("services/pid_distance_service", PID_DISTANCE_SERVICE_, ""))
        ROS_WARN_STREAM("[CCH] Did not load pid_distance_service.");
    if (!private_nh_.param<bool>("params/use_stanley", USE_STANLEY_, ""))
        ROS_WARN_STREAM("[CCH] Did not load use_stanley.");
    if (!private_nh_.param<double>("params/speed", speed_, 0.0))
        ROS_WARN_STREAM("[CCH] Did not load speed.");    
    if (!private_nh_.param<double>("params/speed_mod", speed_mod_, 0.0))
        ROS_WARN_STREAM("[CCH] Did not load speed_mod."); 
    if (!private_nh_.param<int>("params/crossover", crossover_, 10))
        ROS_WARN_STREAM("[CCH] Did not load crossover."); 
    if (!private_nh_.param<std::string>("topics/publish/pp_topic", PUBLISH_FINAL_TOPIC_, ""))
        ROS_WARN_STREAM("[CCH] Did not load pp topic.");
    if (!private_nh_.param<bool>("visualize", visualize_, false))
        ROS_WARN_STREAM("[CCH] Did not load visualize boolean.");    
    if (!private_nh_.param<int>("params/rate", RATE_, 10)) 
        ROS_WARN_STREAM("[PID] Did not load rate");
}

void ControlCommandHandle::setupSubscriber() {

    pose_subscriber_ =
        public_nh_.subscribe(
            SUBSCRIBE_POSE_TOPIC_, 1,
            &ControlCommandHandle::poseCallback, this);    

    odom_subscriber_ =
        public_nh_.subscribe(
            SUBSCRIBE_ODOM_TOPIC_, 1,
            &ControlCommandHandle::odomCallback, this);    

    obstacle_subscriber_ =
        public_nh_.subscribe(
            SUBSCRIBE_OBSTACLE_TOPIC_, 1,
            &ControlCommandHandle::obstacleCallback, this
        );   
    scan_subscriber_ =
        public_nh_.subscribe(
            SUBSCRIBE_SCAN_TOPIC_, 1,
            &ControlCommandHandle::scanCallback, this
        );

    opp_trajectory_subscriber_ =
        public_nh_.subscribe(
            SUBSCRIBE_OPP_TRAJECTORY_TOPIC_, 1,
            &ControlCommandHandle::oppTrajectoryCallback, this
        );
}

void ControlCommandHandle::setupPublisher() {
    command_publisher_ = 
        public_nh_.advertise<f1tenth_gym_agent::drive_param>(
            PUBLISH_FINAL_TOPIC_, 1, true
        );
}

void ControlCommandHandle::setupServiceClients() {

    srv_gap_follower_client_    = public_nh_.serviceClient<hands_srv::Gap_Follower>(GAP_FOLLOWER_SERVICE_);
    srv_pure_pursuit_client_    = public_nh_.serviceClient<hands_srv::Pure_Pursuit>(PURE_PURSUIT_SERVICE_);
    srv_centerline_client_      = public_nh_.serviceClient<centerline_service::Centerline>(CENTERLINE_SERVICE_);
    // srv_opp_traj_client_        = public_nh_.serviceClient<hands_srv::Opp_Trajectory>(OPP_TRAJ_SERVICE_);
    srv_pid_distance_client_    = public_nh_.serviceClient<hands_srv::PID_Distance>(PID_DISTANCE_SERVICE_);
    srv_stanley_controller_client_    = public_nh_.serviceClient<hands_srv::Stanley_Controller>(STANLEY_CONTROLLER_SERVICE_);
}

void ControlCommandHandle::obstacleCallback(const eyes_msgs::SegmentList& segmentList) {
    ROS_WARN_STREAM("[CCH] Retrieved obstacles");
    obstacles_ = segmentList;
}

void ControlCommandHandle::scanCallback(const sensor_msgs::LaserScan& scan) {
    scan_ = scan;
}

void ControlCommandHandle::oppTrajectoryCallback(const hands_msgs::OpponentTrajectory& trajectory) {
    opp_trajectory_ = trajectory;
}

void ControlCommandHandle::odomCallback(const nav_msgs::Odometry& odom) {
    odom_ = odom;
}

// TODO: Don't do action in callback, use frequency for stability
void ControlCommandHandle::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_stamped) {
        pose_stamped_ = pose_stamped;
    }

void ControlCommandHandle::controlCall() {
    
    if (EGO_ID_=="/a1") {

        // if(obstacles_.segments.size() != 0) {
        //     ROS_WARN_STREAM("[CCH] Using gap follower as " << ros::this_node::getNamespace().c_str());
        
        //     gap_follower_srv_.request.scan = scan_;

        //     if (scan_.ranges.size() == 0) {
        //         ROS_WARN_STREAM_THROTTLE(1, "[CCM] No Scan");
        //         final_drive_param_.velocity = 0.3;
        //         final_drive_param_.angle = 0.0;
        //         command_publisher_.publish(final_drive_param_);
        //         return;
        //     }

        //     std_msgs::Float32 vel;
        //     std_msgs::Float32 steer;
        //     if (srv_gap_follower_client_.call(gap_follower_srv_)) {
        //         ROS_WARN_STREAM_ONCE("[CCH] Succesfull loaded gap follower cc");
        //         vel = gap_follower_srv_.response.velocity;
        //         steer = gap_follower_srv_.response.steering_angle;

        //         final_drive_param_.velocity = vel.data; // * speed_mod_;
        //         final_drive_param_.angle = steer.data;
        //     } else {
        //         ROS_WARN_STREAM("[CCH] Cannot call gap follower follower from service");
        //         final_drive_param_.velocity = 0.0;
        //         final_drive_param_.angle = 0.0;
        //     }
        // }

        // command_publisher_.publish(final_drive_param_);

        // Prepare data for pp algorithm and pid
        bool follow_opponent = (opp_trajectory_.trajectory.polygon.points.size() >= crossover_);
        pure_pursuit_srv_.request.pose = pose_stamped_;
        stanley_controller_srv_.request.pose = pose_stamped_;
        if (follow_opponent) {
            //opponent found, follow it
            ROS_WARN("[CCH] %s Following opponent", ros::this_node::getNamespace().c_str());
            pure_pursuit_srv_.request.trajectory = opp_trajectory_.trajectory;
            stanley_controller_srv_.request.trajectory = opp_trajectory_.trajectory;
            std::reverse(stanley_controller_srv_.request.trajectory.polygon.points.begin(), stanley_controller_srv_.request.trajectory.polygon.points.end());
            pid_distance_srv_.request.distance = opp_trajectory_.distance;

            pid_distance_srv_.request.velocity.data = odom_.twist.twist.linear.x;

        } else {
            ROS_WARN("[CCH] %s Following centerline", ros::this_node::getNamespace().c_str());

            pure_pursuit_srv_.request.trajectory = geometry_msgs::PolygonStamped();
            stanley_controller_srv_.request.trajectory = geometry_msgs::PolygonStamped();
        }
        
        std_msgs::Float32 old_steer;
        old_steer.data = final_drive_param_.angle;
        pure_pursuit_srv_.request.old_angle = old_steer;
        pure_pursuit_srv_.request.target_velocity.data = odom_.twist.twist.linear.x;

        std_msgs::Float32 steer;
        bool error = false;

        // set steering angle
        if (USE_STANLEY_) {
            stanley_controller_srv_.request.old_angle = old_steer;
            stanley_controller_srv_.request.target_velocity.data = odom_.twist.twist.linear.x;

            if (srv_stanley_controller_client_.call(stanley_controller_srv_)) {
            steer = stanley_controller_srv_.response.steering_angle;
            ROS_WARN_STREAM(EGO_ID_ << " [CCH] SC - steering angle = " << steer.data / 180.0 * M_PI << "   velocity: " << stanley_controller_srv_.response.speed.data);

            final_drive_param_.angle = steer.data;
            final_drive_param_.velocity = stanley_controller_srv_.response.speed.data;

            } else {
                ROS_WARN_STREAM("[CCH] Cannot call stanley controller from service");
                error = true;
            }
        }
        else {
            if (srv_pure_pursuit_client_.call(pure_pursuit_srv_)) {
            steer = pure_pursuit_srv_.response.steering_angle;
            ROS_WARN_STREAM(EGO_ID_ << " [CCH] PP - steering angle = " << steer.data / 180.0 * M_PI << "   velocity: " << pure_pursuit_srv_.response.speed.data);

            final_drive_param_.angle = steer.data;
            final_drive_param_.velocity = pure_pursuit_srv_.response.speed.data;

            } else {
                ROS_WARN_STREAM("[CCH] Cannot call pure pursuit from service");
                error = true;
            }
        }

        // set velocity angle
        if (follow_opponent) {
            ROS_WARN("[CCH] %s Calling PID with: distnace: %f, velocity: %f", ros::this_node::getNamespace().c_str(), pid_distance_srv_.request.distance.data, pid_distance_srv_.request.velocity.data);
            //opponent found, get velocity to hold distance
            if (srv_pid_distance_client_.call(pid_distance_srv_)) {
                ROS_WARN_STREAM_ONCE("[CCH] Succesfull loaded pid cc");

                final_drive_param_.velocity = pid_distance_srv_.response.velocity.data;

                ROS_WARN_STREAM("[CCH] Velocity = " << final_drive_param_.velocity);
            } else {
                ROS_WARN_STREAM("[CCH] Cannot call pid from service");
                error = true;
            }
        } 

        // emergency breake iff error occures
        if (error) {
            final_drive_param_.angle = 0.0;
            final_drive_param_.velocity = 0.0;
        }

        command_publisher_.publish(final_drive_param_);
    } else {

        // Prepare data for pp algorithm and pid
        pure_pursuit_srv_.request.pose = pose_stamped_;
        
        if (opp_trajectory_.trajectory.polygon.points.size() > 0) {
            //opponent found, follow it
            ROS_WARN("[CCH] %s Following opponent", ros::this_node::getNamespace().c_str());
            pure_pursuit_srv_.request.trajectory = opp_trajectory_.trajectory;
        } else {
            ROS_WARN("[CCH] %s Following centerline", ros::this_node::getNamespace().c_str());

            // pure_pursuit_srv_.request.trajectory = centerline_;
            pure_pursuit_srv_.request.trajectory = geometry_msgs::PolygonStamped();
        }
        
        std_msgs::Float32 old_steer;
        old_steer.data = final_drive_param_.angle;
        pure_pursuit_srv_.request.old_angle = old_steer;
        pure_pursuit_srv_.request.target_velocity.data = odom_.twist.twist.linear.x;

        std_msgs::Float32 steer;
        bool error = false;

        // set steering angle
        if (srv_pure_pursuit_client_.call(pure_pursuit_srv_)) {
            ROS_WARN_STREAM_ONCE("[CCH] Succesfull loaded pure pursuit cc");
            steer = pure_pursuit_srv_.response.steering_angle;
            ROS_WARN_STREAM("[CCH] Steer = " << steer);

            final_drive_param_.angle = steer.data;
            final_drive_param_.velocity = pure_pursuit_srv_.response.speed.data * 0.8;
        } else {
            ROS_WARN_STREAM("[CCH] Cannot call pure pursuit from service");
            error = true;
        }

        // emergency breake iff error occures
        if (error) {
            final_drive_param_.angle = 0.0;
            final_drive_param_.velocity = 0.0;
        }

        command_publisher_.publish(final_drive_param_);
    }
}

    void ControlCommandHandle::fixedRateControlCall() {
        ros::Rate rate(RATE_);
        while(ros::ok()) {
            rate.sleep();
            controlCall();
            ros::spinOnce();
        }
}

