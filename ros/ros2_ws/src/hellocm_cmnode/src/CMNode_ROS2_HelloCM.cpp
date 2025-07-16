/*!
 ******************************************************************************
 **  CarMaker - Version 13.1.1
 **  Vehicle Dynamics Simulation Toolkit
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 * Description:
 * - Prototype/Proof of Concept
 * - Unsupported ROS Example with CarMaker
 * - Structure may change in future!
 * - Change general parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
 * - Basic communication with or without parameterizable synchronization
 *
 *
 * ToDo:
 * - C++!!!
 * - ROS naming/way/namespaces
 * - parameter: CarMaker read, ROS set by service?
 *   -> ROS parameter mechanism seems better solution!
 * - node/topic/... destruction to allow dynamic load/unload
 *   when TestRun starts instead of initialization at CarMaker startup
 * - New Param_Get() function to read parameters from Infofile
 * - ...
 *
 */
#include "CMNode_ROS2_HelloCM.hpp"

using cm_ros::CMNodeHelloCM;
using CMJob::Log;

CMNodeHelloCM::CMNodeHelloCM() {}

/* Lidar RSI --> Number of Beams */
static const unsigned int tableSize = 15000 * 6;
/* Lidar RSI PS150 --> Number of Beams */
static const unsigned int ps150_tableSize = 150 * 6;
/* Lidar RSI Pandar40p --> Number of Beams */
static const unsigned int pandar40p_tableSize = 54000 * 6;

static const unsigned int seed = static_cast<unsigned int>(time(nullptr)); // Initialize seed

/* NDEBUG is set in CarMaker Makefile/MakeDefs in OPT_CFLAGS */
#if !defined NDEBUG
#  define DBLOG LOG
#else
#  define DBLOG(...)
#endif

/* Define a struct for the CameraRSI data */
typedef struct tCameraRSI {
    char*           name;
    int             number;
    double*         pos;                /*!< Mounting position on vehicle frame */
    double*         rot;                /*!< Mounting rotation on vehicle frame */
    double          FoV;
    double*         Resolution;         /*!< Resolution width and height */
    int             UpdRate;
    int             nCycleOffset;
    int             Socket;
} tCameraRSI;

/* Global struct for this Node */
static struct {
    struct {
        double         Duration;      /*!< Time spent for synchronization task */
        int            nCycles;       /*!< Number of cycles in synchronization loop */
    } Sync; /*!< Synchronization related information */

    struct {
        std::shared_ptr<tf2_ros::TransformBroadcaster> br;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> st_br;

        geometry_msgs::msg::TransformStamped Lidar;
        geometry_msgs::msg::TransformStamped Lidar_PS150;
        geometry_msgs::msg::TransformStamped Lidar_Pandar40p;
        geometry_msgs::msg::TransformStamped Lidar_Pandar40p_Obj_Detect;
        geometry_msgs::msg::TransformStamped Camera;
        geometry_msgs::msg::TransformStamped ObjectList;
    } TF;

    struct {
        struct {
        } Sub; /*!< Topics to be subscribed */
        struct {
        } Pub; /*!< Topics to be published */
    } Topics; /*!< ROS Topics used by this Node */

    struct {
        struct {
            int             Active;             /*!< Presence of active Object sensors */
            double*         pos;                /*!< Mounting position on vehicle frame */
            double*         rot;                /*!< Mounting rotation on vehicle frame */
            int             UpdRate;
            int             nCycleOffset;
        } Object;

        struct {
            int             Active;             /*!< Presence of active Camera sensors */
            double*         pos;                /*!< Mounting position on vehicle frame */
            double*         rot;                /*!< Mounting rotation on vehicle frame */
            int             UpdRate;
            int             nCycleOffset;
        } Camera;

        struct {
            int             Active;                 /*!< Presence of active LidarRSI sensors */
            int             index;                  /*!< Tells us which Lidar of all the Lidars the Pandar40p exists at*/
            double*         pos;                    /*!< Mounting position on vehicle frame */
            double*         rot;                    /*!< Mounting rotation on vehicle frame */
            int             UpdRate;
            int             nCycleOffset;

            /* Variables to be used in LidarRSI processing. */
            double *FOV_h;
            double *FOV_v;
            double nBeams_h;
            double nBeams_v;
            double BeamTable[tableSize];
            int ret, rows = 0;
            double x[tableSize / 6];
            double y[tableSize / 6];
            double z[tableSize / 6];
            double* nBeams;
            int cycleCounter = 0;
        } LidarRSI;

        struct {
          int             Active;                 /*!< Presence of active LidarRSI sensors */
          int             index;                  /*!< Tells us which Lidar of all the Lidars the PS150 exists at*/
          double*         pos;                    /*!< Mounting position on vehicle frame */
          double*         rot;                    /*!< Mounting rotation on vehicle frame */
          int             UpdRate;
          int             nCycleOffset;

          /* Variables to be used in LidarRSI processing. */
          double *FOV_h;
          double *FOV_v;
          double nBeams_h;
          double nBeams_v;
          double BeamTable[ps150_tableSize];
          int ret, rows = 0;
          double x[ps150_tableSize / 6];
          double y[ps150_tableSize / 6];
          double z[ps150_tableSize / 6];
          double* nBeams;
          int cycleCounter = 0;
      } LidarRSI_PS150;

      struct {
        int             Active;                 /*!< Presence of active LidarRSI sensors */
        int             index;                  /*!< Tells us which Lidar of all the Lidars the Pandar40p exists at*/
        double*         pos;                    /*!< Mounting position on vehicle frame */
        double*         rot;                    /*!< Mounting rotation on vehicle frame */
        int             UpdRate;
        int             nCycleOffset;

        struct {
          int           Active;
          int           index;
          double*       pos;                    /*!< Mounting position on vehicle frame */
          double*       rot;                    /*!< Mounting rotation on vehicle frame */
          int           UpdRate;
          int           nCycleOffset;
        } Object_Sensor;

        /* Variables to be used in LidarRSI processing. */
        double *FOV_h;
        double *FOV_v;
        double nBeams_h;
        double nBeams_v;
        double BeamTable[pandar40p_tableSize];
        int ret, rows = 0;
        double x[pandar40p_tableSize / 6];
        double y[pandar40p_tableSize / 6];
        double z[pandar40p_tableSize / 6];
        double* nBeams;
        int cycleCounter = 0;
    } LidarRSI_Pandar40p;

        struct {
            int             Active;
            double*         pos;                /*!< Mounting position on vehicle frame */
            double*         rot;                /*!< Mounting rotation on vehicle frame */
            int             UpdRate;
        } GCS;

        std::vector<tCameraRSI> CameraRSI;

    } Sensor; /*!< Sensor parameters */

    struct {
        int                 use_vc;
        // int                 selector_ctrl;
        // double              gas;
        // double              steer_ang_vel;
        // double              steer_ang_acc;
        struct {
            double              velocity;     /*!< m/s desired for wheels/car */
            double              steer_ang;    /*!< Radians desired for steering wheel [-pi - pi] : maps to [+-27 degrees] */
            double              front_brake;
            double              rear_brake;
            double              front_trq;
            double              rear_trq;
        } AI;
        
        struct {
            double              velocity;     /*!< m/s desired for wheels/car */
            double              steer_ang;    /*!< Radians desired for steering wheel [-pi - pi] : maps to [+-27 degrees] */
            double              brake;        /*!< brake desired for pedal [0 - 1] range */
        } Manual;
    } VC; /*!< Vehicle Controls */

    struct {
        int nSensors;

        struct{
          vector<double> current_speed;
          vector<double> desired_speed;
          vector<double> gas_input;
          vector<double> brake_input;
        } pid_data;

        int gt_cones_int;
        bristol_msgs::msg::ConeArrayWithCovariance gt_cones;
        int collision_sensor_active;
    } Misc;
} CMNode;


/*!
 * Description:
 * - Callback for ROS Topic published by external ROS Node
 *
 */
void CMNodeHelloCM::ext2cmCallback(hellocm_msgs::msg::Ext2CM::ConstSharedPtr msg) {
  std::stringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(3);
  ss << "\t[" << rclcpp::Clock().now().seconds();
  if (nhp_->get_clock()->ros_time_is_active()) {
    ss << ", " << nhp_->now().seconds();
  }
  ss << "]: " << nhp_->get_fully_qualified_name() << ": Sub Msg: ";
  ss << "Time " << rclcpp::Time(msg->time).seconds() << "s, ";
  ss << "Cycle " << msg->cycleno;

  Log::printLog(ss.str());
}

/* Vehicle Control Subscriber Callback */
void CMNodeHelloCM::vehicleAIControlCallback(ads_dv_msgs::msg::AI2VCURequests::ConstSharedPtr msg) {

    
    // CMNode.VC.use_vc          = msg->use_vc;
    // CMNode.VC.selector_ctrl   = msg->selector_ctrl;
    // CMNode.VC.gas             = msg->gas;
    // CMNode.VC.brake           = msg->brake;
    // CMNode.VC.steer_ang       = msg->steer_ang;
    // CMNode.VC.steer_ang_vel   = msg->steer_ang_vel;
    // CMNode.VC.steer_ang_acc   = msg->steer_ang_acc;

    const double wheel_radius = 0.2525; // In meters (0.505 / 2)

    // Convert the motor speed rpm to velocity
    double front_vel = (msg->rpm_request * M_PI * wheel_radius) / 30.0;
    double rear_vel = (msg->rpm_request * M_PI * wheel_radius) / 30.0;
    CMNode.VC.AI.velocity        = std::max({front_vel, rear_vel, 0.0});

    CMNode.VC.AI.steer_ang       = (msg->steer_request_deg * M_PI) / 27.18;

    CMNode.VC.AI.front_brake     = msg->brake_request / 100.0; // values are currently received in range 0-100 normalised hydraulic pressure
    CMNode.VC.AI.rear_brake      = msg->brake_request / 100.0;
    
    CMNode.VC.AI.front_trq       = msg->torque_request;
    CMNode.VC.AI.rear_trq        = msg->torque_request;
    RCLCPP_DEBUG_THROTTLE(nhp_->get_logger(), *nhp_->get_clock(), 1000, "Steer: %.4f, Velocity: %.4f", CMNode.VC.AI.steer_ang, CMNode.VC.AI.velocity);


/*
    in->Msg.key             = msg->key;
    in->Msg.sst             = msg->sst;

    in->Msg.brakelight      = msg->brakelight;
    in->Msg.daytimelights   = msg->daytimelights;
    in->Msg.foglights_left  = msg->foglights_left;
    in->Msg.foglights_right = msg->foglights_right;
    in->Msg.foglights_rear  = msg->foglights_rear;
    in->Msg.highbeams       = msg->highbeams;
    in->Msg.indicator_left  = msg->indicator_left;
    in->Msg.indicator_right = msg->indicator_right;
    in->Msg.lowbeams        = msg->lowbeams;
    in->Msg.parklight_left  = msg->parklight_left;
    in->Msg.parklight_right = msg->parklight_right;
    in->Msg.reverselights   = msg->reverselights;
*/
}

/* Vehicle RQT Manual Control Subscriber Callback */
void CMNodeHelloCM::vehicleManualControlCallback(ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
    if (msg->drive.speed >= 0){
      CMNode.VC.Manual.velocity = msg->drive.speed;
      CMNode.VC.Manual.brake = 0;
    } else{
      CMNode.VC.Manual.velocity = 0;
      CMNode.VC.Manual.brake = -msg->drive.speed / 5.0;
    }

    CMNode.VC.Manual.steer_ang = msg->drive.steering_angle * M_PI; // Actual car is -21.0 to 21.0, but sim car is +-27.18
}


/* Publisher for information about ADS-DV state, in the UWE-ROS AutonomousState msg format*/
void CMNodeHelloCM::autonomousStateFillMsg(ads_dv_msgs::msg::AutonomousState& msg) {
    // **TODO** Link proper autonomous state machine logic, currently just sends the "we are currently driving" information to kickstart software.

    msg.as_state = 3; // AS_DRIVING
    msg.ami_state = 4; // AMI_TRACKDRIVE
    msg.res_go_signal = 1; // RES_GO_SIGNAL_GO

}

/* Publisher for information about ADS-DV state, in the UWE-ROS AutonomousState msg format*/
void CMNodeHelloCM::mapFillGTMsg(bristol_msgs::msg::ConeArrayWithCovariance& msg) {
    if (CMNode.Misc.gt_cones.blue_cones.empty() && Traffic_GetNObj() > 0){  

      /* Clear vector data to avoid overflows */
      CMNode.Misc.gt_cones.blue_cones.clear();
      CMNode.Misc.gt_cones.yellow_cones.clear();
      CMNode.Misc.gt_cones.orange_cones.clear();
      CMNode.Misc.gt_cones.big_orange_cones.clear();
      CMNode.Misc.gt_cones.unknown_color_cones.clear();

      CMNode.Misc.gt_cones.header.frame_id = "Fr0"; // Should be global -> Fr0

      for (int j = 0; j < Traffic_GetNObj(); j++) {
        tTrafficObj *TrfObj = Traffic_GetByTrfId(j);
        
        if (!strcmp(TrfObj->Cfg.Info, "Right Yellow Cone")) {
          /* yellow */
          bristol_msgs::msg::ConeWithCovariance yellow_cone;
          yellow_cone.point.x = TrfObj->t_0[0];
          yellow_cone.point.y = TrfObj->t_0[1];
          yellow_cone.point.z = TrfObj->t_0[2];
          CMNode.Misc.gt_cones.yellow_cones.push_back(yellow_cone);
        } else if (!strcmp(TrfObj->Cfg.Info, "Left Blue Cone")) {
          /* blue */
          bristol_msgs::msg::ConeWithCovariance blue_cone;
          blue_cone.point.x = TrfObj->t_0[0];
          blue_cone.point.y = TrfObj->t_0[1];
          blue_cone.point.z = TrfObj->t_0[2];
          CMNode.Misc.gt_cones.blue_cones.push_back(blue_cone);
        } else if (!strcmp(TrfObj->Cfg.Info, "Large Orange Start Cone")) {
          /* orange start cones */
          bristol_msgs::msg::ConeWithCovariance orange_cone;
          orange_cone.point.x = TrfObj->t_0[0];
          orange_cone.point.y = TrfObj->t_0[1];
          orange_cone.point.z = TrfObj->t_0[2];
          CMNode.Misc.gt_cones.orange_cones.push_back(orange_cone);
        }
      }
      
      RCLCPP_INFO(nhp_->get_logger(), "Finished loading %i cones for ground truth map", Traffic_GetNObj());
    }

    if (Traffic_GetNObj() > 0){ 
      msg = CMNode.Misc.gt_cones;
    }
}

void CMNodeHelloCM::mapFillGTMsg2(visualization_msgs::msg::MarkerArray& msg) {

    if (Traffic_GetNObj() > 0){  

      visualization_msgs::msg::Marker markers;

      /* Clear vector data to avoid overflows */
      msg.markers.clear();

      for (int j = 0; j < Traffic_GetNObj(); j++) {
        tTrafficObj *TrfObj = Traffic_GetByTrfId(j);

        markers.header.frame_id = "Fr0"; // Should be global -> Fr0
        markers.id = j;
        markers.type = visualization_msgs::msg::Marker::CUBE;
        markers.action = visualization_msgs::msg::Marker::ADD;

        markers.scale.x = TrfObj->Cfg.l;
        markers.scale.y = TrfObj->Cfg.w;
        markers.scale.z = TrfObj->Cfg.h;

        /* white */
        markers.color.a = 0.7f;
        markers.color.r = 1.0;
        markers.color.g = 1.0;
        markers.color.b = 1.0;
        
        if (!strcmp(TrfObj->Cfg.Info, "Right Yellow Cone")) {
          /* yellow */
          markers.color.r = 1.0;
          markers.color.g = 1.0;
          markers.color.b = 0.0;
        } else if (!strcmp(TrfObj->Cfg.Info, "Left Blue Cone")) {
          /* blue */
          markers.color.r = 0.0;
          markers.color.g = 0.3;
          markers.color.b = 1.0;
        } else if (!strcmp(TrfObj->Cfg.Info, "Large Orange Start Cone")) {
          /* orange start cones */
          markers.color.r = 1.0;
          markers.color.g = 0.6;
          markers.color.b = 0.0;
        }

        /* Object rotation as quaternion */
        tf2::Quaternion rotation;
        rotation.setRPY(TrfObj->r_zyx[0], TrfObj->r_zyx[1], TrfObj->r_zyx[2]);

        /* Apply rotation to vector pointing to the object's center */
        tf2::Vector3 obj_center = tf2::quatRotate(rotation, tf2::Vector3(0.5 * TrfObj->Cfg.l, 0, 0));

        markers.pose.position.x = TrfObj->t_0[0];
        markers.pose.position.y = TrfObj->t_0[1];
        markers.pose.position.z = TrfObj->t_0[2];
        // markers.pose.orientation = tf2::toMsg(rotation);

        msg.markers.push_back(markers);
      } // for (int j = 0; j < ObjectSensor[0].nObsvObjects; j++)
    } // if (CMNode.Sensor.Object.Active)
}

/*****************************************************************************/
/**********          C-Code for interfacing with CarMaker!          **********/
/*****************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Basic Initialization
 * - e.g. create ROS Node, subscriptions, ...
 * - Return values
 *   - "rv <  0" = Error at initialization, CarMaker executable will stop
 *   - "rv >= 0" = Everything OK, CarMaker executable will continue
 *
 * Arguments:
 * - Inf        = Handle to CarMaker Infofile with parameters for this interface
 *                - Please note that pointer may change, e.g. before TestRun begins
 */
int
CMNodeHelloCM::userInit()
{
  CMNode.TF.br = std::make_shared<tf2_ros::TransformBroadcaster>(nhp_);
  CMNode.TF.st_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nhp_);

  Log::printLog("  -> Initializing service client /hellocm/init");
  srv_init_ = nhp_->create_client<hellocm_msgs::srv::Init>("/hellocm/init");

  current_driver_mode_ = "AI"; // *TODO* get from config file
  driver_mode_srv_ = nhp_->create_service<sim_msgs::srv::DriverMode>(
        "/driver_mode",
        std::bind(&CMNodeHelloCM::handle_driver_request, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(nhp_->get_logger(), "Driver mode server ready. Initial mode: %s", current_driver_mode_.c_str());


  reset_publisher_ = nhp_->create_publisher<std_msgs::msg::Bool>("/carmaker/start_signal", 10);
  RCLCPP_INFO(nhp_->get_logger(), "Reset publisher initialized on topic: /carmaker/start_signal");

  param_client_ =
      std::make_shared<rclcpp::SyncParametersClient>(nhp_, "/hellocm/hellocm");

  
  return 1;
}

void CMNodeHelloCM::pidInit()
{
  steer_pid = PID(1.0, 0.1, 0.05, -1.0, 1.0, 0.2);
  brake_pid = PID(0.4, 0.07, 0.2, -1.0, 0.1, 0.1);
  gas_pid   = PID(0.4, 0.07, 0.2, -1.0, 0.1, 0.1);
  // rclcpp::Time current_time = rclcpp::Clock().now();
}

void CMNodeHelloCM::publishResetSignal() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;  // Reset signal
    reset_publisher_->publish(msg);
    RCLCPP_INFO(nhp_->get_logger(), "Reset signal published!");
}

/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after CMRosIF_CMNode_DrivManCalc
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_VehicleControl_Calc()"
 */
int CMNodeHelloCM::userVehicleControlCalc(const double& dt)
{
    (void)dt;
    double turn_speed = 0.5;
    if (SimCore.Time < 0.001) {
        // CMNode.VC.use_vc = 0; // Reset back to DM at beginning of simulation...
        CMNode.VC.use_vc = 1; // Using AI mode
        // current_driver_mode_ = "AI"; // **TODO** set using config
        /* Assign VC to DM quants in case not written to... */
        // VehicleControl.SelectorCtrl     = DrivMan.SelectorCtrl;

        // VehicleControl.Gas              = DrivMan.Gas;
        // VehicleControl.Brake            = DrivMan.Brake;

        // VehicleControl.Steering.Ang     = DrivMan.Steering.Ang;
        // VehicleControl.Steering.AngVel  = DrivMan.Steering.AngVel;
        // VehicleControl.Steering.AngAcc  = DrivMan.Steering.AngAcc;
        VehicleControl.SelectorCtrl     = 1;

        VehicleControl.Gas              = 0.0;
        VehicleControl.Brake            = 0.1;

        VehicleControl.Steering.Ang     = 0.0;
        VehicleControl.Steering.AngVel  = 0.0;
        VehicleControl.Steering.AngAcc  = 0.0;

        CMNode.VC.AI.velocity = 0.0;
        CMNode.VC.AI.steer_ang = 0.0;
        CMNode.VC.AI.front_brake = 0.1;
        CMNode.VC.AI.rear_brake = 0.1;
        CMNode.VC.AI.front_trq = 0.0;
        CMNode.VC.AI.rear_trq = 0.0;

        CMNode.VC.Manual.velocity = 0.0;
        CMNode.VC.Manual.steer_ang = 0.0;
        CMNode.VC.Manual.brake = 0.1;
        
        pidInit();
        publishResetSignal();
    }

    if (CMNode.VC.use_vc == 0) {
      
      /* Assign VC to DM quants in case not written to... */
      VehicleControl.SelectorCtrl     = DrivMan.SelectorCtrl;

      VehicleControl.Gas              = DrivMan.Gas;
      VehicleControl.Brake            = DrivMan.Brake;

      VehicleControl.Steering.Ang     = DrivMan.Steering.Ang;
      VehicleControl.Steering.AngVel  = DrivMan.Steering.AngVel;
      VehicleControl.Steering.AngAcc  = DrivMan.Steering.AngAcc;

    } else if (CMNode.VC.use_vc == 1) {
        rclcpp::Time current_time = rclcpp::Clock().now();
        double dt = (current_time - previous_pid_time).seconds();

        // Velocity PID parameters
        double desired_velocity = CMNode.VC.AI.velocity; // Target steering angle in degrees
        double current_velocity = Vehicle.v;  // Current steering angle
        double error = desired_velocity - current_velocity;

        if (desired_velocity < 0.001) {
            gas_pid.resetIntegral();
            brake_pid.resetIntegral();
            VehicleControl.Gas = 0.0;
            VehicleControl.Brake = 1.0; // Ensure the vehicle comes to a stop
        } 
        else if (error > 0.0) {
            // Acceleration phase
            double velocity_control_signal = gas_pid.update(desired_velocity, current_velocity, dt);
            VehicleControl.Gas = std::clamp(VehicleControl.Gas + velocity_control_signal, 0.0, 1.0);
            VehicleControl.Brake = 0.0; // No braking when accelerating
            brake_pid.resetIntegral();  // Reset brake integrator
        } 
        else {
            // Deceleration phase
            double velocity_control_signal = brake_pid.update(desired_velocity, current_velocity, dt);
            VehicleControl.Brake = std::clamp(VehicleControl.Brake + velocity_control_signal, 0.0, 1.0);
            VehicleControl.Gas = 0.0; // No acceleration when braking
            gas_pid.resetIntegral();  // Reset gas integrator
        }
        
        if (CMNode.VC.AI.front_brake > 0.1){
            VehicleControl.Brake = CMNode.VC.AI.front_brake;
        } // else use the PID braking for speed management
        
        VehicleControl.Steering.Ang = CMNode.VC.AI.steer_ang;

        previous_pid_time = current_time;
        // RCLCPP_DEBUG_THROTTLE(nhp_->get_logger(), *nhp_->get_clock(), 1000, "Veloc: Desired: %.3f, Current: %.3f, control_signal: %.3f, dt: %.6f", desired_velocity, current_velocity, VehicleControl.Gas, dt);
        
        CMNode.Misc.pid_data.current_speed.push_back(current_velocity);
        CMNode.Misc.pid_data.desired_speed.push_back(desired_velocity);
        CMNode.Misc.pid_data.gas_input.push_back(VehicleControl.Gas);
        CMNode.Misc.pid_data.brake_input.push_back(VehicleControl.Brake);

    } else if (CMNode.VC.use_vc == 2) {
        rclcpp::Time current_time = rclcpp::Clock().now();
        double dt = (current_time - previous_pid_time).seconds();

        /* Notes: Current PID gets to speed effectively, but doesn't brake effectively. Although unintentional
        ** it means that the motion control team are responsible for braking performance as in a real competition
        */

        // Velocity PID parameters
        double desired_velocity = CMNode.VC.Manual.velocity; // Target velocity
        double current_velocity = Vehicle.v;  // Current velocity
        double error = desired_velocity - current_velocity;

        if (desired_velocity < 0.001) {
            gas_pid.resetIntegral();
            brake_pid.resetIntegral();
            VehicleControl.Gas = 0.0;
            VehicleControl.Brake = 1.0; // Ensure the vehicle comes to a stop
        } 
        else if (error > 0.0) {
            // Acceleration phase
            double velocity_control_signal = gas_pid.update(desired_velocity, current_velocity, dt);
            VehicleControl.Gas = std::clamp(VehicleControl.Gas + velocity_control_signal, 0.0, 1.0);
            VehicleControl.Brake = 0.0; // No braking when accelerating
            brake_pid.resetIntegral();  // Reset brake integrator
        } 
        else {
            // Deceleration phase
            double velocity_control_signal = brake_pid.update(desired_velocity, current_velocity, dt);
            VehicleControl.Brake = std::clamp(VehicleControl.Brake + velocity_control_signal, 0.0, 1.0);
            VehicleControl.Gas = 0.0; // No acceleration when braking
            gas_pid.resetIntegral();  // Reset gas integrator
        }
        // Current development area
        static double Car_wheel_ang = VehicleControl.Steering.Ang;
        double Desired_wheel_ang = CMNode.VC.Manual.steer_ang;
        // static double desired_time = 0.987;                // Desired speed that can be changed (-+ 0.03 margin of error)
        // double speed_iterations = 0.00057246/desired_time; // Inversely proportional
        const double steering_rate_dps = 12.0;
        const double speed_iterations = steering_rate_dps / 1000.0;  // because 1000Hz sim updates
        const double threshold = speed_iterations/2;

        if(VehicleControl.Steering.Ang != Desired_wheel_ang){
          // To prevent oscillating around the desired
          if (fabs(Desired_wheel_ang - Car_wheel_ang) < threshold) {
              Car_wheel_ang = Desired_wheel_ang;

          // Negative step toward target
          } else if (Desired_wheel_ang < Car_wheel_ang) {
              Car_wheel_ang -= speed_iterations;
            
          // Positive step toward target
          } else if (Desired_wheel_ang > Car_wheel_ang) {
              Car_wheel_ang += speed_iterations;
          }
          VehicleControl.Steering.Ang = Car_wheel_ang;
        }
        previous_pid_time = current_time;
        // RCLCPP_INFO_THROTTLE(nhp_->get_logger(), *nhp_->get_clock(), 1000, "Steer: Desired: %.3f, Current: %.3f, control_signal: %.3f", desired_steering_angle, current_angle, (CMNode.VC.steer_ang * M_PI)/24.0);
        // RCLCPP_INFO_THROTTLE(nhp_->get_logger(), *nhp_->get_clock(), 1000, "Veloc: Desired: %.3f, Current: %.3f, control_signal: %.3f", desired_velocity, current_velocity, VehicleControl.Gas);
        
        CMNode.Misc.pid_data.current_speed.push_back(current_velocity);
        CMNode.Misc.pid_data.desired_speed.push_back(desired_velocity);
        CMNode.Misc.pid_data.gas_input.push_back(VehicleControl.Gas);
        CMNode.Misc.pid_data.brake_input.push_back(VehicleControl.Brake);
    }

    return 1;
}

#ifdef __cplusplus
}
#endif

void CMNodeHelloCM::handle_driver_request(
    const std::shared_ptr<sim_msgs::srv::DriverMode::Request> request,
    std::shared_ptr<sim_msgs::srv::DriverMode::Response> response) {
    if (request->request_mode.empty()) {
        // Query current mode
        response->success = true;
        response->current_mode = current_driver_mode_;
        response->message = "Current mode retrieved successfully.";
        RCLCPP_INFO(nhp_->get_logger(), "Mode query: %s", current_driver_mode_.c_str());
    } else if (request->request_mode == "IPG" || request->request_mode == "Manual" || request->request_mode == "AI") {
        // Update mode
        current_driver_mode_ = request->request_mode;
        
        if (request->request_mode == "IPG"){
          CMNode.VC.use_vc = 0;
          pidInit();
        } else if(request->request_mode == "AI"){
          CMNode.VC.use_vc = 1;
          pidInit();
        } else if(request->request_mode == "Manual"){
          CMNode.VC.use_vc = 2;
          pidInit();
        }
        
        response->success = true;
        response->current_mode = current_driver_mode_;
        response->message = response->message = "Mode changed to " + current_driver_mode_;
        RCLCPP_INFO(nhp_->get_logger(), "Mode updated: %s", current_driver_mode_.c_str());
    } else {
        // Invalid mode request
        response->success = false;
        response->current_mode = current_driver_mode_;
        response->message = "Invalid mode requested: " + request->request_mode;
        RCLCPP_WARN(nhp_->get_logger(), "Invalid mode requested: %s", request->request_mode.c_str());
    }
}

void CMNodeHelloCM::pointcloudFillMsg(sensor_msgs::msg::PointCloud& msg) {

  // std::cout << "===== Generic Lidar Stats =====" << std::endl;
  // std::cout << "Index: " << CMNode.Sensor.LidarRSI.index << std::endl;
  // std::cout << "TableSize: " << tableSize << std::endl;
  // std::cout << "Scan Points: " << LidarRSI[CMNode.Sensor.LidarRSI.index].nScanPoints << std::endl;
  // std::cout << "===============================" << std::endl;

  if (CMNode.Sensor.LidarRSI.Active) {   

    geometry_msgs::msg::Point32 points;
    sensor_msgs::msg::ChannelFloat32 channels;
    channels.name = "intensity";

    /* Clear vector data to avoid overflows */
    msg.points.clear();
    msg.channels.clear();

    /* Populate Intensity channel data points */
    for (int i = 0; i < LidarRSI[0].nScanPoints; i++) {

      const int beam_id = LidarRSI[0].ScanPoint[i].BeamID;
      const double azimuth = angles::from_degrees(CMNode.Sensor.LidarRSI.BeamTable[4*CMNode.Sensor.LidarRSI.rows + beam_id]);
      const double elevation = angles::from_degrees(CMNode.Sensor.LidarRSI.BeamTable[5*CMNode.Sensor.LidarRSI.rows + beam_id]);
      const double ray_length = 0.5 * LidarRSI[0].ScanPoint[i].LengthOF; // length of flight is back and forth

      /* XYZ-coordinates of scan point */
      points.x = ray_length * cos(elevation) * cos(azimuth);
      points.y = ray_length * cos(elevation) * sin(azimuth);
      points.z = ray_length * sin(elevation);

      msg.points.push_back(points);
      channels.values.push_back(LidarRSI[0].ScanPoint[i].Intensity);
    }
    msg.channels.push_back(channels);
    msg.header.frame_id = CMNode.TF.Lidar.child_frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(1e9 * LidarRSI[0].ScanTime), RCL_ROS_TIME);
  }
}

void CMNodeHelloCM::PS150pointcloudFillMsg(sensor_msgs::msg::PointCloud& msg) {

  // std::cout << " ===== PS150 Stats ========" << std::endl;
  // std::cout << "Index: " << CMNode.Sensor.LidarRSI_PS150.index << std::endl;
  // std::cout << "TableSize: " << ps150_tableSize << std::endl;
  // std::cout << "Scan Points: " << LidarRSI[CMNode.Sensor.LidarRSI_PS150.index].nScanPoints << std::endl;
  // std::cout << "===========================" << std::endl;

  if (CMNode.Sensor.LidarRSI_PS150.Active) {   

    geometry_msgs::msg::Point32 points;
    sensor_msgs::msg::ChannelFloat32 channels;
    channels.name = "intensity";

    /* Clear vector data to avoid overflows */
    msg.points.clear();
    msg.channels.clear();

    /* Populate Intensity channel data points */
    for (int i = 0; i < LidarRSI[CMNode.Sensor.LidarRSI_PS150.index].nScanPoints; i++) {

      const int beam_id = LidarRSI[CMNode.Sensor.LidarRSI_PS150.index].ScanPoint[i].BeamID;
      const double azimuth = angles::from_degrees(CMNode.Sensor.LidarRSI_PS150.BeamTable[4*CMNode.Sensor.LidarRSI_PS150.rows + beam_id]);
      const double elevation = angles::from_degrees(CMNode.Sensor.LidarRSI_PS150.BeamTable[5*CMNode.Sensor.LidarRSI_PS150.rows + beam_id]);
      const double ray_length = 0.5 * LidarRSI[CMNode.Sensor.LidarRSI_PS150.index].ScanPoint[i].LengthOF; // length of flight is back and forth

      /* XYZ-coordinates of scan point */
      points.x = ray_length * cos(elevation) * cos(azimuth);
      points.y = ray_length * cos(elevation) * sin(azimuth);
      points.z = ray_length * sin(elevation);

      msg.points.push_back(points);
      channels.values.push_back(LidarRSI[CMNode.Sensor.LidarRSI_PS150.index].ScanPoint[i].Intensity);
    }
    msg.channels.push_back(channels);
    msg.header.frame_id = CMNode.TF.Lidar_PS150.child_frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(1e9 * LidarRSI[CMNode.Sensor.LidarRSI_PS150.index].ScanTime), RCL_ROS_TIME);
  }
}

void CMNodeHelloCM::Pandar40pPointcloudFillMsg(sensor_msgs::msg::PointCloud& msg) {

  // std::cout << " ===== Pandar40 Stats =====" << std::endl;
  // std::cout << "Index: " << CMNode.Sensor.LidarRSI_Pandar40p.index << std::endl;
  // std::cout << "TableSize: " << pandar40p_tableSize << std::endl;
  // std::cout << "Scan Points: " << LidarRSI[CMNode.Sensor.LidarRSI_Pandar40p.index].nScanPoints << std::endl;
  // std::cout << "===========================" << std::endl;
  

  if (CMNode.Sensor.LidarRSI_Pandar40p.Active) {   

    geometry_msgs::msg::Point32 points;
    sensor_msgs::msg::ChannelFloat32 channels;
    channels.name = "intensity";

    /* Clear vector data to avoid overflows */
    msg.points.clear();
    msg.channels.clear();

    /* Populate Intensity channel data points */
    for (int i = 0; i < LidarRSI[CMNode.Sensor.LidarRSI_Pandar40p.index].nScanPoints; i++) {

      const int beam_id = LidarRSI[CMNode.Sensor.LidarRSI_Pandar40p.index].ScanPoint[i].BeamID;
      const double azimuth = angles::from_degrees(CMNode.Sensor.LidarRSI_Pandar40p.BeamTable[4*CMNode.Sensor.LidarRSI_Pandar40p.rows + beam_id]);
      const double elevation = angles::from_degrees(CMNode.Sensor.LidarRSI_Pandar40p.BeamTable[5*CMNode.Sensor.LidarRSI_Pandar40p.rows + beam_id]);
      const double ray_length = 0.5 * LidarRSI[CMNode.Sensor.LidarRSI_Pandar40p.index].ScanPoint[i].LengthOF; // length of flight is back and forth

      /* XYZ-coordinates of scan point */
      points.x = ray_length * cos(elevation) * cos(azimuth);
      points.y = ray_length * cos(elevation) * sin(azimuth);
      points.z = ray_length * sin(elevation);

      msg.points.push_back(points);
      channels.values.push_back(LidarRSI[CMNode.Sensor.LidarRSI_Pandar40p.index].ScanPoint[i].Intensity);
    }
    msg.channels.push_back(channels);
    msg.header.frame_id = CMNode.TF.Lidar_Pandar40p.child_frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(1e9 * LidarRSI[CMNode.Sensor.LidarRSI_Pandar40p.index].ScanTime), RCL_ROS_TIME);
  }
}

void CMNodeHelloCM::objectlistFillMsg(visualization_msgs::msg::MarkerArray& msg) {

  if (CMNode.Sensor.Object.Active) {   

    visualization_msgs::msg::Marker markers;
    rclcpp::Time object_sensor_stamp = rclcpp::Time(static_cast<int64_t>(1e9 * ObjectSensor[0].TimeStamp), RCL_ROS_TIME);

    /* Clear vector data to avoid overflows */
    msg.markers.clear();

    /* Display objects with a duration of the topics cycle time, so rviz forgets them when 
       they are not being published anymore: (80 + milliseconds) to seconds */
    rclcpp::Duration object_viz_time = rclcpp::Duration(static_cast<int64_t>(1e6 * 80 + CMNode.Sensor.Camera.UpdRate), RCL_ROS_TIME);

    for (int j = 0; j < ObjectSensor[0].nObsvObjects; j++) {

      int obj_id = ObjectSensor[0].ObsvObjects[j];
      tObjectSensorObj *pOSO = ObjectSensor_GetObjectByObjId(0, obj_id);
      tTrafficObj *pObj = Traffic_GetByObjId(obj_id);

      markers.header.frame_id = CMNode.TF.ObjectList.child_frame_id;
      markers.id = pOSO->ObjId;
      markers.type = visualization_msgs::msg::Marker::CUBE;
      markers.action = visualization_msgs::msg::Marker::ADD;
      markers.lifetime = object_viz_time;
      markers.header.stamp = object_sensor_stamp;

      markers.scale.x = pOSO->l;
      markers.scale.y = pOSO->w;
      markers.scale.z = pOSO->h;

      /* white */
      markers.color.a = 0.7f;
      markers.color.r = 1.0;
      markers.color.g = 1.0;
      markers.color.b = 1.0;

      if (pOSO->dtct || pOSO->InLane){
        if (!strcmp(pObj->Cfg.Info, "Right Yellow Cone")) {
          /* yellow */
          markers.color.r = 1.0;
          markers.color.g = 1.0;
          markers.color.b = 0.0;
        } else if (!strcmp(pObj->Cfg.Info, "Left Blue Cone")) {
          /* blue */
          markers.color.r = 0.0;
          markers.color.g = 0.3;
          markers.color.b = 1.0;
        } else {
          /* orange start cones */
          markers.color.r = 1.0;
          markers.color.g = 0.6;
          markers.color.b = 0.0;
        }
      }

      /* Object rotation as quaternion */
      tf2::Quaternion rotation;
      rotation.setRPY(pOSO->RefPnt.r_zyx[0], pOSO->RefPnt.r_zyx[1], pOSO->RefPnt.r_zyx[2]);

      /* Apply rotation to vector pointing to the object's center */
      tf2::Vector3 obj_center = tf2::quatRotate(rotation, tf2::Vector3(0.5 * pOSO->l, 0, 0));

      markers.pose.position.x = pOSO->RefPnt.ds[0]; // + obj_center.getX();
      markers.pose.position.y = pOSO->RefPnt.ds[1]; // + obj_center.getY();
      markers.pose.position.z = pOSO->RefPnt.ds[2]; // + obj_center.getZ();
      markers.pose.orientation = tf2::toMsg(rotation);

      msg.markers.push_back(markers);
    } // for (int j = 0; j < ObjectSensor[0].nObsvObjects; j++)
  } // if (CMNode.Sensor.Object.Active)
}

void CMNodeHelloCM::objectlistFillGTConeMsg(bristol_msgs::msg::ConeArrayWithCovariance& msg){
  if (CMNode.Sensor.Object.Active) {  

    // list<bristol_msgs::msg::ConeWithCovariance> blue_cones = {};
    // list<bristol_msgs::msg::ConeWithCovariance> yellow_cones = {};
    // list<bristol_msgs::msg::ConeWithCovariance> orange_cones = {};
    // list<bristol_msgs::msg::ConeWithCovariance> big_orange_cones = {};
    // list<bristol_msgs::msg::ConeWithCovariance> unknown_color_cones = {};

    rclcpp::Time object_sensor_stamp = rclcpp::Time(static_cast<int64_t>(1e9 * ObjectSensor[0].TimeStamp), RCL_ROS_TIME);

    /* Clear vector data to avoid overflows */
    msg.blue_cones.clear();
    msg.yellow_cones.clear();
    msg.orange_cones.clear();
    msg.big_orange_cones.clear();
    msg.unknown_color_cones.clear();

    for (int j = 0; j < ObjectSensor[0].nObsvObjects; j++) {
      int obj_id = ObjectSensor[0].ObsvObjects[j];
      tObjectSensorObj *pOSO = ObjectSensor_GetObjectByObjId(0, obj_id);
      tTrafficObj *pObj = Traffic_GetByObjId(obj_id);

      // msg.header.frame_id = CMNode.TF.ObjectList.child_frame_id;
      msg.header.frame_id = CMNode.TF.ObjectList.child_frame_id;
      msg.header.stamp = object_sensor_stamp;


      /* Object rotation as quaternion */
      tf2::Quaternion rotation;
      rotation.setRPY(pOSO->RefPnt.r_zyx[0], pOSO->RefPnt.r_zyx[1], pOSO->RefPnt.r_zyx[2]);

      /* Apply rotation to vector pointing to the object's center */
      tf2::Vector3 obj_center = tf2::quatRotate(rotation, tf2::Vector3(0.5 * pOSO->l, 0, 0));

      // markers.pose.position.x = pOSO->RefPnt.ds[0] + obj_center.getX();
      // markers.pose.position.y = pOSO->RefPnt.ds[1] + obj_center.getY();
      // markers.pose.position.z = pOSO->RefPnt.ds[2] + obj_center.getZ();

      if (pOSO->dtct || pOSO->InLane){
        if (!strcmp(pObj->Cfg.Info, "Right Yellow Cone")) {
          /* yellow */
          bristol_msgs::msg::ConeWithCovariance yellow_cone;
          yellow_cone.point.x = pOSO->RefPnt.ds[0] /*+ obj_center.getX()*/; // + CMNode.Sensor.Object.pos[0];
          yellow_cone.point.y = pOSO->RefPnt.ds[1] /*+ obj_center.getY()*/;
          yellow_cone.point.z = pOSO->RefPnt.ds[2] /*+ obj_center.getZ()*/;
          msg.yellow_cones.push_back(yellow_cone);
          
        } else if (!strcmp(pObj->Cfg.Info, "Left Blue Cone")) {
          /* blue */
          bristol_msgs::msg::ConeWithCovariance blue_cone;
          blue_cone.point.x = pOSO->RefPnt.ds[0] /*+ obj_center.getX()*/; // + CMNode.Sensor.Object.pos[0];
          blue_cone.point.y = pOSO->RefPnt.ds[1] /*+ obj_center.getY()*/;
          blue_cone.point.z = pOSO->RefPnt.ds[2] /*+ obj_center.getZ()*/;
          msg.blue_cones.push_back(blue_cone);
        } else {
          /* orange start cones */
          bristol_msgs::msg::ConeWithCovariance orange_cone;
          orange_cone.point.x = pOSO->RefPnt.ds[0] /*+ obj_center.getX()*/; // + CMNode.Sensor.Object.pos[0];
          orange_cone.point.y = pOSO->RefPnt.ds[1] /*+ obj_center.getY()*/;
          orange_cone.point.z = pOSO->RefPnt.ds[2] /*+ obj_center.getZ()*/;
          msg.orange_cones.push_back(orange_cone);
        }
      }      
    }
  }
}

void CMNodeHelloCM::objectlistFillConeMsg(bristol_msgs::msg::ConeArrayWithCovariance& msg){
  if (CMNode.Sensor.Object.Active) {  

    list<bristol_msgs::msg::ConeWithCovariance> blue_cones = {};
    list<bristol_msgs::msg::ConeWithCovariance> yellow_cones = {};
    list<bristol_msgs::msg::ConeWithCovariance> orange_cones = {};
    list<bristol_msgs::msg::ConeWithCovariance> big_orange_cones = {};
    list<bristol_msgs::msg::ConeWithCovariance> unknown_color_cones = {};

    rclcpp::Time object_sensor_stamp = rclcpp::Time(static_cast<int64_t>(1e9 * ObjectSensor[0].TimeStamp), RCL_ROS_TIME);

    /* Clear vector data to avoid overflows */
    msg.blue_cones.clear();
    msg.yellow_cones.clear();
    msg.orange_cones.clear();
    msg.big_orange_cones.clear();
    msg.unknown_color_cones.clear();

    // EUFS cone noise model
    double camera_a_noise_param = 0.0184;
    double camera_b_noise_param = 0.2106;

    for (int j = 0; j < ObjectSensor[0].nObsvObjects; j++) {
      int obj_id = ObjectSensor[0].ObsvObjects[j];
      tObjectSensorObj *pOSO = ObjectSensor_GetObjectByObjId(0, obj_id);
      tTrafficObj *pObj = Traffic_GetByObjId(obj_id);

      // msg.header.frame_id = CMNode.TF.ObjectList.child_frame_id;
      msg.header.frame_id = CMNode.TF.ObjectList.child_frame_id;
      msg.header.stamp = object_sensor_stamp;


      /* Object rotation as quaternion */
      tf2::Quaternion rotation;
      rotation.setRPY(pOSO->RefPnt.r_zyx[0], pOSO->RefPnt.r_zyx[1], pOSO->RefPnt.r_zyx[2]);

      /* Apply rotation to vector pointing to the object's center */
      tf2::Vector3 obj_center = tf2::quatRotate(rotation, tf2::Vector3(0.5 * pOSO->l, 0, 0));

      double pos_x = pOSO->RefPnt.ds[0] /*+ obj_center.getX()*/; // + CMNode.Sensor.Object.pos[0];
      double pos_y = pOSO->RefPnt.ds[1] /*+ obj_center.getY()*/;
      double pos_z = pOSO->RefPnt.ds[2] - obj_center.getZ();

      // Apply noise model (EUFS cone noise model)
      auto dist = sqrt(pos_x * pos_x + pos_y * pos_y);
      auto x_noise = camera_a_noise_param * std::fmin(70.0, std::exp(camera_b_noise_param * dist));
      auto y_noise = x_noise / 5;

      // Add noise in direction of cone position vector
      double par_x = pos_x / dist;
      double par_y = pos_y / dist;

      // Generate perpendicular unit vector
      auto perp_x = -1.0 * par_y;
      auto perp_y = par_x;

      // Create noise vector
      auto par_noise = gaussianKernel(0, x_noise);
      par_x *= par_noise;
      par_y *= par_noise;
      auto perp_noise = gaussianKernel(0, y_noise);
      perp_x *= perp_noise;
      perp_y *= perp_noise;

      // Add noise vector to cone pose
      pos_x += par_x + perp_x;
      pos_y += par_y + perp_y;


      if (pOSO->dtct || pOSO->InLane) {
        if (!strcmp(pObj->Cfg.Info, "Right Yellow Cone")) {
            /* yellow */
            bristol_msgs::msg::ConeWithCovariance yellow_cone;
            yellow_cone.point.x = pos_x;
            yellow_cone.point.y = pos_y;
            yellow_cone.point.z = pos_z;
            // yellow_cone.covariance = {x_noise, 0, 0, y_noise}; // Add covariance
            msg.yellow_cones.push_back(yellow_cone);

        } else if (!strcmp(pObj->Cfg.Info, "Left Blue Cone")) {
            /* blue */
            bristol_msgs::msg::ConeWithCovariance blue_cone;
            blue_cone.point.x = pos_x;
            blue_cone.point.y = pos_y;
            blue_cone.point.z = pos_z;
            // blue_cone.covariance = {x_noise, 0, 0, y_noise}; // Add covariance
            msg.blue_cones.push_back(blue_cone);

        } else {
            /* orange start cones */
            bristol_msgs::msg::ConeWithCovariance orange_cone;
            orange_cone.point.x = pos_x;
            orange_cone.point.y = pos_y;
            orange_cone.point.z = pos_z;
            // orange_cone.covariance = {x_noise, 0, 0, y_noise}; // Add covariance
            msg.orange_cones.push_back(orange_cone);
        }
      }
    }
  }
}

// TODO, can we pass an extra argument (e.g. sensor name) to the function in order to reduce copy paste
void CMNodeHelloCM::objectlistFillLidarConeMsg(bristol_msgs::msg::ConeArrayWithCovariance& msg){
  if (CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.Active) {
    int obj_sensor = CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.index;

    list<bristol_msgs::msg::ConeWithCovariance> blue_cones = {};
    list<bristol_msgs::msg::ConeWithCovariance> yellow_cones = {};
    list<bristol_msgs::msg::ConeWithCovariance> orange_cones = {};
    list<bristol_msgs::msg::ConeWithCovariance> big_orange_cones = {};
    list<bristol_msgs::msg::ConeWithCovariance> unknown_color_cones = {};

    rclcpp::Time object_sensor_stamp = rclcpp::Time(static_cast<int64_t>(1e9 * ObjectSensor[obj_sensor].TimeStamp), RCL_ROS_TIME);
    /* Clear vector data to avoid overflows */
    msg.blue_cones.clear();
    msg.yellow_cones.clear();
    msg.orange_cones.clear();
    msg.big_orange_cones.clear();
    msg.unknown_color_cones.clear();

    // EUFS cone noise model
    double camera_a_noise_param = 0.001;
    double camera_b_noise_param = 0.01;

    for (int j = 0; j < ObjectSensor[obj_sensor].nObsvObjects; j++) {
      
      int obj_id = ObjectSensor[obj_sensor].ObsvObjects[j];
      tObjectSensorObj *pOSO = ObjectSensor_GetObjectByObjId(obj_sensor, obj_id);
      tTrafficObj *pObj = Traffic_GetByObjId(obj_id);

      msg.header.frame_id = CMNode.TF.Lidar_Pandar40p_Obj_Detect.child_frame_id;
      // msg.header.frame_id = "Fr1A";
      msg.header.stamp = object_sensor_stamp;


      /* Object rotation as quaternion */
      tf2::Quaternion rotation;
      rotation.setRPY(pOSO->RefPnt.r_zyx[0], pOSO->RefPnt.r_zyx[1], pOSO->RefPnt.r_zyx[2]);

      /* Apply rotation to vector pointing to the object's center */
      tf2::Vector3 obj_center = tf2::quatRotate(rotation, tf2::Vector3(0.5 * pOSO->l, 0, 0));

      double pos_x = pOSO->RefPnt.ds[0] /*+ obj_center.getX()*/; //+ CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.pos[0];
      double pos_y = pOSO->RefPnt.ds[1] /*+ obj_center.getY()*/;
      double pos_z = pOSO->RefPnt.ds[2] - obj_center.getZ();

      // Apply noise model (EUFS cone noise model)
      auto dist = sqrt(pos_x * pos_x + pos_y * pos_y);
      auto x_noise = camera_a_noise_param * std::fmin(70.0, std::exp(camera_b_noise_param * dist));
      auto y_noise = x_noise / 5;

      // Add noise in direction of cone position vector
      double par_x = pos_x / dist;
      double par_y = pos_y / dist;

      // Generate perpendicular unit vector
      auto perp_x = -1.0 * par_y;
      auto perp_y = par_x;

      // Create noise vector
      auto par_noise = gaussianKernel(0, x_noise);
      par_x *= par_noise;
      par_y *= par_noise;
      auto perp_noise = gaussianKernel(0, y_noise);
      perp_x *= perp_noise;
      perp_y *= perp_noise;

      // Add noise vector to cone pose
      pos_x += par_x + perp_x;
      pos_y += par_y + perp_y;


      if (pOSO->dtct || pOSO->InLane) {
        if (
          (!strcmp(pObj->Cfg.Info, "Right Yellow Cone")) or 
          (!strcmp(pObj->Cfg.Info, "Left Blue Cone")) or 
          (!strcmp(pObj->Cfg.Info, "Large Orange Start Cone")) or
          (!strcmp(pObj->Cfg.Info, "TrafficCone_Large_Orange"))
        ){
            /* any type of cone */
            bristol_msgs::msg::ConeWithCovariance unknown_colour_cone;
            unknown_colour_cone.point.x = pos_x;
            unknown_colour_cone.point.y = pos_y;
            unknown_colour_cone.point.z = pos_z;
            // unknown_color_cones.covariance = {x_noise, 0, 0, y_noise}; // Add covariance
            msg.unknown_color_cones.push_back(unknown_colour_cone);

        }
      }
    }
  }
}

// Helper function for modelling cone noise
double CMNodeHelloCM::gaussianKernel(double mu, double sigma) {
    // Using Box-Muller transform to generate Gaussian noise

    // Normalized uniform random variable
    double U = static_cast<double>(rand_r(&this->seed)) / static_cast<double>(RAND_MAX);

    // Normalized uniform random variable
    double V = static_cast<double>(rand_r(&this->seed)) / static_cast<double>(RAND_MAX);

    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
    // Scale to mean (mu) and standard deviation (sigma)
    X = sigma * X + mu;
    return X;
}

void CMNodeHelloCM::cameraFillMsg(camera_msgs::msg::CameraDetectionArray& msg) {

  if (CMNode.Sensor.Camera.Active) {

    camera_msgs::msg::CameraDetection detections; 

    /* Clear vector data to avoid overflows */
    msg.detections.clear();

    for (int k = 0; k <= CameraSensor[0].nObj; k++) {

      detections.objid            = int32_t(CameraSensor[0].Obj[k].ObjID);
      detections.nvispixels       = int64_t(CameraSensor[0].Obj[k].nVisPixels);
      detections.confidence       = CameraSensor[0].Obj[k].Confidence;

      detections.objecttype       = CameraSensor[0].Obj[k].Type;

      detections.mbr_bl_x         = CameraSensor[0].Obj[k].MBR[0][0];
      detections.mbr_bl_y         = CameraSensor[0].Obj[k].MBR[0][1];
      detections.mbr_bl_z         = CameraSensor[0].Obj[k].MBR[0][2];
      detections.mbr_tr_x         = CameraSensor[0].Obj[k].MBR[1][0];
      detections.mbr_tr_y         = CameraSensor[0].Obj[k].MBR[1][1];
      detections.mbr_tr_z         = CameraSensor[0].Obj[k].MBR[1][2];

      detections.facing           = int8_t(CameraSensor[0].Obj[k].Facing);
      detections.lightstate       = int8_t(CameraSensor[0].Obj[k].LightState);

      detections.signmain_val0    = CameraSensor[0].Obj[k].SignMainVal[0];
      detections.signmain_val1    = CameraSensor[0].Obj[k].SignMainVal[1];

      detections.signsuppl1_val0  = CameraSensor[0].Obj[k].SignSuppl1Val[0];
      detections.signsuppl1_val1  = CameraSensor[0].Obj[k].SignSuppl1Val[1];

      detections.signsuppl2_val0  = CameraSensor[0].Obj[k].SignSuppl2Val[0];
      detections.signsuppl2_val1  = CameraSensor[0].Obj[k].SignSuppl2Val[1];

      msg.detections.push_back(detections);
    } // for (int k = 0; k <= CameraSensor[0].nObj; k++)
  } // if (CMNode.Sensor.Camera.Active)
}

void CMNodeHelloCM::cm2extFillMsg(hellocm_msgs::msg::CM2Ext& msg) {
  msg.cycleno = static_cast<uint32_t>(rel_cycle_num_);
  msg.time = rclcpp::Time(static_cast<int64_t>(1e9 * SimCore.Time), RCL_ROS_TIME);
  msg.synthdelay = synth_delay_;
  msg.header.stamp = rclcpp::Clock().now();
}

void CMNodeHelloCM::cm2extHelloWorldFillMsg(std_msgs::msg::String& msg) {
  msg.data = "Hello World: CM2Isaac";
}

void CMNodeHelloCM::collisionFillMsg(std_msgs::msg::Bool& msg) {
  if (CMNode.Misc.collision_sensor_active) {
    if (CollisionSensor.Fr1.ObjId == -1){
      msg.data = false;
    } else{
      msg.data = true;
    }
  }
}

void CMNodeHelloCM::pcan_gps_gt_fill_msg(sensor_msgs::msg::NavSatFix& msg) {
  if (CMNode.Sensor.GCS.Active) {

    // TODO: Construct header, need a way to get latest road update, possibly stamp using current time?
    // rclcpp::Time object_sensor_stamp = rclcpp::Time(static_cast<int64_t>(1e9 * ObjectSensor[0].TimeStamp), RCL_ROS_TIME);

    tDDictEntry *entry = nullptr; // Use tDDictEntry for entries in the Data Dictionary
    double longitude = 0.0, latitude = 0.0, elevation = 0.0;

    // Fetch Longitude
    entry = DDictGetEntry("Car.Road.GCS.Long");
    if (entry != nullptr && entry->Type == DDictDouble) { // Use DDictDouble for double-precision values
        longitude = *static_cast<double *>(entry->Var); // Access data via Var
    } else {
        std::cerr << "Error: Unable to fetch 'Car.Road.GCS.Long'" << std::endl;
    }

    // Fetch Latitude
    entry = DDictGetEntry("Car.Road.GCS.Lat");
    if (entry != nullptr && entry->Type == DDictDouble) { // Use DDictDouble
        latitude = *static_cast<double *>(entry->Var); // Access data via Var
    } else {
        std::cerr << "Error: Unable to fetch 'Car.Road.GCS.Lat'" << std::endl;
    }

    // Fetch Elevation
    entry = DDictGetEntry("Car.Road.GCS.Elev");
    if (entry != nullptr && entry->Type == DDictDouble) { // Use DDictDouble
        elevation = *static_cast<double *>(entry->Var); // Access data via Var
    } else {
        std::cerr << "Error: Unable to fetch 'Car.Road.GCS.Elev'" << std::endl;
    }

    // Convert radians to degrees for latitude and longitude if needed
    double latitude_deg = latitude * (180.0 / M_PI);
    double longitude_deg = longitude * (180.0 / M_PI);

    msg.header.stamp = rclcpp::Clock().now();
    msg.latitude = latitude_deg;
    msg.longitude = longitude_deg;
    msg.altitude = elevation;

  }
}

// The changes for gps by coder
#include <random>  // Include C++ random library

std::random_device rd;  
std::mt19937 gen(rd());  // Random number generator
// std::normal_distribution<double> gps_noise(0.0, 3.0);  // Mean 0, std 3 meters

// Initialize distributions for stochastic noise
std::normal_distribution<double> lat_noise(0.0, 0.00003);   // Latitude noise
std::normal_distribution<double> lon_noise(0.0, 0.00003);   // Longitude noise
std::normal_distribution<double> alt_noise(0.0, 5.0);       // Altitude noise

// Stochastic drift for random walk
std::normal_distribution<double> drift_noise(0.0, 0.00001);  // Small drift in GPS error

void CMNodeHelloCM::pcan_gps_fill_msg(sensor_msgs::msg::NavSatFix& msg) {
  if (CMNode.Sensor.GCS.Active) {

    // msg.latitude = 0.2;

    tDDictEntry *entry = nullptr;
        double longitude = 0.0, latitude = 0.0, elevation = 0.0;

        // Fetch Longitude
        entry = DDictGetEntry("Car.Road.GCS.Long");
        if (entry != nullptr && entry->Type == DDictDouble) {
            longitude = *static_cast<double *>(entry->Var);
        } 

        // Fetch Latitude
        entry = DDictGetEntry("Car.Road.GCS.Lat");
        if (entry != nullptr && entry->Type == DDictDouble) {
            latitude = *static_cast<double *>(entry->Var);
        }

        // Fetch Elevation
        entry = DDictGetEntry("Car.Road.GCS.Elev");
        if (entry != nullptr && entry->Type == DDictDouble) {
            elevation = *static_cast<double *>(entry->Var);
        }

        // Convert radians to degrees
        double latitude_deg = latitude * (180.0 / M_PI);
        double longitude_deg = longitude * (180.0 / M_PI);

        // // Add Gaussian noise
        // latitude_deg += gps_noise(gen);
        // longitude_deg += gps_noise(gen);
        // elevation += gps_noise(gen);

        // Adjust longitude noise based on latitude
        double lon_std_adjusted = 0.00003 / cos(latitude);
        std::normal_distribution<double> lon_noise_adjusted(0.0, lon_std_adjusted);

        // Apply Gaussian noise as stochastic error
        latitude_deg += lat_noise(gen);
        longitude_deg += lon_noise_adjusted(gen);
        elevation += alt_noise(gen);

        // Stochastic drift (random walk) applied to the noise
        latitude_deg += drift_noise(gen);  // Drift effect on latitude
        longitude_deg += drift_noise(gen);  // Drift effect on longitude
        elevation += drift_noise(gen);  // Drift effect on altitude


        // Populate ROS message
        msg.header.stamp = rclcpp::Clock().now();
        msg.latitude = latitude_deg;
        msg.longitude = longitude_deg;
        msg.altitude = elevation;
  }
}

void CMNodeHelloCM::ws_fill_msg(ads_dv_msgs::msg::WheelSpeeds& msg) {
    tDDictEntry* entry = nullptr;

    // Variables to store fetched values
    double fl_speed = 0.0, fr_speed = 0.0, rl_speed = 0.0, rr_speed = 0.0, steering_angle = 0.0;

    // Fetch Steering Angle (in radians)
    entry = DDictGetEntry("Vhcl.Steer.Ang");
    if (entry != nullptr && entry->Type == DDictDouble) {
        steering_angle = *static_cast<double*>(entry->Var);
        msg.steer_angle_deg = static_cast<float>(steering_angle * (27.28 / M_PI)); // Convert steering wheel angle to steering rack angle (limit of 27.28 degrees)
    } else {
        std::cerr << "Error: Unable to fetch 'Car.Steer.Ang'" << std::endl;
        msg.steer_angle_deg = 0.0f;
    }

    // Fetch Front Left Wheel Speed (Rad/s)
    entry = DDictGetEntry("Car.WheelSpd_FL");
    if (entry != nullptr && entry->Type == DDictDouble) {
        fl_speed = *static_cast<double*>(entry->Var);
        msg.fl_speed_rpm = static_cast<uint16_t>(fl_speed * 60.0 / (2.0 * M_PI)); // Convert to RPM
    } else {
        std::cerr << "Error: Unable to fetch 'Car.WheelSpd_FL'" << std::endl;
        msg.fl_speed_rpm = 0;
    }

    // Fetch Front Right Wheel Speed (Rad/s)
    entry = DDictGetEntry("Car.WheelSpd_FR");
    if (entry != nullptr && entry->Type == DDictDouble) {
        fr_speed = *static_cast<double*>(entry->Var);
        msg.fr_speed_rpm = static_cast<uint16_t>(fr_speed * 60.0 / (2.0 * M_PI)); // Convert to RPM
    } else {
        std::cerr << "Error: Unable to fetch 'Car.WheelSpd_FR'" << std::endl;
        msg.fr_speed_rpm = 0;
    }

    // Fetch Rear Left Wheel Speed (Rad/s)
    entry = DDictGetEntry("Car.WheelSpd_RL");
    if (entry != nullptr && entry->Type == DDictDouble) {
        rl_speed = *static_cast<double*>(entry->Var);
        msg.rl_speed_rpm = static_cast<uint16_t>(rl_speed * 60.0 / (2.0 * M_PI)); // Convert to RPM
    } else {
        std::cerr << "Error: Unable to fetch 'Car.WheelSpd_RL'" << std::endl;
        msg.rl_speed_rpm = 0;
    }

    // Fetch Rear Right Wheel Speed (Rad/s)
    entry = DDictGetEntry("Car.WheelSpd_RR");
    if (entry != nullptr && entry->Type == DDictDouble) {
        rr_speed = *static_cast<double*>(entry->Var);
        msg.rr_speed_rpm = static_cast<uint16_t>(rr_speed * 60.0 / (2.0 * M_PI)); // Convert to RPM
    } else {
        std::cerr << "Error: Unable to fetch 'Car.WheelSpd_RR'" << std::endl;
        msg.rr_speed_rpm = 0;
    }

    // Pulses per minute.
    msg.rr_pulse_count = (msg.rr_speed_rpm) * 20; 
    msg.rl_pulse_count = (msg.rl_speed_rpm) * 20;
    msg.fr_pulse_count = (msg.fr_speed_rpm) * 20;
    msg.fl_pulse_count = (msg.rl_speed_rpm) * 20;


    // Debug output
    // std::cout << "Steering Angle: " << msg.steer_angle_deg << "°" << std::endl;
    // std::cout << "Wheel Speeds (RPM): FL=" << msg.fl_speed_rpm << ", FR=" << msg.fr_speed_rpm
    //           << ", RL=" << msg.rl_speed_rpm << ", RR=" << msg.rr_speed_rpm << std::endl;
}

void CMNodeHelloCM::IMU_reading(sensor_msgs::msg::Imu& msg) {
  //tDDictEntry* entry = nullptr;

  // Change B with 0 to change it from body frame to global frame
  msg.angular_velocity.x = InertialSensor[0].Omega_B[0]; //Rotational Velocity of IMU in BODY frame
  msg.angular_velocity.y = InertialSensor[0].Omega_B[1];
  msg.angular_velocity.z = InertialSensor[0].Omega_B[2];

  msg.linear_acceleration.x = InertialSensor[0].Acc_B[0]; // Translational acceleration of IMU in BODY frame
  msg.linear_acceleration.y = InertialSensor[0].Acc_B[1];
  msg.linear_acceleration.z = InertialSensor[0].Acc_B[2];


  
}

void CMNodeHelloCM::wheel_speeds_noisy(ads_dv_msgs::msg::WheelSpeeds& msg) {
    tDDictEntry* entry = nullptr;
    // Variables to store fetched values
    double fl_speed = 0.0, fr_speed = 0.0, rl_speed = 0.0, rr_speed = 0.0, steering_angle = 0.0;
    auto steering_noise = 0.001;
    auto wheel_noise = 0.1;

    // Fetch Steering Angle (in radians)
    entry = DDictGetEntry("Vhcl.Steer.Ang");
    if (entry != nullptr && entry->Type == DDictDouble) {
        steering_angle = *static_cast<double*>(entry->Var);
        if (steering_angle < 0){
          steering_angle = steering_angle + gaussianKernel(0,steering_noise);
        }
        if (steering_angle > 0){
          steering_angle = steering_angle + gaussianKernel(0,steering_noise);
        }
        msg.steer_angle_deg = static_cast<float>(steering_angle * (27.28 / M_PI)); // Convert steering wheel angle to steering rack angle (limit of 27.28 degrees)
    } else {
        std::cerr << "Error: Unable to fetch 'Car.Steer.Ang'" << std::endl;
        msg.steer_angle_deg = 0.0f;
    }

    // Fetch Front Left Wheel Speed (Rad/s)
    entry = DDictGetEntry("Car.WheelSpd_FL");
    if (entry != nullptr && entry->Type == DDictDouble) {
        fl_speed = *static_cast<double*>(entry->Var);
        if (fl_speed > 1){
          fl_speed = fl_speed + gaussianKernel(0,wheel_noise);
        }
        msg.fl_speed_rpm = static_cast<uint16_t>(fl_speed * 60.0 / (2.0 * M_PI)); // Convert to RPM
    } else {
        std::cerr << "Error: Unable to fetch 'Car.WheelSpd_FL'" << std::endl;
        msg.fl_speed_rpm = 0;
    }

    // Fetch Front Right Wheel Speed (Rad/s)
    entry = DDictGetEntry("Car.WheelSpd_FR");
    if (entry != nullptr && entry->Type == DDictDouble) {
        fr_speed = *static_cast<double*>(entry->Var);
        if (fr_speed > 1){
          fr_speed = fr_speed + gaussianKernel(0,wheel_noise);
        }
        if (static_cast<uint16_t>(rr_speed * 60.0 / (2.0 * M_PI)) > 50000){
          msg.rr_speed_rpm = 0;
        }
        msg.fr_speed_rpm = static_cast<uint16_t>(fr_speed * 60.0 / (2.0 * M_PI)); // Convert to RPM
    } else {
        std::cerr << "Error: Unable to fetch 'Car.WheelSpd_FR'" << std::endl;
        msg.fr_speed_rpm = 0;
    }

    // Fetch Rear Left Wheel Speed (Rad/s)
    entry = DDictGetEntry("Car.WheelSpd_RL");
    if (entry != nullptr && entry->Type == DDictDouble) {
        rl_speed = *static_cast<double*>(entry->Var);
        if (rl_speed > 1){
          rl_speed = rl_speed + gaussianKernel(0,wheel_noise);
        }
        if (static_cast<uint16_t>(rr_speed * 60.0 / (2.0 * M_PI)) > 50000){
          msg.rr_speed_rpm = 0;
        }
        msg.rl_speed_rpm = static_cast<uint16_t>(rl_speed * 60.0 / (2.0 * M_PI)); // Convert to RPM
    } else {
        std::cerr << "Error: Unable to fetch 'Car.WheelSpd_RL'" << std::endl;
        msg.rl_speed_rpm = 0;
    }

    // Fetch Rear Right Wheel Speed (Rad/s)
    entry = DDictGetEntry("Car.WheelSpd_RR");
    if (entry != nullptr && entry->Type == DDictDouble) {
        rr_speed = *static_cast<double*>(entry->Var);
        if (rr_speed > 1){
          rr_speed = rr_speed + gaussianKernel(0,wheel_noise);
        }
        msg.rr_speed_rpm = static_cast<uint16_t>(rr_speed * 60.0 / (2.0 * M_PI)); // Convert to RPM
    } else {
        std::cerr << "Error: Unable to fetch 'Car.WheelSpd_RR'" << std::endl;
        msg.rr_speed_rpm = 0;
    }

    // Pulses per minute.
    msg.rr_pulse_count = (msg.rr_speed_rpm) * 20; 
    msg.rl_pulse_count = (msg.rl_speed_rpm) * 20;
    msg.fr_pulse_count = (msg.fr_speed_rpm) * 20;
    msg.fl_pulse_count = (msg.rl_speed_rpm) * 20;

}


void CMNodeHelloCM::lap_stat_fill_msg(sim_msgs::msg::LapStats& msg) {
    tDDictEntry* entry = nullptr;

    // Fetch current lap number
    entry = DDictGetEntry("DM.Lap.No");
    if (entry != nullptr && entry->Type == DDictInt) {
        msg.current_lap_n = *static_cast<int*>(entry->Var);
    } else {
        std::cerr << "Error: Unable to fetch 'DM.Lap.No'" << std::endl;
        msg.current_lap_n = 0;
    }

    // Fetch previous lap time (in seconds)
    float prev_lap_time = 0.0f;
    entry = DDictGetEntry("DM.Lap.Time");
    if (entry != nullptr && entry->Type == DDictDouble) {
        prev_lap_time = static_cast<float>(*static_cast<double*>(entry->Var)); // Convert to float
    } else {
        std::cerr << "Error: Unable to fetch 'DM.Lap.PrevTime'" << std::endl;
        prev_lap_time = 0.0f;
    }

    // Update lap history
    if (msg.current_lap_n == 0) {
        msg.lap_history.clear(); // Clear history if no laps completed
    } else if (msg.current_lap_n != current_lap) {
        msg.lap_history.push_back(prev_lap_time); // Add previous lap time to history
        current_lap = msg.current_lap_n; // Update the current lap number
    }

    // TODO: Can we include the current time of the lap we're on? Do we even need this value (currently in sim_msgs)
    // TODO: Generate header
    // TODO: Are laptime linked to car start or orange cones crossing
}


void CMNodeHelloCM::pose_gt_fill_msg(geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    // // Variables to store fetched values
    double tx = 0.0, ty = 0.0, tz = 0.0, rx = 0.0, ry = 0.0, rz = 0.0;

    tx = Car.Fr1.t_0[0];
    ty = Car.Fr1.t_0[1];
    tz = Car.Fr1.t_0[2];
    rx = Car.Fr1.r_zyx[0];
    ry = Car.Fr1.r_zyx[1];
    rz = Car.Fr1.r_zyx[2];
    
    msg.pose.pose.position.x = tx;
    msg.pose.pose.position.y = ty;
    msg.pose.pose.position.z = tz;

    // Step 1: Create a tf2::Quaternion and set its values using roll, pitch, yaw
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(rx, ry, rz); // Roll (X), Pitch (Y), Yaw (Z)

    // Step 2: Convert to a geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion quaternion_msg;
    msg.pose.pose.orientation.x = tf2_quaternion.x();
    msg.pose.pose.orientation.y = tf2_quaternion.y();
    msg.pose.pose.orientation.z = tf2_quaternion.z();
    msg.pose.pose.orientation.w = tf2_quaternion.w();
    
    // msg.header.frame_id = "Fr0";
    // msg.header.stamp = rclcpp::Clock().now();
    auto now = rclcpp::Time(static_cast<int64_t>(1e9 * SimCore.Time), RCL_ROS_TIME);
    // rclcpp::Time now = this->get_clock()->now();
    msg.header.frame_id = "Fr0";
    msg.header.stamp = now;

    // Now broadcast TF
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "Fr0";     // World/map frame
    transform.child_frame_id = "Fr1A";     // Car frame

    transform.transform.translation.x = tx;
    transform.transform.translation.y = ty;
    transform.transform.translation.z = tz;

    transform.transform.rotation.x = tf2_quaternion.x();
    transform.transform.rotation.y = tf2_quaternion.y();
    transform.transform.rotation.z = tf2_quaternion.z();
    transform.transform.rotation.w = tf2_quaternion.w();

    tf_broadcaster_->sendTransform(transform);
}


/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Add user specific Quantities for data storage
 *   and visualization to DataDictionary
 * - Called once at program start
 * - no realtime conditions
 *
 */
void CMNodeHelloCM::userDeclQuants(void) {

    tDDefault *df = DDefaultCreate("CMRosIF.");

    DDefULong   (df, "CycleNoRel",         "ms",  &rel_cycle_num_,              DVA_None);
    DDefInt     (df, "Sync.Cycles",        "-",   &CMNode.Sync.nCycles,         DVA_None);
    DDefDouble  (df, "Sync.Time",          "s",   &CMNode.Sync.Duration,        DVA_None);
    DDefDouble4 (df, "Sync.SynthDelay",    "s",   &synth_delay_,                DVA_IO_In);

    DDefUChar   (df, "Cfg.Mode",           "-",   (unsigned char*)&node_mode_,  DVA_None);
    DDefInt     (df, "Cfg.nCyclesClock",   "ms",  &clock_cycle_time_,           DVA_None);
    DDefChar    (df, "Cfg.SyncMode",       "-",   (char*)&sync_mode_,           DVA_None);
    DDefDouble4 (df, "Cfg.SyncTimeMax",    "s",   &max_sync_time_,              DVA_IO_In);

    DDefaultDelete(df);
}

int CMNodeHelloCM::userTestrunStartAtBegin() {

  tErrorMsg *errv = nullptr;
  char sbuf[512];
  char *str         = nullptr;
  int idxC, idxS, idxP, ref;

  double def2c[]    = {0, 0};                // Default 2-col table data
  double def3c[]    = {0, 0, 0};             // Default 3-col table data

  /* Send transforms for coordinate systems */
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  tf2::Quaternion q;

  synth_delay_ = 1e-6;

  /* Prepare external node for next simulation */
  if (!srv_init_->wait_for_service(std::chrono::seconds(2))) {
    Log::printError(EC_Sim,
                    "ROS service is not ready! Please start external ROS node "
                    "providing service '" + std::string(srv_init_->get_service_name()) + "'!");
    node_mode_ = NodeMode::kDisabled;
    return -1;
  }

  // Reset the transform between Fr0 and Fr1A
  // -> prevents issues with loading a second track and data display
  tf_broadcaster_.reset();  // Destroy old one
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nhp_);
  geometry_msgs::msg::TransformStamped reset_tf;
  reset_tf.header.stamp = rclcpp::Clock().now();  // Current ROS time
  reset_tf.header.frame_id = "Fr0";
  reset_tf.child_frame_id = "Fr1A";
  reset_tf.transform.translation.x = 0.0;
  reset_tf.transform.translation.y = 0.0;
  reset_tf.transform.translation.z = 0.0;
  reset_tf.transform.rotation.w = 1.0;
  reset_tf.transform.rotation.x = 0.0;
  reset_tf.transform.rotation.y = 0.0;
  reset_tf.transform.rotation.z = 0.0;

  CMNode.TF.st_br->sendTransform(reset_tf);

  Log::printLog("  -> Sending service request");
  srv_init_->async_send_request(
      std::make_shared<hellocm_msgs::srv::Init::Request>());
  
  { /* set up cm2ext publisher + job */
    typedef CMJob::RosPublisher<hellocm_msgs::msg::CM2Ext> cm2ext_t;
    auto job = std::make_shared<cm2ext_t>(nhp_, "cm2ext");
    job->setCycleTime(15000);
    job->registerCallback(&CMNodeHelloCM::cm2extFillMsg, this);
    scheduler_.addJob(job);
  }

  {
    /* set up steer angle and wheelspeed publisher + job */
    typedef CMJob::RosPublisher<ads_dv_msgs::msg::WheelSpeeds> ws_publisher;
    auto job = std::make_shared<ws_publisher>(nhp_, "/VCU2AI/wheel_speeds");
    job->setCycleTime(1000.0/100); // can_node loops every 1/100 of a second
    job->registerCallback(&CMNodeHelloCM::ws_fill_msg, this);
    scheduler_.addJob(job);
  }

  { /* set up Test cm2ext publisher + job */
    typedef CMJob::RosPublisher<std_msgs::msg::String> cm2ext_t2;
    auto jobHW = std::make_shared<cm2ext_t2>(nhp_, "cm2extHelloWorld");
    jobHW->setCycleTime(15000);
    jobHW->registerCallback(&CMNodeHelloCM::cm2extHelloWorldFillMsg, this);
    scheduler_.addJob(jobHW);
  }

  CMNode.Sensor.GCS.Active = true;
  { /* set up PCAN GPS publisher + job (using road GCS rather than GNSS as IPG admin suggested solution) */
    typedef CMJob::RosPublisher<sensor_msgs::msg::NavSatFix> gps_publisher;
    auto job = std::make_shared<gps_publisher>(nhp_, "pcan_gps");
    job->setCycleTime(1000.0/10);
    job->registerCallback(&CMNodeHelloCM::pcan_gps_fill_msg, this);
    scheduler_.addJob(job);
  }
  { /* set up PCAN GPS publisher + job (using road GCS rather than GNSS as IPG admin suggested solution) */
    typedef CMJob::RosPublisher<sensor_msgs::msg::NavSatFix> gps_gt_publisher;
    auto job = std::make_shared<gps_gt_publisher>(nhp_, "ground_truth/pcan_gps");
    job->setCycleTime(1000.0/10);
    job->registerCallback(&CMNodeHelloCM::pcan_gps_gt_fill_msg, this);
    scheduler_.addJob(job);
  }

  { /* Noisy wheel speed and steer angle*/
    typedef CMJob::RosPublisher<ads_dv_msgs::msg::WheelSpeeds> ws_noisy_publisher;
    auto job = std::make_shared<ws_noisy_publisher>(nhp_, "/VCU2AI/wheel_speeds_noisy");
    job->setCycleTime(1000.0/100); // can_node loops every 1/100 of a second
    job->registerCallback(&CMNodeHelloCM::wheel_speeds_noisy, this);
    scheduler_.addJob(job);
  }

  { /* set up IMU reading */
    typedef CMJob::RosPublisher<sensor_msgs::msg::Imu> imu_publisher;
    auto job = std::make_shared<imu_publisher>(nhp_, "/IMU_Sensor");
    job->setCycleTime(1000.0/100); // can_node loops every 1/100 of a second
    job->registerCallback(&CMNodeHelloCM::IMU_reading, this);
    scheduler_.addJob(job);
  }

  { /* set up Autonomous State publisher + job */
    typedef CMJob::RosPublisher<ads_dv_msgs::msg::AutonomousState> as_state_publisher;
    auto job = std::make_shared<as_state_publisher>(nhp_, "/VCU2AI/as_state");
    job->setCycleTime(1000.0/100); // can_node loops every 1/100 of a second
    job->registerCallback(&CMNodeHelloCM::autonomousStateFillMsg, this);
    scheduler_.addJob(job);
  }

  { /* set up map ground truth publisher + job */
    typedef CMJob::RosPublisher<bristol_msgs::msg::ConeArrayWithCovariance> map_gt_publisher;
    auto job = std::make_shared<map_gt_publisher>(nhp_, "ground_truth/map");
    job->setCycleTime(1000.0/1); // can_node loops every 1/100 of a second
    job->registerCallback(&CMNodeHelloCM::mapFillGTMsg, this);
    scheduler_.addJob(job);
  }

  { /* set up map ground truth publisher + job */
    typedef CMJob::RosPublisher<visualization_msgs::msg::MarkerArray> map_gt_publisher2;
    auto job = std::make_shared<map_gt_publisher2>(nhp_, "ground_truth/rviz_map");
    job->setCycleTime(1000.0/1); // can_node loops every 1/100 of a second
    job->registerCallback(&CMNodeHelloCM::mapFillGTMsg2, this);
    scheduler_.addJob(job);
  }

  { /* set up lap history and current lap publisher + job */
    typedef CMJob::RosPublisher<sim_msgs::msg::LapStats> lap_stat_publisher;
    auto job = std::make_shared<lap_stat_publisher>(nhp_, "ground_truth/lap_stats");
    job->setCycleTime(1000.0/100); // can_node loops every 1/100 of a second
    job->registerCallback(&CMNodeHelloCM::lap_stat_fill_msg, this);
    scheduler_.addJob(job);
  }

  { /* set up ground truth car position + job */
    typedef CMJob::RosPublisher<geometry_msgs::msg::PoseWithCovarianceStamped> pose_gt_publisher;
    auto job = std::make_shared<pose_gt_publisher>(nhp_, "ground_truth/pose");
    job->setCycleTime(1000.0/100); // can_node loops every 1/100 of a second
    job->registerCallback(&CMNodeHelloCM::pose_gt_fill_msg, this);
    scheduler_.addJob(job);
  }

  /* Read sensor info from Vehicle InfoFile */
  tInfos *Inf_Vehicle = nullptr;
  Inf_Vehicle = InfoNew();

  const char *FName;
  FName = InfoGetFilename(SimCore.Vhcl.Inf);

  int VehicleInfo_Err = iRead2(&errv, Inf_Vehicle, FName, "SensorReadCode");

  if (VehicleInfo_Err == 0) {

    CMNode.Misc.nSensors = iGetIntOpt(Inf_Vehicle, "Sensor.N", 0);

    { 
    /* Find the first collision sensor in the Vehicle Infofile */
    std::cout << "Setting up collision sensor: Col00" << std::endl;
    for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
      sprintf(sbuf, "Sensor.%d.name", idxS);
      str = iGetStrOpt(Inf_Vehicle, sbuf, 0);

      // If match has been found, check sensor.active
      if (!strcmp(str, "Col00")){
        std::cout << "--> Found the first collision sensor: Col00" << std::endl;
        sprintf(sbuf, "Sensor.%d.Active", idxS);
        idxC = iGetIntOpt(Inf_Vehicle, sbuf, 0);

        CMNode.Misc.collision_sensor_active = idxC;

        // If active create job
        if (idxC){
          std::cout << "--> Creating publisher job" << std::endl;
          /* set up Test cm2ext publisher + job */
          typedef CMJob::RosPublisher<std_msgs::msg::Bool> collision_publisher;
          auto job = std::make_shared<collision_publisher>(nhp_, "collision_detected");
          job->setCycleTime(1000.0/2); // 2Hz, shouldn't be needed faster? Maybe 4Hz-10Hz max
          job->registerCallback(&CMNodeHelloCM::collisionFillMsg, this);
          scheduler_.addJob(job);
        
        // Else pass and break, only 1 collision sensor should be on a IPG vehicle
        }else{
          std::cout << "--> Inactive: ignoring" << std::endl;
          break;
        }
      }
    }
  }

    // { /* set up LidarRSI publisher + job */

    //   /* Read Sensor Infofile for LidarRSI */
    //   tInfos *Inf_Sensor = nullptr;
    //   tErrorMsg *errs = nullptr;

    //   Inf_Sensor = InfoNew();
    //   int SensorInfo_Err = iRead2(&errs, Inf_Sensor, "Data/Sensor/LidarRSI_FS_autonomous", "LidarCode");

    //   /* Extract LidarRSI Parameters from Sensor Infofile */
    //   if (SensorInfo_Err == 0) {
    //     CMNode.Sensor.LidarRSI.nBeams = iGetFixedTable2(Inf_Sensor, "Beams.N", 2, 1);
    //     CMNode.Sensor.LidarRSI.nBeams_h = CMNode.Sensor.LidarRSI.nBeams[0];
    //     CMNode.Sensor.LidarRSI.nBeams_v = CMNode.Sensor.LidarRSI.nBeams[1];
    //     CMNode.Sensor.LidarRSI.ret = iGetTableOpt(Inf_Sensor, "Beams", CMNode.Sensor.LidarRSI.BeamTable, tableSize, 6, &CMNode.Sensor.LidarRSI.rows);
    //     CMNode.Sensor.LidarRSI.FOV_h = iGetFixedTable2(Inf_Sensor, "Beams.FoVH", 2, 1); 
    //     CMNode.Sensor.LidarRSI.FOV_v = iGetFixedTable2(Inf_Sensor, "Beams.FoVV", 2, 1); 
    //     if (CMNode.Sensor.LidarRSI.ret == 0) {
    //       Log("Beam table read successfully. Table consists of %i rows\n", CMNode.Sensor.LidarRSI.rows);
    //       Log("FOVh = %f, FOVv = %f, nBeams_h = %f, nBeams_v = %f\n", 
    //         CMNode.Sensor.LidarRSI.FOV_h[0] + CMNode.Sensor.LidarRSI.FOV_h[1], 
    //         CMNode.Sensor.LidarRSI.FOV_v[0] + CMNode.Sensor.LidarRSI.FOV_v[1], 
    //         CMNode.Sensor.LidarRSI.nBeams_h, 
    //         CMNode.Sensor.LidarRSI.nBeams_v);
    //       Log("Beam Table --> First Azimuth Element = %f\n", CMNode.Sensor.LidarRSI.BeamTable[4*CMNode.Sensor.LidarRSI.rows]); 
    //       Log("Beam Table --> First Elevation Element = %f\n\n", CMNode.Sensor.LidarRSI.BeamTable[5*CMNode.Sensor.LidarRSI.rows]); 
    //     }
    //     CMNode.Sensor.LidarRSI.ret = InfoDelete(Inf_Sensor);
    //   } // if (SensorInfo_Err == 0)

      
    //   // Start of test code //
    //   /* Find the generic lidar in the Vehicle Infofile */
    //   int generic_lidar = -1;
    //   int lidar_index_count = -1;
    //   for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {

    //     sprintf(sbuf, "Sensor.%d.Active", idxS);
    //     int temp_active = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        
    //     sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
    //     ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
    //     sprintf(sbuf, "Sensor.Param.%d.Type", ref);
    //     str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        
    //     /* If a sensor is found of type LidarRSI, get its index and check what type of Lidar it is*/
    //     if (!strcmp(str, "LidarRSI") and temp_active) {
    //       idxP = ref;
    //       std::cout << "--> Lidar RSI found" << std::endl;


    //       lidar_index_count += 1;
          
    //       sprintf(sbuf, "Sensor.Param.%d.Beams.FName", idxP);
    //       std::cout << "--> Searching for " << sbuf << std::endl;
    //       str = iGetStrOpt(Inf_Vehicle, sbuf, "");
    //       std::cout << "--> " << sbuf << " = " << str << std::endl;
    //       /* If the Lidar is a generic FS_autonomous, we have found our sensor and can break. We also save the index for what number lidar sensor it is*/
    //       if (!strcmp(str, "LidarRSI_FS_autonomous")) {
    //         std::cout << "--> Found the LidarRSI_FS_autonomous" << std::endl;
    //         CMNode.Sensor.LidarRSI.index = lidar_index_count;
    //         break;
    //       }
    //     }
    //   }
    //   // *End of test code //



    //   /* Find the first LidarRSI in the Vehicle Infofile */
    //   idxP = -1;
    //   for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
    //     sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
    //     ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
    //     sprintf(sbuf, "Sensor.Param.%d.Type", ref);
    //     str = iGetStrOpt(Inf_Vehicle, sbuf, "");
    //     if (!strcmp(str, "LidarRSI")) {
    //       /* If the LidarRSI sensor is found, get its index and exit loop */
    //       idxP = ref;
    //       break;
    //     }
    //   }

    //   /* Store LidarRSI sensor parameters */
    //   if (idxP != -1) {
    //     sprintf(sbuf, "Sensor.%d.Active", idxS);
    //     CMNode.Sensor.LidarRSI.Active                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);
    //     sprintf(sbuf, "Sensor.%d.pos", idxS);
    //     CMNode.Sensor.LidarRSI.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
    //     sprintf(sbuf, "Sensor.%d.rot", idxS);
    //     CMNode.Sensor.LidarRSI.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
    //     sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
    //     CMNode.Sensor.LidarRSI.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 10);
    //     sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
    //     CMNode.Sensor.LidarRSI.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);

    //     /* Pass LidarRSI sensor parameters to job */
    //     q.setRPY(angles::from_degrees(CMNode.Sensor.LidarRSI.rot[0]), 
    //              angles::from_degrees(CMNode.Sensor.LidarRSI.rot[1]), 
    //              angles::from_degrees(CMNode.Sensor.LidarRSI.rot[2]));
    //     CMNode.TF.Lidar.transform.rotation = tf2::toMsg(q);
    //     CMNode.TF.Lidar.transform.translation.x = CMNode.Sensor.LidarRSI.pos[0];
    //     CMNode.TF.Lidar.transform.translation.y = CMNode.Sensor.LidarRSI.pos[1];
    //     CMNode.TF.Lidar.transform.translation.z = CMNode.Sensor.LidarRSI.pos[2];
    //     sprintf(sbuf, "Sensor.%d.name", idxS);
    //     CMNode.TF.Lidar.child_frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "LIR00");
    //     sprintf(sbuf, "Sensor.%d.Mounting", idxS);
    //     CMNode.TF.Lidar.header.frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "Fr1A");
    //     transforms.push_back(CMNode.TF.Lidar);

    //     typedef CMJob::RosPublisher<sensor_msgs::msg::PointCloud> Lidar;
    //     auto job = std::make_shared<Lidar>(nhp_, "pointcloud");
    //     job->setCycleTime(1000.0);
    //     job->setCycleOffset(CMNode.Sensor.LidarRSI.nCycleOffset);
    //     job->registerCallback(&CMNodeHelloCM::pointcloudFillMsg, this);
    //     scheduler_.addJob(job);
    //   } else {
    //     CMNode.Sensor.LidarRSI.Active                   = 0;
    //   }
    // } /* set up Lidar publisher + job */

    { /* set up LidarRSI_PS150 publisher + job */
      
      std::cout << "Setting up PS150 Lidar" << std::endl;

      /* Read Sensor Infofile for LidarRSI_PS150 */
      tInfos *Inf_Sensor = nullptr;
      tErrorMsg *errs = nullptr;

      Inf_Sensor = InfoNew();
      int SensorInfo_Err = iRead2(&errs, Inf_Sensor, "Data/Sensor/LidarRSI_PS150", "LidarCode");

      sprintf(sbuf, "SensorInfo_Err %i", SensorInfo_Err);

      /* Extract LidarRSI_PS150 Parameters from Sensor Infofile */
      if (SensorInfo_Err == 0) {
        CMNode.Sensor.LidarRSI_PS150.nBeams = iGetFixedTable2(Inf_Sensor, "Beams.N", 2, 1);
        CMNode.Sensor.LidarRSI_PS150.nBeams_h = CMNode.Sensor.LidarRSI_PS150.nBeams[0];
        CMNode.Sensor.LidarRSI_PS150.nBeams_v = CMNode.Sensor.LidarRSI_PS150.nBeams[1];
        CMNode.Sensor.LidarRSI_PS150.ret = iGetTableOpt(Inf_Sensor, "Beams", CMNode.Sensor.LidarRSI_PS150.BeamTable, ps150_tableSize, 6, &CMNode.Sensor.LidarRSI_PS150.rows);
        CMNode.Sensor.LidarRSI_PS150.FOV_h = iGetFixedTable2(Inf_Sensor, "Beams.FoVH", 2, 1); 
        CMNode.Sensor.LidarRSI_PS150.FOV_v = iGetFixedTable2(Inf_Sensor, "Beams.FoVV", 2, 1); 
        if (CMNode.Sensor.LidarRSI_PS150.ret == 0) {
          Log("Beam table read successfully. Table consists of %i rows\n", CMNode.Sensor.LidarRSI_PS150.rows);
          Log("FOVh = %f, FOVv = %f, nBeams_h = %f, nBeams_v = %f\n", 
            CMNode.Sensor.LidarRSI_PS150.FOV_h[0] + CMNode.Sensor.LidarRSI_PS150.FOV_h[1], 
            CMNode.Sensor.LidarRSI_PS150.FOV_v[0] + CMNode.Sensor.LidarRSI_PS150.FOV_v[1], 
            CMNode.Sensor.LidarRSI_PS150.nBeams_h, 
            CMNode.Sensor.LidarRSI_PS150.nBeams_v);
          Log("Beam Table --> First Azimuth Element = %f\n", CMNode.Sensor.LidarRSI_PS150.BeamTable[4*CMNode.Sensor.LidarRSI_PS150.rows]); 
          Log("Beam Table --> First Elevation Element = %f\n\n", CMNode.Sensor.LidarRSI_PS150.BeamTable[5*CMNode.Sensor.LidarRSI_PS150.rows]); 
        }
        CMNode.Sensor.LidarRSI_PS150.ret = InfoDelete(Inf_Sensor);
      } // if (SensorInfo_Err == 0)

      /* Find the PS150 in the Vehicle Infofile */
      int PS150_index = -1;
      int lidar_index_count = -1;
      for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {

        sprintf(sbuf, "Sensor.%d.Active", idxS);
        int temp_active = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        
        sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
        ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.Param.%d.Type", ref);
        str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        
        /* If a sensor is found of type LidarRSI, get its index and check what type of Lidar it is*/
        if (!strcmp(str, "LidarRSI") and temp_active) {
          idxP = ref;
          std::cout << "--> Lidar RSI found" << std::endl;


          lidar_index_count += 1;
          
          sprintf(sbuf, "Sensor.Param.%d.Beams.FName", idxP);
          std::cout << "--> Searching for " << sbuf << std::endl;
          str = iGetStrOpt(Inf_Vehicle, sbuf, "");
          std::cout << "--> " << sbuf << " = " << str << std::endl;
          /* If the Lidar is a PS150, we have found our sensor and can break. We also save the index for what number lidar sensor it is*/
          if (!strcmp(str, "LidarRSI_PS150")) {
            std::cout << "--> Found the PS150" << std::endl;
            CMNode.Sensor.LidarRSI_PS150.index = lidar_index_count;
            PS150_index = lidar_index_count;
            break;
          }
        }
      }

      /* Store LidarRSI_PS150 sensor parameters */
      if (PS150_index != -1) {
        std::cout << "--> Storing lidar params" << std::endl;
        sprintf(sbuf, "Sensor.%d.Active", idxS);
        CMNode.Sensor.LidarRSI_PS150.Active                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.%d.pos", idxS);
        CMNode.Sensor.LidarRSI_PS150.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.%d.rot", idxS);
        CMNode.Sensor.LidarRSI_PS150.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
        CMNode.Sensor.LidarRSI_PS150.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 10);
        sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
        CMNode.Sensor.LidarRSI_PS150.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);

        /* Pass LidarRSI_PS150 sensor parameters to job */
        q.setRPY(angles::from_degrees(CMNode.Sensor.LidarRSI_PS150.rot[0]), 
                 angles::from_degrees(CMNode.Sensor.LidarRSI_PS150.rot[1]), 
                 angles::from_degrees(CMNode.Sensor.LidarRSI_PS150.rot[2]));
        CMNode.TF.Lidar_PS150.transform.rotation = tf2::toMsg(q);
        CMNode.TF.Lidar_PS150.transform.translation.x = CMNode.Sensor.LidarRSI_PS150.pos[0];
        CMNode.TF.Lidar_PS150.transform.translation.y = CMNode.Sensor.LidarRSI_PS150.pos[1];
        CMNode.TF.Lidar_PS150.transform.translation.z = CMNode.Sensor.LidarRSI_PS150.pos[2];
        sprintf(sbuf, "Sensor.%d.name", idxS);
        CMNode.TF.Lidar_PS150.child_frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "LIR00");
        sprintf(sbuf, "Sensor.%d.Mounting", idxS);
        CMNode.TF.Lidar_PS150.header.frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "Fr1A");
        transforms.push_back(CMNode.TF.Lidar_PS150);

        typedef CMJob::RosPublisher<sensor_msgs::msg::PointCloud> Lidar;
        auto job = std::make_shared<Lidar>(nhp_, "PS150/pointcloud");
        job->setCycleTime(CMNode.Sensor.LidarRSI_PS150.UpdRate);
        // job->setCycleTime(1000.0);
        job->setCycleOffset(CMNode.Sensor.LidarRSI_PS150.nCycleOffset);
        job->registerCallback(&CMNodeHelloCM::PS150pointcloudFillMsg, this);
        scheduler_.addJob(job);
        std::cout << "--> Added job for PS150" << std::endl;
      } else {
        std::cout << "--> Lidar not found or inactive, no params stored, set non-active" << std::endl;
        CMNode.Sensor.LidarRSI_PS150.Active                   = 0;
      }
    } /* set up Lidar publisher + job */

    { /* set up Pandar40p Lidar publisher + job */
      std::cout << "Setting up Pandar40p Lidar" << std::endl;
      std::cout << "> Setting up raw pointcloud for lidar" << std::endl;
      const char* sensorName = "Param_Lidar_Pandar40p";

      /* Read Sensor Infofile for LidarRSI_Pandar40p */
      tInfos *Inf_Sensor = nullptr;
      tErrorMsg *errs = nullptr;

      Inf_Sensor = InfoNew();
      int SensorInfo_Err = iRead2(&errs, Inf_Sensor, "Data/Sensor/LidarRSI_Pandar40p", "LidarCode");

      sprintf(sbuf, "SensorInfo_Err %i", SensorInfo_Err);

      /* Extract LidarRSI_Pandar40p Parameters from Sensor Infofile */
      if (SensorInfo_Err == 0) {
        CMNode.Sensor.LidarRSI_Pandar40p.nBeams = iGetFixedTable2(Inf_Sensor, "Beams.N", 2, 1);
        CMNode.Sensor.LidarRSI_Pandar40p.nBeams_h = CMNode.Sensor.LidarRSI_Pandar40p.nBeams[0];
        CMNode.Sensor.LidarRSI_Pandar40p.nBeams_v = CMNode.Sensor.LidarRSI_Pandar40p.nBeams[1];
        CMNode.Sensor.LidarRSI_Pandar40p.ret = iGetTableOpt(Inf_Sensor, "Beams", CMNode.Sensor.LidarRSI_Pandar40p.BeamTable, pandar40p_tableSize, 6, &CMNode.Sensor.LidarRSI_Pandar40p.rows);
        CMNode.Sensor.LidarRSI_Pandar40p.FOV_h = iGetFixedTable2(Inf_Sensor, "Beams.FoVH", 2, 1); 
        CMNode.Sensor.LidarRSI_Pandar40p.FOV_v = iGetFixedTable2(Inf_Sensor, "Beams.FoVV", 2, 1); 
        if (CMNode.Sensor.LidarRSI_Pandar40p.ret == 0) {
          Log("Beam table read successfully. Table consists of %i rows\n", CMNode.Sensor.LidarRSI_Pandar40p.rows);
          Log("FOVh = %f, FOVv = %f, nBeams_h = %f, nBeams_v = %f\n", 
            CMNode.Sensor.LidarRSI_Pandar40p.FOV_h[0] + CMNode.Sensor.LidarRSI_Pandar40p.FOV_h[1], 
            CMNode.Sensor.LidarRSI_Pandar40p.FOV_v[0] + CMNode.Sensor.LidarRSI_Pandar40p.FOV_v[1], 
            CMNode.Sensor.LidarRSI_Pandar40p.nBeams_h, 
            CMNode.Sensor.LidarRSI_Pandar40p.nBeams_v);
          Log("Beam Table --> First Azimuth Element = %f\n", CMNode.Sensor.LidarRSI_Pandar40p.BeamTable[4*CMNode.Sensor.LidarRSI_Pandar40p.rows]); 
          Log("Beam Table --> First Elevation Element = %f\n\n", CMNode.Sensor.LidarRSI_Pandar40p.BeamTable[5*CMNode.Sensor.LidarRSI_Pandar40p.rows]); 
        }
        CMNode.Sensor.LidarRSI_Pandar40p.ret = InfoDelete(Inf_Sensor);
      } // if (SensorInfo_Err == 0)

      /* Find the Pandar40p in the Vehicle Infofile */
      int Pandar40p_index = -1;
      int lidar_index_count = -1;
      for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {

        sprintf(sbuf, "Sensor.%d.Active", idxS);
        int temp_active = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        
        sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
        ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.Param.%d.Type", ref);
        str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        
        /* If a sensor is found of type LidarRSI and it is active, get its index and check what type of Lidar it is*/
        if (!strcmp(str, "LidarRSI") and temp_active) {
          idxP = ref;
          std::cout << "--> Lidar RSI found" << std::endl;


          lidar_index_count += 1;
          
          sprintf(sbuf, "Sensor.Param.%d.Beams.FName", idxP);
          std::cout << "--> Searching for " << sbuf << std::endl;
          str = iGetStrOpt(Inf_Vehicle, sbuf, "");
          std::cout << "--> " << sbuf << " = " << str << std::endl;
          /* If the Lidar is a Pandar40p, we have found our sensor and can break. We also save the index for what number lidar sensor it is*/
          if (!strcmp(str, "LidarRSI_Pandar40p")) {
            std::cout << "--> Found the Pandar40p as LidarRSI[i]=" << std::endl;
            CMNode.Sensor.LidarRSI_Pandar40p.index = lidar_index_count;
            Pandar40p_index = lidar_index_count;
            break;
          }
        }
      }

      /* Store LidarRSI_Pandar40p sensor parameters */
      /* Only if sensor was found and was active during search */
      if (Pandar40p_index != -1) {
        std::cout << "--> Storing lidar params" << std::endl;
        sprintf(sbuf, "Sensor.%d.Active", idxS);
        CMNode.Sensor.LidarRSI_Pandar40p.Active                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.%d.pos", idxS);
        CMNode.Sensor.LidarRSI_Pandar40p.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.%d.rot", idxS);
        CMNode.Sensor.LidarRSI_Pandar40p.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
        CMNode.Sensor.LidarRSI_Pandar40p.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 10);
        sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
        CMNode.Sensor.LidarRSI_Pandar40p.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);

        /* Pass LidarRSI_Pandar40p sensor parameters to job */
        q.setRPY(angles::from_degrees(CMNode.Sensor.LidarRSI_Pandar40p.rot[0]), 
                 angles::from_degrees(CMNode.Sensor.LidarRSI_Pandar40p.rot[1]), 
                 angles::from_degrees(CMNode.Sensor.LidarRSI_Pandar40p.rot[2]));
        CMNode.TF.Lidar_Pandar40p.transform.rotation = tf2::toMsg(q);
        CMNode.TF.Lidar_Pandar40p.transform.translation.x = CMNode.Sensor.LidarRSI_Pandar40p.pos[0];
        CMNode.TF.Lidar_Pandar40p.transform.translation.y = CMNode.Sensor.LidarRSI_Pandar40p.pos[1];
        CMNode.TF.Lidar_Pandar40p.transform.translation.z = CMNode.Sensor.LidarRSI_Pandar40p.pos[2];
        sprintf(sbuf, "Sensor.%d.name", idxS);
        CMNode.TF.Lidar_Pandar40p.child_frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "LIR00");
        sprintf(sbuf, "Sensor.%d.Mounting", idxS);
        CMNode.TF.Lidar_Pandar40p.header.frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "Fr1A");
        transforms.push_back(CMNode.TF.Lidar_Pandar40p);
        
        typedef CMJob::RosPublisher<sensor_msgs::msg::PointCloud> Lidar;
        auto job = std::make_shared<Lidar>(nhp_, "Pandar40p/pointcloud");
        job->setCycleTime(CMNode.Sensor.LidarRSI_Pandar40p.UpdRate);
        // job->setCycleTime(1000.0);
        job->setCycleOffset(CMNode.Sensor.LidarRSI_Pandar40p.nCycleOffset);
        job->registerCallback(&CMNodeHelloCM::Pandar40pPointcloudFillMsg, this);
        scheduler_.addJob(job);
        std::cout << "--> Added job for Pandar40p" << std::endl;
      } else {
        // If a param doesn't exist for the Pandar40p no pointcloud
        std::cout << "--> Lidar not found or inactive, no params stored, set non-active" << std::endl;
        CMNode.Sensor.LidarRSI_Pandar40p.Active                   = 0;
      }
    
      std::cout << "> Setting up object detector for lidar" << std::endl;
      /* Find the Pandar40p related object detector sensor in the Vehicle Infofile */
      int Pandar40p_objsensor_index = -1;
      int objsensor_index_count = -1;
      for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
        sprintf(sbuf, "Sensor.%d.Active", idxS);
        int temp_active = iGetIntOpt(Inf_Vehicle, sbuf, 0);

        sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
        ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.Param.%d.Type", ref);
        str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        if (!strcmp(str, "Object") and temp_active) {
          /* If the Object sensor is found, get its index and exit loop */
          idxP = ref;
          std::cout << "--> Object Sensor found" << std::endl;

          objsensor_index_count += 1;
          
          sprintf(sbuf, "Sensor.Param.%d.Name", idxP);
          std::cout << "--> Searching for " << sbuf << std::endl;
          str = iGetStrOpt(Inf_Vehicle, sbuf, "");
          std::cout << "--> " << sbuf << " = " << str << std::endl;
          /* If the Lidar is a Pandar40p, we have found our sensor and can break. We also save the index for what number lidar sensor it is*/
          if (!strcmp(str, "Param_Lidar_Pandar40p")) {
            std::cout << "--> Found the Pandar40p object sensor as Object[i]=" << objsensor_index_count << std::endl;
            CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.index = objsensor_index_count;
            Pandar40p_objsensor_index = objsensor_index_count;
            break;
          }
        }
      }

      if (Pandar40p_objsensor_index != -1){
        std::cout << "--> Storing lidar object sensor params" << std::endl;
        sprintf(sbuf, "Sensor.%d.Active", idxS);
        CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.Active = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.%d.pos", idxS);
        CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.pos              = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.%d.rot", idxS);
        CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.rot              = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
        CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.UpdRate          = iGetIntOpt(Inf_Vehicle, sbuf, 10);
        sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
        CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.nCycleOffset     = iGetIntOpt(Inf_Vehicle, sbuf, 0);

        /* Pass LidarRSI_Pandar40p sensor parameters to job */
        q.setRPY(angles::from_degrees(CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.rot[0]), 
                 angles::from_degrees(CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.rot[1]), 
                 angles::from_degrees(CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.rot[2]));
        CMNode.TF.Lidar_Pandar40p_Obj_Detect.transform.rotation = tf2::toMsg(q);
        CMNode.TF.Lidar_Pandar40p_Obj_Detect.transform.translation.x = CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.pos[0];
        CMNode.TF.Lidar_Pandar40p_Obj_Detect.transform.translation.y = CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.pos[1];
        CMNode.TF.Lidar_Pandar40p_Obj_Detect.transform.translation.z = CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.pos[2];
        sprintf(sbuf, "Sensor.%d.name", idxS);
        CMNode.TF.Lidar_Pandar40p_Obj_Detect.child_frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "LIR00");
        sprintf(sbuf, "Sensor.%d.Mounting", idxS);
        CMNode.TF.Lidar_Pandar40p_Obj_Detect.header.frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "Fr1A");
        transforms.push_back(CMNode.TF.Lidar_Pandar40p_Obj_Detect);

        std::cout << "--> Stored params" << std::endl;
        std::cout << "--> Setting up job" << std::endl;

        typedef CMJob::RosPublisher<bristol_msgs::msg::ConeArrayWithCovariance> ConeDetectionPublisher;
        auto lidar_cd_job = std::make_shared<ConeDetectionPublisher>(nhp_, "Pandar40p/cone_detections");
        lidar_cd_job->setCycleTime(1000.0/CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.UpdRate);
        lidar_cd_job->setCycleOffset(CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.nCycleOffset);
        lidar_cd_job->registerCallback(&CMNodeHelloCM::objectlistFillLidarConeMsg, this);
        scheduler_.addJob(lidar_cd_job);
        std::cout << "--> Set up job" << std::endl;
        
      } else{
        std::cout << "--> Lidar not found or inactive, no params stored, set non-active" << std::endl;
        CMNode.Sensor.LidarRSI_Pandar40p.Object_Sensor.Active = 0;
      }

      


    } /* set up Pandar40p Lidar publisher + job */

    { /* set up ObjectList publisher + job */

      /* Find the first Object sensor in the Vehicle Infofile */
      idxP = -1;
      for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
        sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
        ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.Param.%d.Type", ref);
        str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        if (!strcmp(str, "Object")) {
          /* If the Object sensor is found, get its index and exit loop */
          idxP = ref;
          break;
        }
      }

      /* Store Object sensor parameters */
      if (idxP != -1) {
        sprintf(sbuf, "Sensor.%d.Active", idxS);
        CMNode.Sensor.Object.Active                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.%d.pos", idxS);
        CMNode.Sensor.Object.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.%d.rot", idxS);
        CMNode.Sensor.Object.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
        CMNode.Sensor.Object.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 30);
        sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
        CMNode.Sensor.Object.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);
      
        /* Pass Object sensor parameters to job */
        q.setRPY(angles::from_degrees(CMNode.Sensor.Object.rot[0]), 
                 angles::from_degrees(CMNode.Sensor.Object.rot[1]), 
                 angles::from_degrees(CMNode.Sensor.Object.rot[2]));
        CMNode.TF.ObjectList.transform.rotation = tf2::toMsg(q);
        CMNode.TF.ObjectList.transform.translation.x = CMNode.Sensor.Object.pos[0];
        CMNode.TF.ObjectList.transform.translation.y = CMNode.Sensor.Object.pos[1];
        CMNode.TF.ObjectList.transform.translation.z = CMNode.Sensor.Object.pos[2];
        sprintf(sbuf, "Sensor.%d.name", idxS);
        CMNode.TF.ObjectList.child_frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "OB00"); // Obj_F
        sprintf(sbuf, "Sensor.%d.Mounting", idxS);
        CMNode.TF.ObjectList.header.frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "Fr1A");
        transforms.push_back(CMNode.TF.ObjectList); 

        typedef CMJob::RosPublisher<visualization_msgs::msg::MarkerArray> ObjectList;
        auto job = std::make_shared<ObjectList>(nhp_, "ObjectList");
        job->setCycleTime(1000.0/CMNode.Sensor.Object.UpdRate);
        job->setCycleOffset(CMNode.Sensor.Object.nCycleOffset);
        job->registerCallback(&CMNodeHelloCM::objectlistFillMsg, this);
        scheduler_.addJob(job);

        // Repeat job for cone detections; ground truth and noisy
        typedef CMJob::RosPublisher<bristol_msgs::msg::ConeArrayWithCovariance> GTConeDetectionPublisher;
        auto gt_cd_job = std::make_shared<GTConeDetectionPublisher>(nhp_, "ground_truth/cone_detections");
        gt_cd_job->setCycleTime(1000.0/CMNode.Sensor.Object.UpdRate);
        gt_cd_job->setCycleOffset(CMNode.Sensor.Object.nCycleOffset);
        gt_cd_job->registerCallback(&CMNodeHelloCM::objectlistFillGTConeMsg, this);
        scheduler_.addJob(gt_cd_job);

        typedef CMJob::RosPublisher<bristol_msgs::msg::ConeArrayWithCovariance> ConeDetectionPublisher;
        auto noisy_cd_job = std::make_shared<ConeDetectionPublisher>(nhp_, "zed/cone_detections");
        noisy_cd_job->setCycleTime(1000.0/CMNode.Sensor.Object.UpdRate);
        noisy_cd_job->setCycleOffset(CMNode.Sensor.Object.nCycleOffset);
        noisy_cd_job->registerCallback(&CMNodeHelloCM::objectlistFillConeMsg, this);
        scheduler_.addJob(noisy_cd_job);
      } else {
        CMNode.Sensor.Object.Active = 0;
      }
    } // set up ObjectList publisher + job

    { /* set up Camera publisher + job */
      
      /* Find the first Camera sensor in the Vehicle Infofile */
      idxP = -1;
        for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
          sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
          ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
          sprintf(sbuf, "Sensor.Param.%d.Type", ref);
          str = iGetStrOpt(Inf_Vehicle, sbuf, "");
          if (!strcmp(str, "Camera")) {
            /* If the Camera sensor is found, get its index and exit loop */
            idxP = ref;
            break;
          }
        }

        /* Store Camera sensor parameters */
        if (idxP != -1) {
          sprintf(sbuf, "Sensor.%d.Active", idxS);
          CMNode.Sensor.Camera.Active                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);
          sprintf(sbuf, "Sensor.%d.pos", idxS);
          CMNode.Sensor.Camera.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
          sprintf(sbuf, "Sensor.%d.rot", idxS);
          CMNode.Sensor.Camera.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
          sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
          CMNode.Sensor.Camera.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 100);
          sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
          CMNode.Sensor.Camera.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);

          /* Pass Camera sensor parameters to job */
          q.setRPY(angles::from_degrees(CMNode.Sensor.Camera.rot[0]), 
                   angles::from_degrees(CMNode.Sensor.Camera.rot[1]), 
                   angles::from_degrees(CMNode.Sensor.Camera.rot[2]));
          CMNode.TF.Camera.transform.rotation = tf2::toMsg(q);
          CMNode.TF.Camera.transform.translation.x = CMNode.Sensor.Camera.pos[0];
          CMNode.TF.Camera.transform.translation.y = CMNode.Sensor.Camera.pos[1];
          CMNode.TF.Camera.transform.translation.z = CMNode.Sensor.Camera.pos[2];
          sprintf(sbuf, "Sensor.%d.name", idxS);
          CMNode.TF.Camera.child_frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "LIN00");
          sprintf(sbuf, "Sensor.%d.Mounting", idxS);
          CMNode.TF.Camera.header.frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "Fr1A");
          transforms.push_back(CMNode.TF.Camera);

          typedef CMJob::RosPublisher<camera_msgs::msg::CameraDetectionArray> Camera;
          auto job = std::make_shared<Camera>(nhp_, "Camera");
          job->setCycleTime(CMNode.Sensor.Camera.UpdRate);
          job->setCycleOffset(CMNode.Sensor.Camera.nCycleOffset);
          job->registerCallback(&CMNodeHelloCM::cameraFillMsg, this);
          scheduler_.addJob(job);
          
      } else {
        CMNode.Sensor.Camera.Active                   = 0;
      }
    } // set up Camera publisher + job

    { /* set up CameraRSI publishing nodes */

      int camera_no = 0;
      tCameraRSI Camera;

      /* Find each CameraRSI in the Vehicle Infofile */
      idxP = -1;
      for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
        sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
        ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.Param.%d.Type", ref);
        str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        if (!strcmp(str, "CameraRSI")) {
          /* If the Camera sensor is found, get its index and extract its parameters */
          idxP = ref;
          sprintf(sbuf, "Sensor.%d.Ref.Cluster", idxS);
          idxC = iGetIntOpt(Inf_Vehicle, sbuf, 0);

          sprintf(sbuf, "Sensor.%d.Active", idxS);
          int camera_active = iGetIntOpt(Inf_Vehicle, sbuf, 0);

          /* Only store and process active camera details */
          if (camera_active) {
            /* Store Camera sensor parameters */
            sprintf(sbuf, "Sensor.%d.name", idxS);
            str                             = iGetStrOpt(Inf_Vehicle, sbuf, "");
            Camera.name = strdup(str);
            strcpy(Camera.name, str);
            Camera.number = camera_no;
            sprintf(sbuf, "Sensor.%d.pos", idxS);
            Camera.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.%d.rot", idxS);
            Camera.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
            Camera.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 100);
            sprintf(sbuf, "Sensor.Param.%d.FoV", idxP);
            Camera.FoV                      = iGetDblOpt(Inf_Vehicle, sbuf, 60);
            sprintf(sbuf, "Sensor.Param.%d.Resolution", idxP);
            Camera.Resolution               = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def2c, 2, 1);
            sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
            Camera.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);
            sprintf(sbuf, "SensorCluster.%d.Socket", idxC);
            Camera.Socket                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);

            /* Do a system call to ros2 launch and launch the ROS node */
            sprintf(sbuf, "ros2 launch carmaker_rsds_client carmaker_rsds_client.launch.py \
            camera_no:=%d \
            camera_name:=%s \
            rsds_port:=%d \
            param_trans_rot:=[%f,%f,%f,%f,%f,%f] \
            fov_deg:=%f \
            width:=%f \
            height:=%f &",
            Camera.number,
            Camera.name,
            Camera.Socket,
            Camera.pos[0],Camera.pos[1],Camera.pos[2],Camera.rot[0],Camera.rot[1],Camera.rot[2],
            Camera.FoV,
            Camera.Resolution[0],
            Camera.Resolution[1]);
            system(sbuf);

            CMNode.Sensor.CameraRSI.push_back(Camera);

            camera_no++;
          } // if (camera_active)
        } // if (!strcmp(str, "CameraRSI"))
      } // for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++)
    } // set up CameraRSI publishing nodes

    { /* set up VehicleControl subscriber + job */
      std::string topic = "/AI2VCU/requests";
      bool synchronize = (sync_mode_ == SyncMode::kTopic);
      CMJob::JobType job_type =
          synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

      typedef CMJob::RosSubscriber<ads_dv_msgs::msg::AI2VCURequests> VC_ai;
      auto job = std::make_shared<VC_ai>(job_type, synchronize, nhp_, topic);
      job->setCycleTime(static_cast<unsigned long>(1));
      job->skipFirstCycles(1);
      job->registerCallback(&CMNodeHelloCM::vehicleAIControlCallback, this);
      scheduler_.addJob(job);
    } // set up VehicleControl subscriber + job

    { /* set up VehicleControl subscriber + job */
      std::string topic = "/carmaker/cmd";
      bool synchronize = (sync_mode_ == SyncMode::kTopic);
      CMJob::JobType job_type =
          synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

      typedef CMJob::RosSubscriber<ackermann_msgs::msg::AckermannDriveStamped> VC_manual;
      auto job = std::make_shared<VC_manual>(job_type, synchronize, nhp_, topic);
      job->setCycleTime(static_cast<unsigned long>(1));
      job->skipFirstCycles(1);
      job->registerCallback(&CMNodeHelloCM::vehicleManualControlCallback, this);
      scheduler_.addJob(job);
    } // set up VehicleControl subscriber + job

    // { /* set up ext2cm subscriber + job */
    //   std::string topic = "ext2cm";
    //   bool synchronize = (sync_mode_ == SyncMode::kTopic);
    //   CMJob::JobType job_type =
    //       synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

    //   auto cycle_time = param_client_->get_parameter("cycletime", 15000);

    //   typedef CMJob::RosSubscriber<hellocm_msgs::msg::Ext2CM> ext2cm_t;
    //   auto job = std::make_shared<ext2cm_t>(job_type, synchronize, nhp_, topic);
    //   job->setCycleTime(static_cast<unsigned long>(cycle_time));
    //   job->skipFirstCycles(1);
    //   job->setTimeoutTime(max_sync_time_);
    //   job->registerCallback(&CMNodeHelloCM::ext2cmCallback, this);
    //   scheduler_.addJob(job);

    //   if (cycle_time % clock_cycle_time_ != 0 ||
    //       (cycle_time < clock_cycle_time_ && clock_cycle_time_ > 0)) {
    //     node_mode_ = NodeMode::kDisabled;
    //     LogErrF(EC_Sim,
    //             "Ext. ROS node has an invalid cycle time! Expected multiple of "
    //             "%iums but got %ims",
    //             clock_cycle_time_, cycle_time);

    //     return -1;
    //   }
    // } // set up ext2cm subscriber + job */
  } // if (VehicleInfo_Err == 0)

  InfoDelete(Inf_Vehicle);

  CMNode.TF.st_br->sendTransform(transforms);

  RCLCPP_INFO(nhp_->get_logger(), "N: %i", Traffic_GetNObj());
  return 1;
}

void CMNodeHelloCM::exportVectorsToCSV(
  const std::vector<double>& col1, 
  const std::vector<double>& col2,
  const std::vector<double>& col3, 
  const std::vector<double>& col4, 
  const std::string& filename, 
  const std::string& headers) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        std::cerr << "If you provide an empty file with required name at this location, PID control outputs will be provided" << std::endl;
        return;
    }

    // Write headers
    file << headers << "\n";

    // Write data row by row
    size_t size = std::min({col1.size(), col2.size(), col3.size(), col4.size()});
    for (size_t i = 0; i < size; ++i) {
        file << col1[i] << "," << col2[i] << "," << col3[i] << "," << col4[i] << "\n";
    }

    file.close();
    std::cout << "Data successfully written to " << filename << std::endl;
}

int CMNodeHelloCM::userTestrunEnd() {

  /* Go through existing CameraRSI RSDS nodes and shut them down */
  if (!CMNode.Sensor.CameraRSI.empty()) {
    system("pkill -f carmaker_rsds_client_node > /dev/null 2>&1");
    CMNode.Sensor.CameraRSI.clear();
  }

  CMNodeHelloCM::exportVectorsToCSV(CMNode.Misc.pid_data.current_speed, CMNode.Misc.pid_data.desired_speed, CMNode.Misc.pid_data.gas_input, CMNode.Misc.pid_data.brake_input, "/home/uweai-ssd/Documents/carmaker_pid.csv", "Current,Desired,Gas,Brake");

  scheduler_.deleteJob("pointcloud");
  scheduler_.deleteJob("PS150/pointcloud");
  scheduler_.deleteJob("Pandar40p/pointcloud");
  scheduler_.deleteJob("Pandar40P/cone_detections");
  scheduler_.deleteJob("ObjectList");
  scheduler_.deleteJob("ground_truth/cone_detections");
  scheduler_.deleteJob("cone_detections");
  scheduler_.deleteJob("ground_truth/pcan_gps");
  scheduler_.deleteJob("/VCU2AI/wheel_speeds");
  scheduler_.deleteJob("/VCU2AI/wheel_speeds_noisy");
  scheduler_.deleteJob("/IMU_Sensor");
  scheduler_.deleteJob("/VCU2AI/as_state");
  scheduler_.deleteJob("ground_truth/lap_stats");
  scheduler_.deleteJob("ground_truth/pose");
  scheduler_.deleteJob("ground_truth/map");
  scheduler_.deleteJob("ground_truth/rviz_map");
  scheduler_.deleteJob("pcan_gps");
  scheduler_.deleteJob("Camera");
  scheduler_.deleteJob("/AI2VCU/requests");
  scheduler_.deleteJob("/carmaker/cmd");
  // scheduler_.deleteJob("ext2cm");
  scheduler_.deleteJob("cm2ext");
  scheduler_.deleteJob("cm2extHelloWorld");
  // scheduler_.deleteJob("/carmaker/start_signal"); # Uncommented as bugfix. This is a service so should be allowed to remain
  
  
  return 1;
}

// Important: Use this macro to register the derived class as an interface with
// the CarMaker C++ Interface Loader module
REGISTER_CARMAKER_CPP_IF(CMNodeHelloCM)
