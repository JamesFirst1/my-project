/*!
 ******************************************************************************
 **  Example for a CarMaker ROS Node that communicates with an external node
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 */

#ifndef CMNODE_ROS2_HELLOCM_HPP
#define CMNODE_ROS2_HELLOCM_HPP

/* CarMaker
 * - include other headers e.g. to access to vehicle data
 *   - e.g. "Vehicle.h" or "Vehicle/Sensor_*.h".
 * - additional headers can be found in "<CMInstallDir>/include/"
 * - see Reference Manual, chapter "User Accessible Quantities" to find some variables
 *   that are already defined in DataDictionary and their corresponding C-Code Name
 */
#include "Log.h"
#include "DataDict.h"
#include "SimCore.h"
#include "InfoUtils.h"


#include "apo.h"
#include "GuiCmd.h"

//CarMaker Header File Includes
#include "Vehicle.h"
#include "Vehicle/Sensor_LidarRSI.h"
#include "Vehicle/Sensor_Object.h"
#include "Vehicle/Sensor_Camera.h"
#include "Vehicle/Sensor_Inertial.h"
#include "Vehicle/Sensor_Collision.h"
#include "infoc.h"
#include <math.h>
#include "Car/Car.h"
#include "Car/Brake.h"
#include "DrivMan.h"
#include "VehicleControl.h"
#include "Traffic.h"
#include "Car/Steering.h"

/* ROS */
#include "cmrosutils/cmrosif.hpp"                   /* Only for CarMaker ROS Node!!! Functions are located in library for CarMaker ROS Interface */
#include "cmrosutils/srv/cm_remote_control.hpp"     /* Basic service for CarMaker remote from ROS */
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <angles/angles.h>
#include "vehiclecontrol_msgs/msg/vehicle_control.hpp"
#include "camera_msgs/msg/camera_detection_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

/* UWE ROS */
#include "bristol_msgs/msg/cone_array_with_covariance.hpp"
#include "ads_dv_msgs/msg/wheel_speeds.hpp"
#include "ads_dv_msgs/msg/ai2_vcu_requests.hpp"
#include "ads_dv_msgs/msg/autonomous_state.hpp"
#include "sim_msgs/msg/lap_stats.hpp"
#include "sim_msgs/srv/driver_mode.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"


/* UWE CPP */
#include "pid_controller.hpp"

/* UWE Other */
#include <list>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <string>

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
using namespace std;

/* Following header from external ROS node can be used to get topic/service/... names
 * Other mechanism:
 * 1. Put names manually independently for each node
 * 2. Using command line arguments or launch files and ROS remapping
 * - Doing so, only general message headers are necessary
 */
#if 0
//#  include "hellocm/ROS1_HelloCM.h"  /* External ROS Node. Topic name, ... */
#else
  #include "hellocm_msgs/msg/cm2_ext.hpp"
  #include "hellocm_msgs/msg/ext2_cm.hpp"
  #include "hellocm_msgs/srv/init.hpp"
#endif

namespace cm_ros {
/**
 * @brief The CMNodeHelloCM class serves as an example for a CarMaker ROS Node
 * that communicates with an external node. It derives from the
 * CarMakerROSInterface base class, which already implements the basic CarMaker
 * ROS node functioniality.
 */
class CMNodeHelloCM : public CarMakerROSInterface {
 public:
  CMNodeHelloCM();

  double gaussianKernel(double mu, double sigma);

  void exportVectorsToCSV(
  const std::vector<double>& col1, 
  const std::vector<double>& col2,
  const std::vector<double>& col3, 
  const std::vector<double>& col4, 
  const std::string& filename, 
  const std::string& headers);

  
  // helper function for PIDs
  void pidInit();

 private:
  /**
   * @brief userInit sets up the ros publisher job and the service client
   * @return 1
   */
  int userInit() final;

  /**
   * @brief userDeclQuants declares some User Accessible Quantities (UAQs) for
   * data storage in ERG files, data access via e.g. DVA or visualization in
   * e.g. IPGControl
   */
  void userDeclQuants() final;

  /**
   * @brief userTestrunStartAtBegin first calls the service of the external node
   * to resets it. Then it sets up the ros subscriber job. In case of
   * synchronized mode the job uses the cycle time of the external node
   * retrieved via ros parameter server and checks whether it is compatible with
   * the current clock cycle time.
   * @return 1 if successful, -1 if otherwise
   */
  int userTestrunStartAtBegin() final;

  /**
   * @brief userTestrunEnd deletes the subscriber job
   * @return 1
   */
  int userTestrunEnd() final;

  /**
   * @brief userVehicleControlCalc called in realtime context, after vehicle
   * control calculation
   * @param dt the simulation time step
   * @return < 0 if errors occur, >= 0 otherwise
   */
  int userVehicleControlCalc(const double& dt);

  /**
   * @brief ext2cmCallback Callback function for ext2cm subscriber.
   * Used in this example to synchronize the CM cycle to
   * @param msg Received ROS message
   */
  void ext2cmCallback(hellocm_msgs::msg::Ext2CM::ConstSharedPtr msg);

  /** 
   * @brief vehicleControlCallback Vehicle Control subcriber callback when receiving a VC message.
   * @param msg Received ROS Vehicle Control message.
   */
  void vehicleAIControlCallback(ads_dv_msgs::msg::AI2VCURequests::ConstSharedPtr msg);
  void vehicleManualControlCallback(ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg);
  PID steer_pid;
  PID brake_pid;
  PID gas_pid;
  // Previous timestamp for delta time calculation
  rclcpp::Time previous_pid_time;

  /** 
   * @brief autonomousStateFillMsg Prepares the msg describing the current AS State
   * @param msg Received UWE-ROS AutonomousState message that this function prepares for publishing.
   */
  void autonomousStateFillMsg(ads_dv_msgs::msg::AutonomousState& msg);

  /**
   * @brief cm2extFillMsg prepares the message to be sent to the external node.
   * Demonstrates how CarMaker variables can be sent out as ros messages.
   * @param msg the actual message being prepared by this function
   */
  void cm2extFillMsg(hellocm_msgs::msg::CM2Ext& msg);

   /**
   * @brief **TEST**
   * @param msg 
   */
  void cm2extHelloWorldFillMsg(std_msgs::msg::String& msg);

  /**
   * @brief pointcloudFillMsg prepares the message of LidarRSI pointclouid data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being prepared by this function
   */
  void pointcloudFillMsg(sensor_msgs::msg::PointCloud& msg);

  void PS150pointcloudFillMsg(sensor_msgs::msg::PointCloud& msg);

  void Pandar40pPointcloudFillMsg(sensor_msgs::msg::PointCloud& msg);
  

  /**
   * @brief objectlistFillMsg prepares the message of Object sensor ObjectList data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being prepared by this function
   */
  void objectlistFillMsg(visualization_msgs::msg::MarkerArray& msg);

  void collisionFillMsg(std_msgs::msg::Bool& msg);

  void objectlistFillGTConeMsg(bristol_msgs::msg::ConeArrayWithCovariance& msg);

  void objectlistFillConeMsg(bristol_msgs::msg::ConeArrayWithCovariance& msg);
  void objectlistFillLidarConeMsg(bristol_msgs::msg::ConeArrayWithCovariance& msg);  

  void mapFillGTMsg(bristol_msgs::msg::ConeArrayWithCovariance& msg);

  void mapFillGTMsg2(visualization_msgs::msg::MarkerArray& msg);

  void pcan_gps_gt_fill_msg(sensor_msgs::msg::NavSatFix& msg);

  void pcan_gps_fill_msg(sensor_msgs::msg::NavSatFix& msg);
  
  void ws_fill_msg(ads_dv_msgs::msg::WheelSpeeds& msg);

  void wheel_speeds_noisy(ads_dv_msgs::msg::WheelSpeeds& msg);
  
  void IMU_reading(sensor_msgs::msg::Imu& msg);

  void lap_stat_fill_msg(sim_msgs::msg::LapStats& msg);
  vector<float_t> lap_history;
  int current_lap = 0;

  void pose_gt_fill_msg(geometry_msgs::msg::PoseWithCovarianceStamped& msg);
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  
  // For random seeding on cone noise
  unsigned int seed;
  
  /**
   * @brief objectlistFillMsg prepares the message of Camera data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being prepared by this function
   */
  void cameraFillMsg(camera_msgs::msg::CameraDetectionArray& msg);

  /**
   * @brief init_ service client to demonstrate ros service calls to an
   * external node. In this example the service is called at each TestRun start
   * resetting the external node.
   */
  rclcpp::Client<hellocm_msgs::srv::Init>::SharedPtr srv_init_;

  /**
   * @brief param_client_ synchronous parameter client to retrieve parameters
   * from external node.
   */
  rclcpp::SyncParametersClient::SharedPtr param_client_;

  /**
   * @brief synth_delay_ Synthetic delay in seconds to artificially delay the
   * external node to showcase synchronization mode
   */
  double synth_delay_;

  /**
   * @brief current_driver_mode_ Current driver mode of choice {IPG, AI, Manual}
   * for the IPG sim to use.
   */
  rclcpp::Service<sim_msgs::srv::DriverMode>::SharedPtr driver_mode_srv_;
  std::string current_driver_mode_;

  /**
   * @brief Allow sending start/reset signal from simulator to other software.
   * Primarily made to reset RQT GUI on restart
   */
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_publisher_;
  void publishResetSignal();

  void handle_driver_request(
        const std::shared_ptr<sim_msgs::srv::DriverMode::Request> request,
        std::shared_ptr<sim_msgs::srv::DriverMode::Response> response);

};
}  // namespace cm_ros

#endif  // CMNODE_ROS2_HELLOCM_HPP
