#ifndef OROCOS_CARTESIAN_POSITION_CONTROLLER_COMPONENT_HPP
#define OROCOS_CARTESIAN_POSITION_CONTROLLER_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <lwr_fri/typekit/Types.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <rtt/os/TimeService.hpp>

#include <Eigen/Geometry>

using Eigen::Vector3f;
using Eigen::Affine3f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

using namespace RTT;

class CartesianPositionController : public RTT::TaskContext{
 
  //Input Port
  InputPort<geometry_msgs::Pose> port_cart_pos_;
  geometry_msgs::Pose cart_pos_;

  //Output Ports
  OutputPort<geometry_msgs::Wrench> port_cart_wrench_cmd_;
  geometry_msgs::Wrench cart_wrench_cmd_;
  OutputPort<lwr_fri::CartesianImpedance> port_cart_imp_cmd_;
  lwr_fri::CartesianImpedance cart_imp_cmd_;
  OutputPort<geometry_msgs::Pose> port_cart_pos_cmd_;
  geometry_msgs::Pose cart_pos_cmd_;

  //Orocos Properties
  float t_out_;

  //Other
  std::vector<float> pos_des_;
  std::vector<float> cart_vel_;
  std::vector<float> last_pos_;
  RTT::os::TimeService::ticks t_start_;
  RTT::os::TimeService::Seconds t_cur_;
  RTT::os::TimeService::Seconds t_last_;
  RTT::os::TimeService::Seconds t_disp_;
  // logging
  RTT::os::TimeService::Seconds t_log_;

  
  public:
    CartesianPositionController(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
