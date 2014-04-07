#include "CartesianPositionController.hpp"
#include <rtt/Component.hpp>
#include <iostream>

CartesianPositionController::CartesianPositionController(std::string const& name) : TaskContext(name), t_out_(1),   t_disp_(0), t_last_(0){
  //Setup data ports
  this->addPort("CartesianPosition",port_cart_pos_);

  this->addPort("CartesianWrenchCommand", port_cart_wrench_cmd_);
  this->addPort("CartesianImpedanceCommand", port_cart_imp_cmd_);
  this->addPort("CartesianPositionCommand",port_cart_pos_cmd_);

  //Setup properties
  this->addProperty("T_out",t_out_);

}

bool CartesianPositionController::configureHook(){
  return true;
}

bool CartesianPositionController::startHook(){

  while(port_cart_pos_.read(cart_pos_)!=NewData) {
	std::cout << "Waiting for data from KUKA" << std::endl;
	usleep(1e6);
  }
  //Set Cartesian Impedance
  cart_imp_cmd_.stiffness.linear.x = 5000;
  cart_imp_cmd_.stiffness.linear.y = 5000;
  cart_imp_cmd_.stiffness.linear.z = 5000;
  cart_imp_cmd_.stiffness.angular.x = 50;
  cart_imp_cmd_.stiffness.angular.y = 50;
  cart_imp_cmd_.stiffness.angular.z = 50;
  cart_imp_cmd_.damping.linear.x = 1;
  cart_imp_cmd_.damping.linear.y = 1;
  cart_imp_cmd_.damping.linear.z = 1;
  cart_imp_cmd_.damping.angular.x = 1;
  cart_imp_cmd_.damping.angular.y = 1;
  cart_imp_cmd_.damping.angular.z = 1;
  port_cart_imp_cmd_.write(cart_imp_cmd_);

  //Initialize outputs
  cart_wrench_cmd_.force.x = 0;
  cart_wrench_cmd_.force.y = 0;
  cart_wrench_cmd_.force.z = 0;
  cart_wrench_cmd_.torque.x = 0;
  cart_wrench_cmd_.torque.y = 0;
  cart_wrench_cmd_.torque.z = 0;
  
  //Assume zero initial velocity
  for (size_t ii(0); ii < 3; ++ii) {
    cart_vel_.push_back(0);
  }
  
  //Time setup
  t_start_ = RTT::os::TimeService::Instance()->getTicks();
  
  port_cart_imp_cmd_.write(cart_imp_cmd_);
  port_cart_wrench_cmd_.write(cart_wrench_cmd_);
  port_cart_pos_cmd_.write(cart_pos_);

  return true;
}

void CartesianPositionController::updateHook(){

  //Read data
  t_cur_ = RTT::os::TimeService::Instance()->getSeconds(t_start_);
  if(port_cart_pos_.read(cart_pos_) == NewData) {
   
    //Initialize from first data in this test
    if (pos_des_.size() == 0) {
      //Set desired position to the current
      pos_des_.push_back(cart_pos_.position.x);
      pos_des_.push_back(cart_pos_.position.y);
      pos_des_.push_back(cart_pos_.position.z);
      cart_pos_cmd_ = cart_pos_;

      //Set parameters for caluclating velocity
      last_pos_.push_back(cart_pos_.position.x);
      last_pos_.push_back(cart_pos_.position.y);
      last_pos_.push_back(cart_pos_.position.z);
    }

    //Calculate velocity
    cart_vel_[0] = (cart_pos_.position.x - last_pos_[0])/(t_cur_ - t_last_);
    cart_vel_[1] = (cart_pos_.position.y - last_pos_[1])/(t_cur_ - t_last_);
    cart_vel_[2] = (cart_pos_.position.z - last_pos_[2])/(t_cur_ - t_last_);
    
    cart_wrench_cmd_.force.x = 0;
    cart_wrench_cmd_.force.y = 0;
    cart_wrench_cmd_.force.z = 0;
    cart_wrench_cmd_.torque.x = 0;
    cart_wrench_cmd_.torque.y = 0;
    cart_wrench_cmd_.torque.z = 0;
    
    //Update for next velocity calculation
    last_pos_[0] = cart_pos_.position.x;
    last_pos_[1] = cart_pos_.position.y;
    last_pos_[2] = cart_pos_.position.z;
    t_last_ = t_cur_;
  }
  
  //Write Desired Position
  port_cart_pos_cmd_.write(cart_pos_cmd_);
    
  //Write Impedance
  port_cart_imp_cmd_.write(cart_imp_cmd_);
  
  //Write command
  port_cart_wrench_cmd_.write(cart_wrench_cmd_);
  
  //Display data
  if (t_cur_ - t_disp_ > t_out_) {
    std::cout << "Time: " << t_cur_<< std::endl;
    std::cout << "Cartesian Position: " << cart_pos_.position.x << " " << cart_pos_.position.y << " " << cart_pos_.position.z << std::endl;
    std::cout << "\n";
    
    t_disp_ = t_cur_;
  } 

}

void CartesianPositionController::stopHook() {

}

void CartesianPositionController::cleanupHook() {
 
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(CartesianPositionController)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(CartesianPositionController)
