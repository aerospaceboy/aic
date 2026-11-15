// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
// Run kinematic patrols on Gazebo's update thread, without a high-rate ROS
// SetEntityState service queue. Static geometry is audited before deployment.
#include <atomic>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace bot_navigation
{
class DensePatrol : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model,sdf::ElementPtr sdf)override {
    model_=model;world_=model->GetWorld();node_=gazebo_ros::Node::Get(sdf);
    a_={sdf->Get<double>("start_x"),sdf->Get<double>("start_y")};
    b_={sdf->Get<double>("end_x"),sdf->Get<double>("end_y")};
    speed_=sdf->Get<double>("speed");phase_=sdf->Get<double>("phase_fraction");
    length_=(b_-a_).Length();height_=sdf->Get<double>("height");radius_=sdf->Get<double>("radius");
    if(length_<.1 || speed_<=0 || speed_>.65)throw std::runtime_error("invalid dense patrol");
    start_=node_->create_subscription<std_msgs::msg::Bool>(
      "/competition/dense_motion_started",rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr m){active_.store(m->data);});
    guard_=node_->create_publisher<std_msgs::msg::String>("/competition/dense_obstacles/guards",10);
    connection_=gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&DensePatrol::update,this));
  }
private:
  ignition::math::Vector2d point(double elapsed)const {
    double distance=std::fmod(elapsed*speed_+2*length_*phase_,2*length_);
    return a_+(b_-a_)*(std::min(distance,2*length_-distance)/length_);
  }
  void update() {
    const double now=world_->SimTime().Double();
    if(last_<0){last_=now;return;}
    if(now-last_<.049)return;
    const double dt=std::max(0.,std::min(.10,now-last_));last_=now;
    if(!robot_)robot_=world_->ModelByName("six_arm");
    if(cubes_.empty())for(auto color:{"red","blue"})for(int i=1;i<=5;++i) {
      auto model=world_->ModelByName(std::string(color)+"_cube_"+std::to_string(i));
      if(model)cubes_.push_back(model);
    }
    const auto candidate=point(elapsed_+(active_.load()?dt:0.));
    bool hold=false;
    if(robot_) {
      const auto p=robot_->WorldPose().Pos();
      hold=std::hypot(candidate.X()-p.X(),candidate.Y()-p.Y())<.45;
    }
    for(const auto & cube:cubes_) {
      const auto p=cube->WorldPose().Pos();
      if(p.Z()>.04 && std::hypot(candidate.X()-p.X(),candidate.Y()-p.Y())<radius_+.10)hold=true;
    }
    if(hold && !paused_)++holds_;
    paused_=hold;
    if(active_.load() && !hold)elapsed_+=dt;
    const auto p=point(elapsed_);
    model_->SetWorldPose(ignition::math::Pose3d(p.X(),p.Y(),height_/2,0,0,0));
    if(now-last_status_>.5) {
      last_status_=now;std::ostringstream s;
      s<<"{\"model\":\""<<model_->GetName()<<"\",\"stamp_sec\":"<<now
       <<",\"holds\":"<<holds_<<",\"active\":"<<(hold?"true":"false")<<"}";
      std_msgs::msg::String message;message.data=s.str();guard_->publish(message);
    }
  }
  gazebo::physics::ModelPtr model_,robot_;gazebo::physics::WorldPtr world_;
  std::vector<gazebo::physics::ModelPtr> cubes_;gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr connection_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr guard_;
  ignition::math::Vector2d a_,b_;
  std::atomic<bool> active_{false};bool paused_{false};unsigned holds_{0};
  double elapsed_{0},last_{-1},last_status_{0},length_,speed_,phase_,height_,radius_;
};
GZ_REGISTER_MODEL_PLUGIN(DensePatrol)
}
