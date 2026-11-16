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
#include "bot_navigation/patrol_step.hpp"

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
    progress_=2*length_*phase_;
    start_=node_->create_subscription<std_msgs::msg::Bool>(
      "/competition/dense_motion_started",rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr m){active_.store(m->data);});
    guard_=node_->create_publisher<std_msgs::msg::String>("/competition/dense_obstacles/guards",10);
    connection_=gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&DensePatrol::update,this));
  }
private:
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
    std::vector<PatrolBlocker> blockers;
    if(robot_) {
      const auto p=robot_->WorldPose().Pos();
      blockers.push_back({{p.X(),p.Y()},.34+radius_+.10});
    }
    for(const auto & cube:cubes_) {
      const auto p=cube->WorldPose().Pos();
      if(p.Z()>.01)blockers.push_back({{p.X(),p.Y()},radius_+.10});
    }
    const PatrolPoint a{a_.X(),a_.Y()},b{b_.X(),b_.Y()};
    const auto step=active_.load() ? guardedPatrolStep(a,b,progress_,direction_,
      speed_*dt,blockers):PatrolStep{progress_,direction_,false,false};
    const bool hold=step.blocked;
    if(hold && !paused_)++holds_;
    if(step.reversed)++reversals_;
    paused_=hold;
    progress_=step.phase;direction_=step.direction;
    const auto p=patrolPoint(a,b,progress_);
    if(step.reversed || hold)guard_active_=true;
    bool clear=true;
    for(const auto & blocker:blockers)if(std::hypot(p.x-blocker.point.x,
      p.y-blocker.point.y)<blocker.clearance+.12)clear=false;
    if(clear)guard_active_=false;
    model_->SetWorldPose(ignition::math::Pose3d(p.x,p.y,height_/2,0,0,0));
    if(now-last_status_>.5) {
      last_status_=now;std::ostringstream s;
      s<<"{\"model\":\""<<model_->GetName()<<"\",\"stamp_sec\":"<<now
       <<",\"holds\":"<<holds_<<",\"active\":"<<(guard_active_?"true":"false")
       <<",\"reversals\":"<<reversals_<<",\"motion_state\":\""
       <<(hold?"physically_blocked":guard_active_?"collision_backoff":"patrol")<<"\"}";
      std_msgs::msg::String message;message.data=s.str();guard_->publish(message);
    }
  }
  gazebo::physics::ModelPtr model_,robot_;gazebo::physics::WorldPtr world_;
  std::vector<gazebo::physics::ModelPtr> cubes_;gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr connection_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr guard_;
  ignition::math::Vector2d a_,b_;
  std::atomic<bool> active_{false};bool paused_{false},guard_active_{false};
  unsigned holds_{0},reversals_{0};int direction_{1};
  double progress_{0},last_{-1},last_status_{0},length_,speed_,phase_,height_,radius_;
};
GZ_REGISTER_MODEL_PLUGIN(DensePatrol)
}
