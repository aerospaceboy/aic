/*
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2023.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) IFRA Gazebo-ROS2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Light.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <gazebo_ros/node.hpp>
#include <memory>
#include <mutex>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"

#include "ros2_linkattacher/gazebo_link_attacher.hpp"   // INCLUDE HADER FILE.
#include <linkattacher_msgs/srv/attach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include <linkattacher_msgs/srv/detach_link.hpp>        // INCLUDE ROS2 SERVICE.

// GLOBAL VARIABLE:
std::vector<JointSTRUCT> GV_joints;

namespace gazebo_ros
{

class GazeboLinkAttacherPrivate
{
public:

  // ATTACH (ROS2 service):
  void Attach(
    linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
    linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res);

  // DETACH (ROS2 service):
  void Detach(
    linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
    linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res);

  // World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  // ROS services to handle requests for attach/detach.
  rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_service_;
  rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_link_service_;

  // getJoint function:
  bool getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint);

  // Gazebo services may be dispatched concurrently by the ROS executor.
  std::mutex joints_mutex_;

};

GazeboLinkAttacher::GazeboLinkAttacher()
: impl_(std::make_unique<GazeboLinkAttacherPrivate>())
{
}

GazeboLinkAttacher::~GazeboLinkAttacher()
{
}

void GazeboLinkAttacher::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  
  // Gazebo WORLD:
  impl_->world_ = _world;

  // ROS2 NODE:
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // ROS2 SERVICE SERVERS:
  impl_->attach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::AttachLink>(
    "ATTACHLINK", std::bind(
      &GazeboLinkAttacherPrivate::Attach, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));
  impl_->detach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::DetachLink>(
    "DETACHLINK", std::bind(
      &GazeboLinkAttacherPrivate::Detach, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

}

void GazeboLinkAttacherPrivate::Attach(
  linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res)
{
  std::lock_guard<std::mutex> lock(joints_mutex_);

  // CHECK if -> Joint already exists in GV_joints:
  /* THIS IS NO LONGER NEEDED, SINCE THE JOINT IS REMOVED AFTER DETACHING!
  JointSTRUCT j;
  if (this->getJoint(_req->model1_name, _req->link1_name, _req->model2_name, _req->link2_name, j)){
    j.joint->Attach(j.l1, j.l2);
    _res->success = true;
    _res->message = "ATTACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
    return;
  }
  */

  // Get the first link:
  gazebo::physics::ModelPtr model1 = world_->ModelByName(_req->model1_name);
  if (!model1) {
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model1_name;
    return;
  }
  gazebo::physics::LinkPtr link1 = model1->GetLink(_req->link1_name);
  if (!link1) {
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link1_name;
    return;
  }

  // Get the second link:
  gazebo::physics::ModelPtr model2 = world_->ModelByName(_req->model2_name);
  if (!model2) {
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model2_name;
    return;
  }
  gazebo::physics::LinkPtr link2 = model2->GetLink(_req->link2_name);
  if (!link2) {
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link2_name;
    return;
  }

  // A link may participate in only one active attachment, but unrelated link
  // pairs can be attached at the same time (for example a stabilized mobile
  // base and a cube held by the gripper).
  for (const auto & existing : GV_joints) {
    const bool first_link_is_busy =
      (existing.model1 == _req->model1_name &&
      existing.link1 == _req->link1_name) ||
      (existing.model2 == _req->model1_name &&
      existing.link2 == _req->link1_name);
    const bool second_link_is_busy =
      (existing.model1 == _req->model2_name &&
      existing.link1 == _req->link2_name) ||
      (existing.model2 == _req->model2_name &&
      existing.link2 == _req->link2_name);
    if (first_link_is_busy || second_link_is_busy) {
      _res->success = false;
      _res->message =
        "At least one requested link is already attached; "
        "aborting new attachment.";
      return;
    }
  }

  // Create a true fixed joint. A zero-range revolute joint can inject large
  // constraint forces when the attached links are moved by a controller.
  const std::string joint_name =
    _req->model1_name + "_" + _req->link1_name + "_" +
    _req->model2_name + "_" + _req->link2_name + "_joint";
  gazebo::physics::JointPtr joint =
    model1->CreateJoint(joint_name, "fixed", link1, link2);
  if (!joint) {
    _res->success = false;
    _res->message = "Failed to create fixed attachment joint: " + joint_name;
    return;
  }

  // Model::CreateJoint already attaches and loads the joint. Repeating those
  // operations corrupts the dynamic joint lifecycle in Gazebo Classic.
  joint->SetProvideFeedback(true);
  joint->Init();
  model1->Update();

  JointSTRUCT joint_record;
  joint_record.joint_name = joint_name;
  joint_record.model1 = _req->model1_name;
  joint_record.model2 = _req->model2_name;
  joint_record.link1 = _req->link1_name;
  joint_record.link2 = _req->link2_name;
  joint_record.m1 = model1;
  joint_record.m2 = model2;
  joint_record.l1 = link1;
  joint_record.l2 = link2;
  joint_record.joint = joint;

  GV_joints.push_back(joint_record);

  // Set the success and message in the response:
  _res->success = true;
  _res->message =
    "ATTACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " +
    _req->link1_name + "} -- {" + _req->model2_name + " , " +
    _req->link2_name + "}.";
}

void GazeboLinkAttacherPrivate::Detach(
  linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res)
{
  std::lock_guard<std::mutex> lock(joints_mutex_);

  for (auto it = GV_joints.begin(); it != GV_joints.end(); ++it) {
    if (
      it->model1 == _req->model1_name &&
      it->link1 == _req->link1_name &&
      it->model2 == _req->model2_name &&
      it->link2 == _req->link2_name)
    {
      gazebo::physics::ModelPtr model1 = world_->ModelByName(_req->model1_name);
      if (!model1 || !model1->RemoveJoint(it->joint_name)) {
        _res->success = false;
        _res->message =
          "Failed to remove attachment joint: " + it->joint_name;
        return;
      }

      // Model::RemoveJoint performs Detach and Fini exactly once. Erase our
      // owning record only after Gazebo confirms that removal succeeded.
      GV_joints.erase(it);

      _res->success = true;
      _res->message =
        "DETACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " +
        _req->link1_name + "} -- {" + _req->model2_name + " , " +
        _req->link2_name + "}.";
      return;
    }
  }

  _res->success = false;
  _res->message =
    "DETACHED -- ERROR (Joint does not exist!): {MODEL , LINK} -> {" +
    _req->model1_name + " , " + _req->link1_name + "} -- {" +
    _req->model2_name + " , " + _req->link2_name + "}.";
}

bool GazeboLinkAttacherPrivate::getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint)
  {
    JointSTRUCT j;
    for(std::vector<JointSTRUCT>::iterator it = GV_joints.begin(); it != GV_joints.end(); ++it){
      j = *it;
      if ((j.model1.compare(M1) == 0) && (j.model2.compare(M2) == 0) && (j.link1.compare(L1) == 0) && (j.link2.compare(L2) == 0)){
        joint = j;
        return true;
      }
    }
    return false;
  }

GZ_REGISTER_WORLD_PLUGIN(GazeboLinkAttacher)

}  // namespace gazebo_ros
