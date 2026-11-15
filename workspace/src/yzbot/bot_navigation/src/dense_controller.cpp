// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
// Nav2 Humble MPPI integration. Sensor forecasts are time indexed; simulator
// actor state is used only by the independent downstream safety monitor.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_mppi_controller/critic_function.hpp>
#include <nav2_util/node_utils.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include "bot_navigation/remaining_path.hpp"

namespace bot_navigation
{
struct ForecastPoint {double t,x,y,error;};
struct SensorTrack {
  int id; double stamp,x,y,vx,vy,radius,uncertainty; bool moving,confident;
  std::vector<ForecastPoint> forecast;
};

class SensorTracks
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
  {
    node_=node;costmap_=costmap;
    sub_=node->create_subscription<std_msgs::msg::String>(
      "/competition/dynamic_tracks",rclcpp::QoS(2),
      [this](const std_msgs::msg::String::SharedPtr message) {
        try {
          auto j=nlohmann::json::parse(message->data);
          if(j.value("source","")!="laser_stamped_map_tf")return;
          const std::string source=j.value("frame_id", "map");
          double tx=0,ty=0,yaw=0;
          const auto target=costmap_->getGlobalFrameID();
          if(source!=target) {
            const auto tf=costmap_->getTfBuffer()->lookupTransform(target,source,tf2::TimePointZero);
            tx=tf.transform.translation.x;ty=tf.transform.translation.y;
            yaw=tf2::getYaw(tf.transform.rotation);
          }
          const double c=std::cos(yaw),s=std::sin(yaw);
          std::vector<SensorTrack> tracks;
          for(const auto & v:j.at("tracks")) {
            SensorTrack a;
            a.id=v.at("id");a.stamp=v.at("stamp");
            const double x=v.at("x"),y=v.at("y"),vx=v.at("vx"),vy=v.at("vy");
            a.x=tx+c*x-s*y;a.y=ty+s*x+c*y;a.vx=c*vx-s*vy;a.vy=s*vx+c*vy;
            a.radius=v.value("radius",.18);a.uncertainty=v.value("uncertainty",.05);
            a.confident=v.value("confident",false);a.moving=v.value("was_moving",false);
            if(!std::isfinite(a.x+a.y+a.vx+a.vy+a.stamp+a.radius) ||
              a.radius<=0 || a.radius>.4 || std::hypot(a.vx,a.vy)>.8)continue;
            if(v.contains("forecast"))for(const auto & f:v["forecast"]) {
              double fx=f[1],fy=f[2];
              a.forecast.push_back({f[0],tx+c*fx-s*fy,ty+s*fx+c*fy,f[3]});
            }
            tracks.push_back(a);
          }
          std::lock_guard<std::mutex> lock(mutex_);tracks_=std::move(tracks);
          received_=std::chrono::steady_clock::now();
        }catch(const std::exception &){/* Raw scans and the final guard remain active. */}
      });
    mission_=node->create_subscription<std_msgs::msg::String>(
      "/competition/mission/state",rclcpp::QoS(2),[this](const std_msgs::msg::String::SharedPtr m){
        try{auto j=nlohmann::json::parse(m->data);auto s=j.value("state","");
          std::lock_guard<std::mutex> lock(mutex_);
          loaded_=(s=="navigating_departure" || s=="navigating_destination" || s=="placing");
        }catch(const std::exception &){}
      });
  }
  std::vector<SensorTrack> snapshot(double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SensorTrack> result;
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count()>.7)return result;
    for(const auto & a:tracks_)if(now-a.stamp>=-.05 && now-a.stamp<1.0 &&
      (a.moving || std::hypot(a.vx,a.vy)>.04))result.push_back(a);
    return result;
  }
  bool loaded(){std::lock_guard<std::mutex> lock(mutex_);return loaded_;}
  static ForecastPoint at(const SensorTrack & a,double elapsed) {
    if(a.confident && !a.forecast.empty()) {
      for(size_t i=1;i<a.forecast.size();++i)if(a.forecast[i].t>=elapsed) {
        const auto & p=a.forecast[i-1];const auto & q=a.forecast[i];
        const double f=std::clamp((elapsed-p.t)/std::max(.001,q.t-p.t),0.,1.);
        return {elapsed,p.x+f*(q.x-p.x),p.y+f*(q.y-p.y),p.error+f*(q.error-p.error)};
      }
    }
    if(a.confident)return {elapsed,a.x+a.vx*elapsed,a.y+a.vy*elapsed,a.uncertainty+.06*elapsed};
    return {elapsed,a.x,a.y,a.uncertainty+.65*elapsed};
  }
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_,mission_;
  std::mutex mutex_;std::vector<SensorTrack> tracks_;bool loaded_{false};
  std::chrono::steady_clock::time_point received_{};
};

// Keeps the validated static path follower outside dynamic conflict zones.
// Both children live in the same Nav2 action/controller: no goal cancellation,
// direct-channel leases or late mover-hold takeover are introduced.
class AdaptiveDynamicController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_=parent.lock();name_=name;tf_=tf;
    nav2_util::declare_parameter_if_not_declared(node_,name+".nominal_plugin",
      rclcpp::ParameterValue("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
    auto nominal_plugin=node_->get_parameter(name+".nominal_plugin").as_string();
    loader_=std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>("nav2_core","nav2_core::Controller");
    nominal_=loader_->createSharedInstance(nominal_plugin);
    dynamic_=loader_->createSharedInstance("nav2_mppi_controller::MPPIController");
    nominal_->configure(parent,name+".Nominal",tf,costmap);
    dynamic_->configure(parent,name+".Dynamic",tf,costmap);
    tracks_.configure(node_,costmap);
    status_=node_->create_publisher<std_msgs::msg::String>("/competition/dynamic_controller/state",10);
    RCLCPP_INFO(node_->get_logger(),"%s adaptive controller: static follower + sensor-time MPPI",name.c_str());
  }
  void cleanup()override{nominal_->cleanup();dynamic_->cleanup();}
  void activate()override{nominal_->activate();dynamic_->activate();status_->on_activate();}
  void deactivate()override{nominal_->deactivate();dynamic_->deactivate();status_->on_deactivate();}
  void setPlan(const nav_msgs::msg::Path & path)override {
    path_=path;nominal_->setPlan(path);dynamic_plan_pending_=true;
  }
  void setSpeedLimit(const double & speed,const bool & percentage)override {
    nominal_->setSpeedLimit(speed,percentage);dynamic_->setSpeedLimit(speed,percentage);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * checker)override
  {
    const double now=node_->now().seconds();auto tracks=tracks_.snapshot(now);
    double tx=0,ty=0,heading=0;
    if(path_.header.frame_id!=pose.header.frame_id) {
      const auto transform=tf_->lookupTransform(pose.header.frame_id,path_.header.frame_id,tf2::TimePointZero);
      tx=transform.transform.translation.x;ty=transform.transform.translation.y;
      heading=tf2::getYaw(transform.transform.rotation);
    }
    const double cosine=std::cos(heading),sine=std::sin(heading);
    std::vector<std::pair<double,double>> path;
    size_t nearest=0;double best=1.e10;
    for(const auto & p:path_.poses) {
      const auto & q=p.pose.position;
      path.emplace_back(tx+cosine*q.x-sine*q.y,ty+sine*q.x+cosine*q.y);
      const double distance=std::hypot(path.back().first-pose.pose.position.x,
        path.back().second-pose.pose.position.y);
      if(distance<best){best=distance;nearest=path.size()-1;}
    }
    geometry_msgs::msg::TwistStamped baseline;
    std::exception_ptr nominal_error;
    try{baseline=nominal_->computeVelocityCommands(pose,velocity,checker);}
    catch(...){nominal_error=std::current_exception();}
    const double preview_speed=std::max(.6,std::hypot(baseline.twist.linear.x,baseline.twist.linear.y));
    bool conflict=false;std::vector<int> ids;
    for(const auto & a:tracks) {
      if(std::hypot(a.x-pose.pose.position.x,a.y-pose.pose.position.y)>4.)continue;
      bool intersects=nominal_error!=nullptr;
      double travelled=0.;
      for(size_t i=nearest;i<path.size() && !intersects;++i) {
        if(i>nearest)travelled+=std::hypot(path[i].first-path[i-1].first,path[i].second-path[i-1].second);
        if(travelled>3.0)break;
        const double arrival=travelled/preview_speed;
        if(arrival>2.0)break;
        for(double slack:{-.25,0.,.25}) {
          const double elapsed=std::max(0.,now-a.stamp)+std::max(0.,arrival+slack);
          auto f=a.confident?SensorTracks::at(a,elapsed):ForecastPoint{
            elapsed,a.x+a.vx*elapsed,a.y+a.vy*elapsed,a.uncertainty};
          if(std::hypot(f.x-path[i].first,f.y-path[i].second)<a.radius+.62){intersects=true;break;}
        }
      }
      if(intersects){conflict=true;ids.push_back(a.id);}
    }
    if(conflict)last_conflict_=now;
    const bool use_dynamic=conflict || (active_ && now-last_conflict_<.3);
    if(!use_dynamic && nominal_error)std::rethrow_exception(nominal_error);
    if(use_dynamic && (!active_ || dynamic_plan_pending_)) {
      dynamic_->setPlan(remainingPath(path_,nearest));
      dynamic_plan_pending_=false;
    }
    auto command=use_dynamic?dynamic_->computeVelocityCommands(pose,velocity,checker):baseline;
    if(use_dynamic!=active_ || now-last_status_>.19) {
      active_=use_dynamic;last_status_=now;
      nlohmann::json j={{"stamp",now},{"controller",name_},{"mode",active_?"sensor_mppi":"nominal"},
        {"track_ids",ids},{"source","laser_stamped_map_tf"},
        {"selected_command",{command.twist.linear.x,command.twist.linear.y,command.twist.angular.z}},
        {"frame_id",pose.header.frame_id}};
      if(use_dynamic) {
        // Read-only comparison: never publish this command to the actuator.
        try {
          if(nominal_error)std::rethrow_exception(nominal_error);
          j["nominal_command"]={baseline.twist.linear.x,baseline.twist.linear.y,baseline.twist.angular.z};
        }catch(const std::exception & error){j["nominal_rejected"]=error.what();}
      }
      std_msgs::msg::String m;m.data=j.dump();status_->publish(m);
    }
    return command;
  }
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;std::string name_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_core::Controller::Ptr nominal_,dynamic_;
  SensorTracks tracks_;nav_msgs::msg::Path path_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_;
  bool active_{false},dynamic_plan_pending_{true};
  double last_conflict_{-100.},last_status_{-100.};
};
} // namespace bot_navigation

namespace mppi::critics
{
class SensorTimeCritic : public CriticFunction
{
public:
  void initialize() override {
    auto node=parent_.lock();tracks_.configure(node,costmap_ros_);
    auto p=parameters_handler_->getParamGetter(name_);
    p(weight_,"cost_weight",12.0);p(margin_,"clearance",.10);
    p(collision_cost_,"collision_cost",1000000.0);
  }
  void score(CriticData & data) override {
    if(!enabled_)return;
    const double now=parent_.lock()->now().seconds();auto tracks=tracks_.snapshot(now);
    const double body=tracks_.loaded()?.34:.28;
    const auto rows=data.trajectories.x.shape(0),steps=data.trajectories.x.shape(1);
    std::vector<unsigned char> collision(rows,0);
    for(const auto & a:tracks) {
      if(std::hypot(a.x-data.state.pose.pose.position.x,a.y-data.state.pose.pose.position.y)>6.)continue;
      const double age=std::max(0.,now-a.stamp);
      for(size_t k=0;k<steps;++k) {
        const double t=(k+1)*data.model_dt;auto p=bot_navigation::SensorTracks::at(a,age+t);
        const double r=body+a.radius+margin_+p.error;
        for(size_t i=0;i<rows;++i) {
          double dx=data.trajectories.x(i,k)-p.x,dy=data.trajectories.y(i,k)-p.y;
          const double d=std::hypot(dx,dy)-r;
          // Uncertain tracks retain a hard short reaction horizon. The
          // expanding reachable set after that is a risk cost, otherwise a
          // briefly occluded distant mover makes *every* 1.8 s rollout fail.
          // Raw current occupancy and the downstream stopping guard remain
          // hard throughout; this is not a long-horizon safety certificate.
          if(d<0 && (a.confident || t<=.4))collision[i]=1;
          else if(d<.35)data.costs(i)+=weight_*(.35-d)*data.model_dt;
        }
      }
    }
    bool all=rows>0;
    for(size_t i=0;i<rows;++i){if(collision[i])data.costs(i)+=collision_cost_;else all=false;}
    if(all)RCLCPP_WARN_THROTTLE(parent_.lock()->get_logger(),*parent_.lock()->get_clock(),1000,
      "sensor_time_all_rollouts_blocked tracks=%zu",tracks.size());
    data.fail_flag=data.fail_flag || all;
  }
private:
  bot_navigation::SensorTracks tracks_;
  double weight_{12.},margin_{.1},collision_cost_{1.e6};
};
}
PLUGINLIB_EXPORT_CLASS(bot_navigation::AdaptiveDynamicController,nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(mppi::critics::SensorTimeCritic,mppi::critics::CriticFunction)
