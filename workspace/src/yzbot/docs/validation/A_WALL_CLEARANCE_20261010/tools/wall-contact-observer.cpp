#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <map>
#include <string>

std::mutex guard;
unsigned long frames = 0, wallHits = 0, robotHits = 0;
double firstTime = -1, lastTime = -1;
std::map<std::string, unsigned long> pairs;

void receive(ConstContactsPtr &msg) {
  std::lock_guard<std::mutex> lock(guard);
  ++frames;
  double t = msg->time().sec() + msg->time().nsec() * 1e-9;
  if (firstTime < 0) firstTime = t;
  lastTime = t;
  for (int i = 0; i < msg->contact_size(); ++i) {
    const auto &c = msg->contact(i);
    const std::string a = c.collision1(), b = c.collision2();
    const bool robot = a.find("six_arm") != std::string::npos ||
                       b.find("six_arm") != std::string::npos;
    const bool cubeWall = (a.find("Wall_") != std::string::npos ||
                          b.find("Wall_") != std::string::npos) &&
                         (a.find("_cube_") != std::string::npos ||
                          b.find("_cube_") != std::string::npos);
    if (!robot && !cubeWall) continue;
    if (robot) ++robotHits;
    const std::string other = a.find("six_arm") != std::string::npos ? b : a;
    if (other.find("ground_plane") != std::string::npos ||
        other.find("six_arm") != std::string::npos) continue;
    unsigned long count = ++pairs[a + " | " + b];
    if (a.find("Wall_") != std::string::npos ||
        b.find("Wall_") != std::string::npos) ++wallHits;
    double depth = 0;
    for (int j = 0; j < c.depth_size(); ++j) depth = std::max(depth, c.depth(j));
    if (count <= 5 || count % 250 == 0)
      std::cout << "CONTACT t=" << t << " pair=" << a << " | " << b
                << " depth=" << depth << " count=" << count << std::endl;
  }
}

int main(int argc, char **argv) {
  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init("my_world");
  auto subscriber = node->Subscribe("~/physics/contacts", receive);
  const std::string stop = argc > 1 ? argv[1] : "/tmp/no-contact-stop";
  for (int i = 0; i < 1000; ++i) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    {
      std::lock_guard<std::mutex> lock(guard);
      if (i % 10 == 0)
        std::cout << "STATS frames=" << frames << " robot_contacts=" << robotHits
                  << " wall_contacts=" << wallHits << " first_sim=" << firstTime
                  << " last_sim=" << lastTime << std::endl;
    }
    if (std::ifstream(stop).good()) break;
  }
  {
    std::lock_guard<std::mutex> lock(guard);
    std::cout << "FINAL frames=" << frames << " robot_contacts=" << robotHits
              << " wall_contacts=" << wallHits << " first_sim=" << firstTime
              << " last_sim=" << lastTime << std::endl;
    for (const auto &p : pairs) std::cout << "PAIR " << p.first << " count=" << p.second << std::endl;
  }
  gazebo::client::shutdown();
}
