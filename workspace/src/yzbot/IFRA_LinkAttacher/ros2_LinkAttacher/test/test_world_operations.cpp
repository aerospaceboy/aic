#include "ros2_linkattacher/world_operations.hpp"
#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <vector>
using ros2_linkattacher::WorldOperations;
using namespace std::chrono_literals;

template<class T> void finish(WorldOperations & queue,std::future<T> & result) {
  auto deadline=std::chrono::steady_clock::now()+2s;
  while(result.wait_for(0ms)!=std::future_status::ready && std::chrono::steady_clock::now()<deadline) {
    queue.drain();std::this_thread::yield();
  }
  ASSERT_EQ(result.wait_for(0ms),std::future_status::ready);
}
TEST(WorldOperations, PhysicsChangesExecuteOnlyOnWorldThread) {
  WorldOperations queue;auto world=std::this_thread::get_id();std::thread::id actual;
  auto result=std::async(std::launch::async,[&](){return queue.invoke<int>([&](){actual=std::this_thread::get_id();return 7;},1s);});
  finish(queue,result);EXPECT_EQ(result.get(),7);EXPECT_EQ(actual,world);
}
TEST(WorldOperations, ExpiredRequestCannotAttachAfterServiceReturnsFailure) {
  WorldOperations queue;int calls=0;
  EXPECT_THROW(queue.invoke<int>([&](){++calls;return 1;},0ms),std::runtime_error);
  queue.drain();EXPECT_EQ(calls,0);
}
TEST(WorldOperations, ConcurrentServiceRequestsExecuteExactlyOnce) {
  WorldOperations queue;int calls=0;std::vector<std::future<int>> results;
  for(int i=0;i<20;i++)results.emplace_back(std::async(std::launch::async,[&](){return queue.invoke<int>([&](){return ++calls;},1s);}));
  for(auto & result:results)finish(queue,result);
  for(auto & result:results)EXPECT_GT(result.get(),0);
  queue.drain();EXPECT_EQ(calls,20);
}
TEST(WorldOperations, PhysicsExceptionsBecomeServiceFailures) {
  WorldOperations queue;
  auto result=std::async(std::launch::async,[&](){return queue.invoke<int>([]()->int{throw std::logic_error("cannot attach");},1s);});
  finish(queue,result);EXPECT_THROW(result.get(),std::logic_error);
}
