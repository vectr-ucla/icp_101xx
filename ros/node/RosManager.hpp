/*
  Author: Brett Lopez
  Contact: btlopez@ucla.edu
  Date: Oct 3, 2022
*/

#include <ros/ros.h>

#include <icp_101xx/Barometer.h>

#include "serial.h"

#pragma once
#include "pktStructs.h"

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

namespace RosManager {
class RosNode;
}

/**
 * Ros node
 */
class RosManager::RosNode {
public:
  RosNode(ros::NodeHandle node_handle);

  ~RosNode();

  static void Abort() { abort_ = true; }

  void Start();
  void Stop();

private:
  // class methods
  uint8_t ProcessPkt(uint8_t data[], uint8_t len);
  void PublishBaroPkt(const vectr::baroPkt baro_pkt);
  void SerialThreadMain();
  void TimerCB(const ros::TimerEvent &e);

  // data members
  ros::NodeHandle nh;
  ros::Publisher baro_pub;
  ros::Timer abort_timer;

  libserial::SerialPort serial_manager;

  std::thread serial_read_thread;

  static std::atomic_bool abort_;

  std::mutex mtx;

  std::atomic<bool> stop_serial_thread;

  std::string ns;
};
