/*
  Author: Brett Lopez
  Contact: btlopez@ucla.edu
  Date: Oct 3, 2022
*/

#include "RosManager.hpp"

std::atomic_bool RosManager::RosNode::abort_(false);

RosManager::RosNode::RosNode(ros::NodeHandle node_handle) : nh(node_handle) {
  this->stop_serial_thread = false;

  // UART params
  std::string port_name;
  ros::param::param<std::string>("~port_name", port_name, "/dev/ttyUSB0");

  int baud_rate;
  ros::param::param<int>("~baud_rate", baud_rate, 9600);

  this->baro_pub =
      this->nh.advertise<sensor_msgs::FluidPressure>("pressure", 1);

  this->abort_timer = this->nh.createTimer(ros::Duration(0.01),
                                           &RosManager::RosNode::TimerCB, this);

  this->serial_manager.spInitialize(port_name, baud_rate);

  this->ns = ros::this_node::getNamespace();
  this->ns.erase(0, 1);
  // std::cout << "name: " << this->ns << std::endl;

  ROS_INFO("ICP-101xx Reader Node Initialized");
}

RosManager::RosNode::~RosNode() {
  // TODO
}

void RosManager::RosNode::TimerCB(const ros::TimerEvent &e) {
  if (abort_) {
    Stop();
  }
}

void RosManager::RosNode::Start() {
  this->serial_read_thread =
      std::thread(&RosManager::RosNode::SerialThreadMain, this);

  ros::Duration(1.0).sleep();
}

void RosManager::RosNode::Stop() {
  ROS_WARN("KILLING ICP-101xx NODE...");

  this->stop_serial_thread = true;
  if (this->serial_read_thread.joinable()) {
    this->serial_read_thread.join();
  }

  this->serial_manager.spClose();
  ros::shutdown();
}

void RosManager::RosNode::SerialThreadMain() {
  ROS_WARN("STARTING ICP-101xx READ THREAD");

  static bool first = true;

  while (!this->stop_serial_thread) {
    uint8_t header = serial_manager.spReceiveSingle();
    if (header == HEADER) {
      uint8_t len;
      uint8_t id = serial_manager.spReceiveSingle();
      if (id == PACKETID_BARO) {
        len = sizeof(vectr::baroPkt);
      } else {
        ROS_ERROR("UNKNOWN PACKET ID");
        break;
      }

      uint8_t data[len];
      data[0] = HEADER;
      data[1] = id;
      uint8_t valid = ProcessPkt(data, len);

      if (!valid) {
        ROS_ERROR("BAD PKT WITH ID %i", id);
      } else {
        switch (id) {
        case PACKETID_BARO:
          vectr::baroPkt baro_pkt;
          memcpy(&baro_pkt, data, sizeof(data));
          PublishBaroPkt(baro_pkt);
          break;
        }
      }
    }
  }
  ROS_WARN("EXITING ICP-101xx READ THREAD");
}

void RosManager::RosNode::PublishBaroPkt(const vectr::baroPkt baro_pkt) {
  sensor_msgs::FluidPressure baro;

  baro.header.stamp = ros::Time::now();
  baro.fluid_pressure = baro_pkt.pres / 1000.;

  this->baro_pub.publish(baro);
}

uint8_t RosManager::RosNode::ProcessPkt(uint8_t data[], uint8_t len) {
  uint8_t ret;
  uint8_t checksum = data[0] + data[1];
  for (int i = 2; i < len; i++) {
    data[i] = serial_manager.spReceiveSingle();
    if (i < len - 1) {
      checksum += data[i];
    }
  }
  if (checksum == data[len - 1]) {
    ret = 1;
  } else {
    ret = 0;
  }
  return ret;
}