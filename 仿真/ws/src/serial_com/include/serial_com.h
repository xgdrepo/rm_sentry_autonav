#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <atomic>

extern serial::Serial sentry_ser;
extern std::string cmd_vel_topic;
extern std::atomic<bool> spin_mode_enabled;

#endif