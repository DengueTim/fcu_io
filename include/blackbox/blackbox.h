/**
 * A blatant copy of code from Mavrosflight
 * \file mavrosflight.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */
#ifndef BLACKBOX_BLACKBOX_H
#define BLACKBOX_BLACKBOX_H

#include <blackbox/blackbox_serial.h>
#include <blackbox/blackbox_serial_listener.h>
#include <boost/function.hpp>

#include <stdint.h>
#include <string>

namespace blackbox
{

class Blackbox : public BlackboxSerialListener
{
public:

  Blackbox(std::string port, int baud_rate);

  ~Blackbox();

  virtual void handle_blackbox_message(const std::string msg);

private:
  Serial serial;

};

}

#endif
