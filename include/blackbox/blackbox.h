/**
 * A blatant copy of code from Mavrosflight
 * \file mavrosflight.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */
#ifndef BLACKBOX_BLACKBOX_H
#define BLACKBOX_BLACKBOX_H

#include <blackbox/blackbox_listener.h>
#include <blackbox/serial.h>
#include <boost/function.hpp>

#include <stdint.h>
#include <string>

namespace blackbox
{

class Blackbox : public SerialListener
{
public:
  Blackbox(std::string port, int baud_rate, BlackboxListener &listener);

  ~Blackbox();

  void serial_data_received(const uint8_t* data, const size_t length);

private:
  Serial serial;
};

}

#endif
