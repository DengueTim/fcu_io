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

namespace blackbox {

class Blackbox : public SerialListener {
public:
	Blackbox(std::string port, int baud_rate, BlackboxListener * const listener);

	~Blackbox();

	void serial_data_received(const uint8_t byte);

	void serial_data_send(float roll, float pitch, float yaw, float trottle);
private:
	Serial serial_;
	BlackboxListener* listener_;
};

}

#endif
