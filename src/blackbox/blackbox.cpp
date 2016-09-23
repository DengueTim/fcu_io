#include <string>
#include <stdint.h>

#include "blackbox/blackbox.h"

namespace blackbox {

Blackbox::Blackbox(std::string port, int baud_rate, BlackboxListener * const listener) :
		serial_(port, baud_rate),
		listener_(listener) {
	// serial.register_listener(this);
	//  mavrosflight_->param.register_param_listener(this);
}

Blackbox::~Blackbox(){

}

void Blackbox::serial_data_received(const uint8_t* data, const size_t length) {
}

}
