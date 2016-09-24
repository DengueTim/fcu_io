#ifndef BLACKBOX_SERIAL_LISTENER_H
#define BLACKBOX_SERIAL_LISTENER_H

#include <stddef.h>
#include <stdint.h>

namespace blackbox {

class SerialListener {
public:
	virtual void serial_data_received(const uint8_t* const data, const size_t length) = 0;
	virtual ~SerialListener() {};
};

}

#endif
