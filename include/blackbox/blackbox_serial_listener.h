#ifndef BLACKBOX_SERIAL_LISTENER_H
#define BLACKBOX_SERIAL_LISTENER_H

namespace blackbox {

class BlackboxSerialListener {
public:
	virtual void handle_blackbox_message(const std::string msg) = 0;
	virtual ~ListenerInterface();
};

}

#endif
