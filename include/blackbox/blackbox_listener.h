#ifndef BLACKBOX_LISTENER_H
#define BLACKBOX_LISTENER_H

namespace blackbox {

class BlackboxListener {
public:
	virtual void handle_blackbox_message(const uint8_t byte) = 0;
	virtual ~BlackboxListener() {
	}
	;
};

}

#endif
