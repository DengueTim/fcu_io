#ifndef BLACKBOX_LISTENER_H
#define BLACKBOX_LISTENER_H

namespace blackbox {

class BlackboxListener {
public:
	virtual void handle_blackbox_message(const std::string msg) = 0;
	virtual ~BlackboxListener();
};

}

#endif
