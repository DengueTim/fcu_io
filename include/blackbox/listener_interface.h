#ifndef BLACKBOX_LISTENER_INTERFACE_H
#define BLACKBOX_LISTENER_INTERFACE_H

namespace blackbox {

class ListenerInterface {
public:
	virtual void handle_blackbox_message(const std::string &msg) = 0;
	virtual ~ListenerInterface();
};

}

#endif // BLACKBOX_LISTENER_INTERFACE_H
