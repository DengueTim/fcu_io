#include <string>
#include <stdint.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "fcu_io.h"

namespace blackbox
{

blackbox::Blackbox()
{
	 serial.register_listener(this);
	//  mavrosflight_->param.register_param_listener(this);
}

blackbox::~Blackbox(){

}
