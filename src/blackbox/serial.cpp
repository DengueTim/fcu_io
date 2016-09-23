#include <exception>
#include <blackbox/serial.h>
#include <boost/thread.hpp>
#include <ros/ros.h>

namespace blackbox
{

using boost::asio::serial_port_base;

Serial::Serial(std::string port, int baud_rate) :
  io_service_(),
  serial_port_(io_service_),
  write_in_progress_(false)
{
  // setup serial port
  try
  {
    serial_port_.open(port);
    serial_port_.set_option(serial_port_base::baud_rate(baud_rate));
    serial_port_.set_option(serial_port_base::character_size(8));
    serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  }
  catch (boost::system::system_error e)
  {
    throw std::exception(e);
  }

  // start reading from serial port
  do_async_read();
  io_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &this->io_service_));
}

Serial::~Serial()
{
  close();
}

void Serial::register_listener(SerialListener * const listener)
{
  if (listener == NULL)
    return;

  bool already_registered = false;
  for (int i = 0; i < listeners_.size(); i++)
  {
    if (listener == listeners_[i])
    {
      already_registered = true;
      break;
    }
  }

  if (!already_registered)
    listeners_.push_back(listener);
}

void Serial::unregister_listener(SerialListener * const listener)
{
  if (listener == NULL)
    return;

  for (int i = 0; i < listeners_.size(); i++)
  {
    if (listener == listeners_[i])
    {
      listeners_.erase(listeners_.begin() + i);
      i--;
    }
  }
}

void Serial::close()
{
  mutex_lock lock(mutex_);

  io_service_.stop();
  serial_port_.close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void Serial::do_async_read()
{
  if (!serial_port_.is_open()) return;

  serial_port_.async_read_some(
        boost::asio::buffer(read_buf_raw_, SERIAL_READ_BUF_SIZE),
        boost::bind(
          &Serial::async_read_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}

void Serial::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (!serial_port_.is_open()) return;

  if (error)
  {
    close();
    return;
  }

  for (int i = 0; i < listeners_.size(); i++) {
    listeners_[i]->serial_data_received(read_buf_raw_, bytes_transferred);
  }

  do_async_read();
}

void Serial::send_data(const uint8_t* const data, const size_t length)
{
  WriteBuffer *buffer = new WriteBuffer();
  buffer->len = length;
  assert(buffer->len <= SERIAL_WRITE_BUF_SIZE); //! \todo Do something less catastrophic here

  {
    mutex_lock lock(mutex_);
    write_queue_.push_back(buffer);
  }

  do_async_write(true);
}

void Serial::do_async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_)
    return;

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
    return;

  write_in_progress_ = true;
  WriteBuffer *buffer = write_queue_.front();
  serial_port_.async_write_some(
        boost::asio::buffer(buffer->dpos(), buffer->nbytes()),
        boost::bind(
          &Serial::async_write_end,
          this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));

}

void Serial::async_write_end(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (error)
  {
    close();
    return;
  }

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
  {
    write_in_progress_ = false;
    return;
  }

  WriteBuffer *buffer = write_queue_.front();
  buffer->pos += bytes_transferred;
  if (buffer->nbytes() == 0)
  {
    write_queue_.pop_front();
    delete buffer;
  }

  if (write_queue_.empty())
    write_in_progress_ = false;
  else
    do_async_write(false);
}

}
