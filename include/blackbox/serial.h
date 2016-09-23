#ifndef BLACKBOX_SERIAL_H
#define BLACKBOX_SERIAL_H

#include <blackbox/serial_listener.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <list>
#include <string>
#include <vector>
#include <exception>

#include <stdint.h>

#define SERIAL_READ_BUF_SIZE 256
#define SERIAL_WRITE_BUF_SIZE 256

namespace blackbox
{

class Serial
{
public:

  Serial(std::string port, int baud_rate);

  ~Serial();

  void register_listener(SerialListener * const listener);

  void unregister_listener(SerialListener * const listener);

  void send_data(const uint8_t* const data, const size_t length);

private:

  struct WriteBuffer
  {
    uint8_t data[SERIAL_WRITE_BUF_SIZE];
    size_t len;
    size_t pos;

    WriteBuffer() : len(0), pos(0) {}

    WriteBuffer(const uint8_t * buf, uint16_t len) : len(len), pos(0)
    {
      assert(len <= SERIAL_WRITE_BUF_SIZE); //! \todo Do something less catastrophic here
      memcpy(data, buf, len);
    }

    uint8_t * dpos() { return data + pos; }

    size_t nbytes() { return len - pos; }
  };

  typedef boost::lock_guard<boost::recursive_mutex> mutex_lock;

  void do_async_read();

  void async_read_end(const boost::system::error_code& error, size_t bytes_transferred);

  /**
   * \brief Initialize an asynchronous write operation
   * \param check_write_state If true, only start another write operation if a write sequence is not already running
   */
  void do_async_write(bool check_write_state);

  /**
   * \brief Handler for end of asynchronous write operation
   * \param error Error code
   * \param bytes_transferred Number of bytes sent
   */
  void async_write_end(const boost::system::error_code& error, size_t bytes_transferred);

  /**
   * \brief Stops communication and closes the serial port
   */
  void close();


  std::vector<SerialListener*> listeners_;

  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  boost::thread io_thread_; 
  boost::recursive_mutex mutex_;

  uint8_t read_buf_raw_[SERIAL_READ_BUF_SIZE];

  std::list<WriteBuffer*> write_queue_;
  bool write_in_progress_;
};

}

#endif // BLACKBOX_SERIAL_H
