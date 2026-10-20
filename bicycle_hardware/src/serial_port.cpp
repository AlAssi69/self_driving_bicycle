// Copyright 2024-2026 Ashraf AlAssi, Mhd. Osama Keakaty, Sarah AlAhmad
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bicycle_hardware/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <string>
#include <system_error>

namespace bicycle_hardware
{
namespace
{

speed_t to_speed(uint32_t baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:
      throw std::invalid_argument("unsupported baud rate " + std::to_string(baud_rate));
  }
}

[[noreturn]] void throw_errno(const std::string & what)
{
  throw std::system_error(errno, std::generic_category(), what);
}

using Clock = std::chrono::steady_clock;

int remaining_ms(Clock::time_point deadline)
{
  const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
  return left.count() > 0 ? static_cast<int>(left.count()) : 0;
}

}  // namespace

PosixSerialPort::PosixSerialPort()
{
  buffer_.reserve(kMaxBuffered);
}

PosixSerialPort::~PosixSerialPort()
{
  close();
}

void PosixSerialPort::open(const std::string & device, uint32_t baud_rate)
{
  close();
  const speed_t speed = to_speed(baud_rate);
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    throw_errno("cannot open " + device);
  }
  termios tty{};
  if (::tcgetattr(fd, &tty) != 0) {
    const int error = errno;
    ::close(fd);
    throw std::system_error(error, std::generic_category(), device + " is not a tty");
  }
  ::cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(CSTOPB | CRTSCTS);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0 ||
    ::tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    const int error = errno;
    ::close(fd);
    throw std::system_error(error, std::generic_category(), "cannot configure " + device);
  }
  ::tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  buffer_.clear();
}

void PosixSerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  buffer_.clear();
}

bool PosixSerialPort::is_open() const
{
  return fd_ >= 0;
}

bool PosixSerialPort::write(std::string_view data, std::chrono::milliseconds timeout)
{
  const auto deadline = Clock::now() + timeout;
  while (!data.empty()) {
    const ssize_t written = ::write(fd_, data.data(), data.size());
    if (written > 0) {
      data.remove_prefix(static_cast<std::size_t>(written));
      continue;
    }
    if (written < 0 && errno != EAGAIN && errno != EINTR) {
      throw_errno("serial write failed");
    }
    pollfd pfd{fd_, POLLOUT, 0};
    const int ready = ::poll(&pfd, 1, remaining_ms(deadline));
    if (ready == 0) {
      return false;
    }
    if (ready < 0 && errno != EINTR) {
      throw_errno("serial poll failed");
    }
  }
  return true;
}

bool PosixSerialPort::read_line(std::string & line, std::chrono::milliseconds timeout)
{
  const auto deadline = Clock::now() + timeout;
  char chunk[64];
  while (!take_line(line)) {
    pollfd pfd{fd_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, remaining_ms(deadline));
    if (ready == 0) {
      return false;
    }
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw_errno("serial poll failed");
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::system_error(EIO, std::generic_category(), "serial device lost");
    }
    const ssize_t received = ::read(fd_, chunk, sizeof(chunk));
    if (received < 0 && errno != EAGAIN && errno != EINTR) {
      throw_errno("serial read failed");
    }
    if (received == 0) {
      throw std::system_error(EIO, std::generic_category(), "serial device lost");
    }
    if (received > 0) {
      if (buffer_.size() + static_cast<std::size_t>(received) > kMaxBuffered) {
        buffer_.clear();  // no terminator in sight: noise, drop it
      }
      buffer_.append(chunk, static_cast<std::size_t>(received));
    }
  }
  return true;
}

void PosixSerialPort::discard_input()
{
  if (fd_ >= 0) {
    ::tcflush(fd_, TCIFLUSH);
  }
  buffer_.clear();
}

bool PosixSerialPort::take_line(std::string & line)
{
  const auto end = buffer_.find('\n');
  if (end == std::string::npos) {
    return false;
  }
  std::size_t length = end;
  if (length > 0 && buffer_[length - 1] == '\r') {
    --length;
  }
  line.assign(buffer_, 0, length);
  buffer_.erase(0, end + 1);
  return true;
}

}  // namespace bicycle_hardware
