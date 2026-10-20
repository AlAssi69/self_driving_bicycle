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

#ifndef BICYCLE_HARDWARE__SERIAL_PORT_HPP_
#define BICYCLE_HARDWARE__SERIAL_PORT_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>

namespace bicycle_hardware
{

// Line-oriented serial link. Timeouts return false; lost devices and other I/O failures
// throw std::system_error.
class SerialPort
{
public:
  virtual ~SerialPort() = default;

  // Throws std::system_error when the device cannot be opened or configured, and
  // std::invalid_argument for an unsupported baud rate.
  virtual void open(const std::string & device, uint32_t baud_rate) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual bool write(std::string_view data, std::chrono::milliseconds timeout) = 0;
  // Reads one line without its terminator (and without a trailing '\r').
  virtual bool read_line(std::string & line, std::chrono::milliseconds timeout) = 0;
  // Drops buffered input, e.g. a late reply to a timed-out request.
  virtual void discard_input() = 0;
};

// SerialPort on a POSIX tty (8N1, raw, no flow control).
class PosixSerialPort : public SerialPort
{
public:
  PosixSerialPort();
  ~PosixSerialPort() override;
  PosixSerialPort(const PosixSerialPort &) = delete;
  PosixSerialPort & operator=(const PosixSerialPort &) = delete;

  void open(const std::string & device, uint32_t baud_rate) override;
  void close() override;
  bool is_open() const override;
  bool write(std::string_view data, std::chrono::milliseconds timeout) override;
  bool read_line(std::string & line, std::chrono::milliseconds timeout) override;
  void discard_input() override;

private:
  // Longest partial line kept while waiting for a terminator; longer input is garbage.
  static constexpr std::size_t kMaxBuffered = 256;

  bool take_line(std::string & line);

  int fd_{-1};
  std::string buffer_;
};

}  // namespace bicycle_hardware

#endif  // BICYCLE_HARDWARE__SERIAL_PORT_HPP_
