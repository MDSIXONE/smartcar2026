#ifndef UCAR_CONTROLLER_SERIAL_READ_EXACT_H_
#define UCAR_CONTROLLER_SERIAL_READ_EXACT_H_

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ucarController
{

struct SerialReadResult
{
  SerialReadResult(size_t requested_value, size_t received_value,
                   size_t read_calls_value)
      : requested(requested_value), received(received_value),
        read_calls(read_calls_value) {}

  bool complete() const
  {
    return received == requested;
  }

  size_t requested;
  size_t received;
  size_t read_calls;
};

// Read an entire serial field even when the operating system splits it over
// several successful read() calls. The caller supplies a total-field budget
// predicate, evaluated before every retry after the first. A zero-byte read,
// overrun, or expired budget leaves an incomplete result that must be discarded.
template <typename Reader, typename ContinueReading>
SerialReadResult serialReadExactly(
    uint8_t* destination, size_t requested, Reader reader,
    ContinueReading continue_reading)
{
  size_t received = 0;
  size_t read_calls = 0;
  std::memset(destination, 0, requested);
  while (received < requested)
  {
    if (received > 0 && !continue_reading())
    {
      break;
    }
    const size_t remaining = requested - received;
    const size_t count = reader(destination + received, remaining);
    ++read_calls;
    if (count == 0 || count > remaining)
    {
      break;
    }
    received += count;
  }
  return SerialReadResult(requested, received, read_calls);
}

template <typename Reader>
SerialReadResult serialReadExactly(
    uint8_t* destination, size_t requested, Reader reader)
{
  return serialReadExactly(destination, requested, reader,
                           []() { return true; });
}

}  // namespace ucarController

#endif  // UCAR_CONTROLLER_SERIAL_READ_EXACT_H_
