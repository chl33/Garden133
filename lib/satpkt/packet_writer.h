#pragma once
#include "packet.h"

namespace og3::satpkt {

class PacketWriter {
 public:
  PacketWriter(uint8_t* buffer, std::size_t nbytes, uint16_t seq_num);
  bool is_ok() const { return m_is_ok; }
  bool add_message(uint16_t type, const uint8_t* msg, uint16_t msg_size);

 protected:
  uint8_t* m_buffer;
  const std::size_t m_buffer_size;
  bool m_is_ok = true;
  uint16_t m_pkt_size = 0;
  uint16_t m_num_msgs = 0;
};

}  // namespace og3::satpkt
