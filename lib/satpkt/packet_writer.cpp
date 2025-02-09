#include "packet_writer.h"

#include <cstring>

namespace og3::satpkt {
namespace {
uint8_t byte1(uint16_t u16) { return static_cast<uint8_t>(u16 >> 8); }
uint8_t byte2(uint16_t u16) { return static_cast<uint8_t>(u16 & 0xff); }
}

PacketWriter::PacketWriter(uint8_t* buffer, std::size_t nbytes, uint16_t seq_id)
    : m_buffer(buffer), m_buffer_size(nbytes) {
  if (nbytes < kPktHeaderSize) {
    m_is_ok = false;
    return;
  }
  m_buffer[0] = 'o';
  m_buffer[1] = 'g';
  m_buffer[2] = '3';
  m_buffer[3] = 'p';
  m_buffer[4] = kProtocolVersionMajor;
  m_buffer[5] = kProtocolVersionMinor;
  m_buffer[6] = byte1(kPktHeaderSize);
  m_buffer[7] = byte2(kPktHeaderSize); 
  m_pkt_size = kPktHeaderSize;
  m_buffer[8] = byte1(seq_id);
  m_buffer[9] = byte2(seq_id);
  m_buffer[10] = 0;
  m_buffer[11] = 0;
  m_num_msgs = 0;
}

bool PacketWriter::add_message(uint16_t type, const uint8_t* msg, uint16_t msg_size) {
  if (!is_ok()) {
    return false;
  }
  const uint32_t msg_end = m_pkt_size + kPktMsgHeaderSize + (msg_size + 3) / 4 * 4;
  if (msg_end > m_buffer_size) {
    return false;
  }
  uint8_t* msg_start = m_buffer + m_pkt_size;
  msg_start[0] = byte1(msg_size);
  msg_start[1] = byte2(msg_size);
  msg_start[2] = byte1(type);
  msg_start[3] = byte2(type);
  memcpy(msg_start + 4, msg, msg_size);
  static_assert(kPktMsgHeaderSize == 4);
  m_num_msgs += 1;
  m_buffer[10] = byte1(m_num_msgs);
  m_buffer[11] = byte2(m_num_msgs);
  m_pkt_size = msg_end;
  m_buffer[6] = byte1(m_pkt_size);
  m_buffer[7] = byte2(m_pkt_size);
  return true;
}
}  // namespace og3::satpkt
