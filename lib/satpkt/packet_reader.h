#pragma once
#include "packet.h"

namespace og3::satpkt {

inline bool is_pkt(const uint8_t* buffer, std::size_t nbytes) {
  return nbytes >= kPktHeaderSize && buffer[0] == 'c' && buffer[1] == '3' &&
         buffer[2] == kProtocolVersionMajor && buffer[3] >= kProtocolVersionMinor;
}

class PacketReader {
 public:
  PacketReader(const uint8_t* buffer, std::size_t nbytes);

  enum ParseResult : int {
    kOk = 0,
    kBadSize = 1,
    kBadPrefix = 2,
    kBadVersion = 3,
  };
  static constexpr uint16_t kMaxAvailableMsgs = 16;

  ParseResult parse();

  bool get_msg(std::size_t msg_index, const uint8_t** out_msg, uint16_t* out_msg_type,
               std::size_t* out_msg_size) const;

 protected:
  const uint8_t* m_buffer;
  uint16_t m_buffer_size;
  bool m_is_ok = false;
  uint8_t m_protocol_version_major = 0xff;
  uint8_t m_protocol_version_minor = 0xff;
  uint16_t m_pkt_size = 0;
  uint16_t m_seq_id = 0xff;
  uint16_t m_num_msgs = 0;
  uint16_t m_num_available_msgs = 0;
  uint16_t m_msg_sizes[kMaxAvailableMsgs];
  uint16_t m_msg_offsets[kMaxAvailableMsgs];
};

}  // namespace og3::satpkt
