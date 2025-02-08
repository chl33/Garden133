#pragma once
#include <cstddef>
#include <cstdint>

namespace og3::satpkt {

// Packet:
// Magic              [4] 'og3p'
// ProtocolVersion    [2] major, minor
// PktSize            [2] uint16le, includes header
// SeqId              [2] uint16le
// NumMsgs            [2] uint16le
// - Msg 0
//   Sz               [2] uint16le, includes full msg
//   Type             [2] uint16le: indicates kind of data.
// -...

constexpr size_t kPktHeaderSize = 12;
constexpr uint8_t kProtocolVersionMajor = 0x0;
constexpr uint8_t kProtocolVersionMinor = 0x1;
constexpr size_t kPktMsgHeaderSize = 4;

}  // namespace og3::satpkt
