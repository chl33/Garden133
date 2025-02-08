#include <array>
#include <functional>
#include <cstdio>
#include <cstring>

#include <unity.h>
#include "packet_reader.h"
#include "packet_writer.h"

void setUp() {
    // set stuff up here
}

void tearDown() {
    // clean stuff up here
}

void test_packet() {
  uint8_t buffer[64];
  memset(buffer, 0, sizeof(buffer));
  constexpr uint16_t kSeqId = 0;
  {
    og3::satpkt::PacketWriter writer(buffer, 10, kSeqId);
    TEST_ASSERT_FALSE(writer.is_ok());
  }
  og3::satpkt::PacketWriter writer(buffer, sizeof(buffer), kSeqId);
  TEST_ASSERT_TRUE(writer.is_ok());
  std::array<uint8_t, 8> msg{0};

  using MsgByteFn = std::function<uint8_t(uint8_t)>;

  auto setmsg = [](const MsgByteFn& fn, std::array<uint8_t, 8>& msg) {
    for (std::size_t i = 0; i < msg.size(); i++) {
      msg[i] = fn(i);
    }
  };
  auto checkmsg = [](const MsgByteFn& fn, const uint8_t* msg, std::size_t sz) -> bool {
    for (std::size_t i = 0; i < sz; i++) {
      if (msg[i] != fn(i)) {
	printf("msg[%zu]:%u != expected:%u\n", i, msg[i], fn(i));
        return false;
      }
    }
    return true;
  };

  const MsgByteFn msg1fn = [](uint8_t i) { return i; };
  setmsg(msg1fn, msg);
  constexpr uint16_t kMsg1Type = 0x1234;
  TEST_ASSERT_TRUE(writer.add_message(kMsg1Type, msg.data(), msg.size()));

  const MsgByteFn msg2fn = [](uint8_t i) { return 7-i; };
  setmsg(msg2fn, msg);
  constexpr uint16_t kMsg2Type = 0x4321;
  TEST_ASSERT_TRUE(writer.add_message(kMsg2Type, msg.data(), msg.size()));

#if 0
  for (std::size_t i = 0; i < sizeof(buffer); i++) {
    printf(" %02x", buffer[i]);
    if ((i + 1) % 8 == 0) {
      printf("\n");
    }
  }
#endif

  og3::satpkt::PacketReader reader(buffer, sizeof(buffer));
  TEST_ASSERT_EQUAL(og3::satpkt::PacketReader::ParseResult::kOk, reader.parse());

  const uint8_t* pmsg = nullptr;
  uint16_t msg_type = 0;
  std::size_t msg_size = 0;
  // Check msg 1
  TEST_ASSERT_TRUE(reader.get_msg(0, &pmsg, &msg_type, &msg_size));
  TEST_ASSERT_EQUAL(kMsg1Type, msg_type);
  TEST_ASSERT_EQUAL(msg.size(), msg_size);
  TEST_ASSERT_TRUE(checkmsg(msg1fn, pmsg, msg_size));
  
  // Check msg 2
  TEST_ASSERT_TRUE(reader.get_msg(1, &pmsg, &msg_type, &msg_size));
  TEST_ASSERT_EQUAL(kMsg2Type, msg_type);
  TEST_ASSERT_EQUAL(msg.size(), msg_size);
  TEST_ASSERT_TRUE(checkmsg(msg2fn, pmsg, msg_size));
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_packet);
    UNITY_END();
}
