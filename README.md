# Garden133

Solar-powered board for measuring garden soil moisture level.

## TODO

- [x] On startup, and every N-hours, send device description.
- [x] Debug mode: use WiFi and make info available on web interface
  - [x] Periodically read sensors and send packet with protobufs.
- [x] Normal mode:
  - [x] Wake up.  If first wakeup, add device description to proto to send.
  - [x] Read sensors, send packet.
  - [x] Go into deep sleep, set timer for wake-up.

- [ ] Board improvements:
  - [x] Pull-up resistors on some pins (search web for page describing this).
  - [x] Read battery & solar voltages.
  - [x] Is one of the SS14 diodes redundant?
  - [ ] Add LED for blinking.
