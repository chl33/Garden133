# Garden133

Solar-powered board for measuring garden soil moisture level.

## TODO

- [ ] On startup, and every N-hours, send device description.
- [ ] Debug mode: use WiFi and make info available on web interface
  - [ ] Periodically read sensors and send packet with protobufs.
- [ ] Normal mode:
  - [ ] Wake up.  If first wakeup, add device description to proto to send.
  - [ ] Read sensors, send packet.
  - [ ] Go into deep sleep, set timer for wake-up.

- [ ] Board improvements:
  - [ ] Pull-up resistors on some pins (search web for page describing this).
  - [ ] Read battery & solar voltages.
  - [ ] Is one of the SS14 diodes redundant?
  - [ ] Add LED for blinking.
