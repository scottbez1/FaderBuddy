/*
 * FaderBuddyHost basic example: one fader on the default I2C bus.
 *
 * Prints touches and moves, and sends the fader to the far end of its travel
 * whenever it is double-tapped.
 */

#include <Arduino.h>
#include <Wire.h>

#include <FaderBuddyHost.h>

using namespace faderbuddy;

WireTransport transport(Wire, 0x20);  // 0x20 + the board's address jumpers
ArduinoPlatform platform(&Serial, LogLevel::INFO);
FaderBuddy fader(transport, platform);

class Events : public FaderBuddyListener {
  void on_manual_move(uint8_t position, uint8_t layer) override {
    Serial.printf("moved to %u (layer %u)\n", position, layer);
  }
  void on_touch_change(bool touched, uint8_t layer) override {
    Serial.printf("%s (layer %u)\n", touched ? "touched" : "released", layer);
  }
  void on_double_tap(uint8_t layer) override {
    uint8_t target = fader.get_last_state().position < 128 ? 255 : 0;
    fader.remote_move_to(target, layer, 128);  // half speed
  }
};
Events events;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  fader.set_listener(&events);
  fader.set_layer_value_change_min_interval(0, 50);  // at most 20 moves/s
  fader.set_poll_interval(10);
  fader.begin();
}

void loop() { fader.loop(); }
