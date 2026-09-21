namespace
{

constexpr uint8_t kLedPin = 9;
constexpr bool kLedActiveHigh = true;
constexpr unsigned long kPreDarkMs = 1500;
constexpr unsigned long kPostDarkMs = 1000;
constexpr uint8_t kRepeatCount = 2;

struct Step
{
  bool on;
  unsigned long duration_ms;
};

// Durations are deliberately non-round so that RGB edges do not repeatedly
// land at the same phase of common 30/60/90 Hz frame periods.
constexpr Step kPattern[] = {
  {true, 137},
  {false, 211},
  {true, 293},
  {false, 163},
  {true, 421},
  {false, 251},
  {true, 179},
  {false, 613},
};

void set_led(const bool on)
{
  const bool pin_high = kLedActiveHigh ? on : !on;
  digitalWrite(kLedPin, pin_high ? HIGH : LOW);
}

void log_edge(const uint8_t repeat, const uint8_t index, const bool on)
{
  if (!Serial) {
    return;
  }
  Serial.print("EDGE,");
  Serial.print(repeat);
  Serial.print(',');
  Serial.print(index);
  Serial.print(',');
  Serial.print(on ? 1 : 0);
  Serial.print(',');
  Serial.println(micros());
}

void emit_pattern()
{
  if (Serial) {
    Serial.println("SYNC_BEGIN");
  }

  for (uint8_t repeat = 0; repeat < kRepeatCount; ++repeat) {
    for (uint8_t index = 0; index < sizeof(kPattern) / sizeof(kPattern[0]); ++index) {
      const Step & step = kPattern[index];
      set_led(step.on);
      log_edge(repeat, index, step.on);
      delay(step.duration_ms);
    }
  }

  set_led(false);
  if (Serial) {
    Serial.println("SYNC_END");
  }
}

}  // namespace

void setup()
{
  pinMode(kLedPin, OUTPUT);
  set_led(false);
  Serial.begin(115200);

  // The bootloader may add its own delay. This guard starts only after setup().
  delay(kPreDarkMs);
  emit_pattern();
  delay(kPostDarkMs);
}

void loop()
{
  // One optical marker per reset/power-on. Keep the LED dark afterwards.
  set_led(false);
  delay(1000);
}
