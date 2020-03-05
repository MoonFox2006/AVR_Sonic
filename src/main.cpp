#include <avr/pgmspace.h>
#include <Arduino.h>

const uint8_t SONIC_ECHO_PIN = 8; // ICP1
const uint8_t SONIC_TRIG_PINS[] PROGMEM = { 6, 7 };

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#define LED_PIN LED_BUILTIN
#ifdef LED_PIN
#define LED_LEVEL HIGH
#endif

#define MEDIAN_COUNT  3
#ifdef MEDIAN_COUNT
#define REPEAT_COUNT  2
#endif

static uint16_t captureInput() {
  uint16_t result = 0;

  TCNT1 = 0;
  TIFR1 = bit(ICF1) | bit(OCF1A); // Clear pending interrupts
  TCCR1B |= bit(ICES1); // Detecting rising edge, CTC mode
  TCCR1B |= bit(CS11); // Start Timer1 with prescaler /8
  while (! (TIFR1 & (bit(ICF1) | bit(OCF1A)))); // Wait until Input Capture or Top
  if (TIFR1 & bit(OCF1A)) {
    TCCR1B &= ~(bit(CS10) | bit(CS11) | bit(CS12)); // Stop Timer1
    return 0;
  }
  result = ICR1;
  TCCR1B &= ~bit(ICES1); // Detecting falling edge
  TIFR1 = bit(ICF1); // Clear pending interrupts
  while (! (TIFR1 & (bit(ICF1) | bit(OCF1A)))); // Wait until Input Capture or Top
  TCCR1B &= ~(bit(CS10) | bit(CS11) | bit(CS12)); // Stop Timer1
  if (TIFR1 & bit(OCF1A))
    return 0;
  result = ICR1 - result;
  return result;
}

static uint16_t sonicMeasure(uint8_t trigPin) {
  uint16_t result;
#ifdef MEDIAN_COUNT
  uint16_t median[MEDIAN_COUNT];
  uint8_t i, len;

  len = MEDIAN_COUNT;
  i = 0;
  while (i < len) {
    uint8_t repeat = REPEAT_COUNT;

    do {
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      result = captureInput();
    } while ((! result) && --repeat);
    if (result) {
      median[i++] = result;
    } else {
      if (len)
        --len;
      else
        break;
    }
  }
  if (len) {
    for (i = 0; i < len - 1; ++i) {
      for (uint8_t j = i + 1; j < len; ++j) {
        if (median[i] > median[j]) {
          uint16_t t = median[i];

          median[i] = median[j];
          median[j] = t;
        }
      }
    }
    result = median[len / 2];
  } else
    result = 0;
#else
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  result = captureInput();
#endif
  return (result / (58 * 2));
}

void setup() {
  Serial.begin(115200);

#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ! LED_LEVEL);
#endif

  pinMode(SONIC_ECHO_PIN, INPUT);
  for (uint8_t i = 0; i < ARRAY_SIZE(SONIC_TRIG_PINS); ++i) {
    uint8_t pin = pgm_read_byte(&SONIC_TRIG_PINS[i]);

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Setup Timer1
  TCCR1A = 0;
  TCCR1B = bit(ICNC1) | bit(ICES1) | bit(WGM12); // Input Capture Noise Canceler enabled, detecting rising edge, CTC mode
  OCR1A = 59999; // 30000 us.
}

void loop() {
#ifdef LED_PIN
  digitalWrite(LED_PIN, LED_LEVEL);
#endif
  for (uint8_t i = 0; i < ARRAY_SIZE(SONIC_TRIG_PINS); ++i) {
    uint16_t distance = sonicMeasure(pgm_read_byte(&SONIC_TRIG_PINS[i]));

    if (i)
      Serial.print(',');
    if (distance)
      Serial.print(distance);
    else
      Serial.print('-');
  }
  Serial.println(F(" mm."));
#ifdef LED_PIN
  digitalWrite(LED_PIN, ! LED_LEVEL);
#endif
  delay(1000);
}
