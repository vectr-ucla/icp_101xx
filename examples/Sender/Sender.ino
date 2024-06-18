#include <icp101xx.h>
#include <pktStructs.h>

// Sensor is an ICP101xx object
ICP101xx sensor;

vectr::baroPkt baro_pkt;

elapsedMillis current_time, led_time;

int counter = 0;

int led = LED_BUILTIN;
int ledState = HIGH;

void setup() {
  // Initialize sensor.
  // Optional: Pass pointer to Wire to use other than default I2C port.
  //    For example: sensor.begin(&Wire1);
  sensor.begin();
  Serial.begin(115200);

  pinMode(led, OUTPUT);
}

void loop() {
  // Check if sensor is responding.
  if (!sensor.isConnected()) {
    Serial.println("sensor not responding!");
  } else {
    // Start measurement cycle, waiting until it is completed.
    // Optional: Measurement mode
    //    sensor.FAST: ~3ms
    //    sensor.NORMAL: ~7ms (default)
    //    sensor.ACCURATE: ~24ms
    //    sensor.VERY_ACCURATE: ~95ms
    sensor.measure(sensor.VERY_ACCURATE);

    baro_pkt.temp = sensor.getTemperatureC();
    baro_pkt.pres = sensor.getPressurePa();

    // sendBaro();

    // Read and output measured temperature in Celsius and pressure in Pascal.
    Serial.print(sensor.getTemperatureC());
    Serial.print(",");
    Serial.println(sensor.getPressurePa());

    counter++;
  }

  if (led_time >= 1000) {
    led_time = 0;
    ledState = !ledState;
    digitalWrite(led, ledState);
  }

  delay(100);
}

void sendBaro() {
  baro_pkt.header = HEADER;
  baro_pkt.id = PACKETID_BARO;
  baro_pkt.seq = counter;
  baro_pkt.timestamp = current_time;
  baro_pkt.checksum = 0;

  uint8_t data[sizeof(baro_pkt)];
  memcpy(data, &baro_pkt, sizeof(baro_pkt));

  uint8_t checksum = calcChecksum(data, sizeof(baro_pkt));
  data[sizeof(baro_pkt) - 1] = checksum;

  Serial.write(data, sizeof(data));
}

char calcChecksum(uint8_t data[], uint8_t len) {
  char checksum = 0;
  for (int i = 0; i < len - 1; i++) {
    checksum += data[i];
  }
  return checksum;
}
