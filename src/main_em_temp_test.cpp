#include <DS18B20.h>

DS18B20 ds(2);

void setup() {
  Serial.begin(115200);
  Serial.print("Devices: ");
  Serial.println(ds.getNumberOfDevices());
  Serial.println();
}

void loop() {
  while (ds.selectNext()) {
    Serial.println(ds.getTempC());
  }
  Serial.print("Resolution: ");
  Serial.println(ds.getResolution());

  Serial.print("Power Mode: ");
  if (ds.getPowerMode()) {
    Serial.println("External");
  } else {
    Serial.println("Parasite");
  }
}

// #include <DS18B20.h>

// #define LOW_ALARM 20
// #define HIGH_ALARM 25

// DS18B20 ds(2);
// uint8_t address[] = {40, 250, 31, 218, 4, 0, 0, 52};
// uint8_t selected;

// void setup() {
//   Serial.begin(9600);
//   selected = ds.select(address);

//   if (selected) {
//     ds.setAlarms(LOW_ALARM, HIGH_ALARM);
//   } else {
//     Serial.println("Device not found!");
//   }
// }

// void loop() {
//   if (selected) {
//     if (ds.hasAlarm()) {
//       Serial.print("Warning! Temperature is ");
//       Serial.print(ds.getTempC());
//       Serial.println(" C");
//     }
//   } else {
//     Serial.println("Device not found!");
//   }

//   delay(10000);
// }