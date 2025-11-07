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