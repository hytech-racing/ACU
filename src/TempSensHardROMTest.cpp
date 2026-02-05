//This file is a rewrite of "TempSensDriverTest.cpp" that does not rely on the search algorithm.

#include <Arduino.h>
#include "DS2480B.h"

// Create DS2480B object, passing Serial2 by reference
DS2480B ds(Serial2);

void setup(void) {
  // Begin serial communication
  Serial.begin(115200);
  Serial2.begin(9600);  // DS2480B communicates at 19200 baud
  
  delay(1000);  // Wait for serial to initialize
  Serial.println("Serial Successfully Initialized");
  // ds.begin function resets transceiver and puts it in command mode
  ds.begin();
  ds.reset();
  Serial.println("DS2480B initialized!");
}

void loop(void) {
  byte present = 0;
  byte data[12];
  byte addr[8]{};
  
  int RomIDArr[48] = {
    40,215,97,17,17,0,0,213,  //RomID1 [0-7]
    40,198,156,16,17,0,0,124, //RomID2 [8-15] 
    40,148,57,17,17,0,0,82,   //RomID3 [16-23]
    40,22,55,17,17,0,0,116,   //RomID4 [24-31]
    40,34,54,17,17,0,0,136,   //RomID5 [32-39]
    40,246,127,16,17,0,0,5    //RomID6 [40-47]  
  };

  float celsius, fahrenheit;

  for (int i = 0; i < 6; i++){
    //First lets load the rom ID into addr[]

    for (int j = 0; j < 8 ; j++){
        addr[j] = (byte)(RomIDArr[j + (8 * i)]); //casts into a byte and writes into corresponding addr. Does in reversing order(testing)
        Serial.print((RomIDArr[j + (8 * i)]) + ' ');

    }

    Serial.println(' ');

    Serial.print("ROM =");
    for(int k = 0; k < 8; k++) {
        Serial.write(' ');
        Serial.print(addr[k], DEC);
    }
    delay(1000);

    Serial.println("We are looking at ROMID:");

    delay(1000);
    //Rest of code should be identical from here on

    // The last byte of ROM ID should equal the CRC (derived from first 7 bytes)
    // This checks if the CRCs align
    if (DS2480B::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
    }

    Serial.println();

    // We know the chip is DS18B20, just clarifying through Serial Monitor
    Serial.println("Chip = DS18B20");
    
    // Reset the one-wire bus
    ds.reset();
    
    // Brief delay to allow bus to stabilize
    delay(100);
    
    // Reset again and select the device
    ds.reset();
    ds.select(addr);
    ds.write(0x44);  // Start temperature conversion command

    // Wait for conversion to complete
    // DS18B20 takes ~750ms for 12-bit conversion (default resolution)
    // Adjustable based on your resolution needs:
    // - 93.75ms for 9-bit
    // - 187.5ms for 10-bit
    // - 375ms for 11-bit
    // - 750ms for 12-bit (default)
    delay(1000);

    // Read the temperature data
    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);  // Read Scratchpad command

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");

    // Read 9 bytes of data from the scratchpad
    for(int l = 0; l < 9; l++) {
        data[i] = ds.read();
        Serial.print(data[l], HEX);
        Serial.print(" ");
    }

    Serial.print(" CRC=");
    Serial.print(DS2480B::crc8(data, 8), HEX);
    Serial.println();

    // Convert the data to actual temperature
    // The result is a 16-bit signed integer stored in bytes 0 and 1
    // Byte 0 = LSB (lower 8 bits), Byte 1 = MSB (upper 8 bits)
    int16_t raw = (data[1] << 8) | data[0];

    // DS18B20 always returns 12-bit resolution by default
    // Each LSB = 0.0625°C, so divide by 16
    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;

    Serial.print("  Temperature = ");
    Serial.print(celsius);
    Serial.print(" Celsius, ");
    Serial.print(fahrenheit);
    Serial.println(" Fahrenheit");

    Serial.println(i);
    Serial.println("Going to the next line");
    }   
}