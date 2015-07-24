/* DHT library 

MIT license
written by Adafruit Industries
*/

#include "DHT.h"

#define MIN_INTERVAL 2000

DHT::DHT(uint8_t pin, uint8_t type, uint8_t) {
  _pin = pin;
  _type = type;
}

void DHT::begin(void) {
  // set up the pins!
  pinMode(_pin, INPUT_PULLUP);
  // Using this value makes sure that millis() - lastreadtime will be
  // >= MIN_INTERVAL right away. Note that this assignment wraps around,
  // but so will the subtraction.
  _lastreadtime = -MIN_INTERVAL;
}

//boolean S == Scale.  True == Farenheit; False == Celcius
float DHT::readTemperature(bool S, bool force) {
  float f;

  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if(S)
      	f = convertCtoF(f);
      	
      return f;
    case DHT22:
    case DHT21:
      f = data[2] & 0x7F;
      f *= 256;
      f += data[3];
      f /= 10;
      if (data[2] & 0x80)
	f *= -1;
      if(S)
	f = convertCtoF(f);

      return f;
    }
  }
  return NAN;
}

float DHT::convertCtoF(float c) {
	return c * 9 / 5 + 32;
}

float DHT::convertFtoC(float f) {
  return (f - 32) * 5 / 9; 
}

float DHT::readHumidity(bool force) {
  float f;
  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[0];
      return f;
    case DHT22:
    case DHT21:
      f = data[0];
      f *= 256;
      f += data[1];
      f /= 10;
      return f;
    }
  }
  return NAN;
}

float DHT::computeHeatIndex(float tempFahrenheit, float percentHumidity) {
  // Adapted from equation at: https://github.com/adafruit/DHT-sensor-library/issues/9 and
  // Wikipedia: http://en.wikipedia.org/wiki/Heat_index
  return -42.379 + 
           2.04901523 * tempFahrenheit + 
          10.14333127 * percentHumidity +
          -0.22475541 * tempFahrenheit*percentHumidity +
          -0.00683783 * pow(tempFahrenheit, 2) +
          -0.05481717 * pow(percentHumidity, 2) + 
           0.00122874 * pow(tempFahrenheit, 2) * percentHumidity + 
           0.00085282 * tempFahrenheit*pow(percentHumidity, 2) +
          -0.00000199 * pow(tempFahrenheit, 2) * pow(percentHumidity, 2);
}


boolean DHT::read(bool force) {
  unsigned long currenttime;

  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < MIN_INTERVAL)) {
    return true; // return last correct measurement
    //delay(2000 - (currenttime - _lastreadtime));
  }
  _lastreadtime = currenttime;

  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  
  // pull the pin high and wait 250 milliseconds
  digitalWrite(_pin, HIGH);
  delay(250);

  // now pull it low for ~20 milliseconds
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);
  noInterrupts();
  pinMode(_pin, INPUT_PULLUP);
  delayMicroseconds(40);

  // Sensor pulls low and high for 80uS each (so timeout should be >
  // 80uS + 80uS)
  if (!pulseIn(_pin, HIGH, /* timeout */ 200)) {
    interrupts();
    return false; // timeout
  }

  // Now, the sensor pullse low for 50us and high for 26-28uS (0) or
  // 70uS (1) (values for DHT22, others are similar).  Below loop
  // measures the length of the high pulse and uses 50uS as a cutoff
  // value.
  for (uint8_t i = 0; i < sizeof(data) * 8; i++) {
    // Timeout should be > 50uS + 70uS
    uint8_t length = pulseIn(_pin, HIGH, 150);

    if (!length) {
      interrupts();
      return false; // timeout
    }

    data[i/8] <<= 1;
    if (length > 50)
      data[i/8] |= 1;
  }

  interrupts();
  
  /*
  Serial.print(data[0], HEX); Serial.print(", ");
  Serial.print(data[1], HEX); Serial.print(", ");
  Serial.print(data[2], HEX); Serial.print(", ");
  Serial.print(data[3], HEX); Serial.print(", ");
  Serial.print(data[4], HEX); Serial.print(" =? ");
  Serial.println(data[0] + data[1] + data[2] + data[3], HEX);
  */

  // check that the checksum matches
  uint8_t checksum = (data[0] + data[1] + data[2] + data[3]);
  return (data[4] == checksum);

}
