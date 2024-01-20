#include <TI_HDC10XX.h>

//TI_HDC1000 dhtSensor;
//TI_HDC1010 dhtSensor;
//TI_HDC1050 dhtSensor;
TI_HDC1080 dhtSensor;
bool sensorInitialized = false;

void setup()
{
  Serial.begin(115200);
  while(!Serial);
  
  Wire.begin();
  Wire.setTimeout(10000); // [µs]
  dhtSensor.setBlocking(true);
  if (dhtSensor.reset())
  {
    dhtSensor.setResolution(11, 11);

    bool voltageOK = dhtSensor.isSupplyVoltageOK();
    Serial.println(voltageOK? "sensor supply voltage OK" : "sensor supply voltage LOW");

    uint32_t serial = dhtSensor.readSerialIdLow();
    Serial.print("sensor SNR: ");
    Serial.println(serial);
    
    Serial.print("heating sensor for 60 s ...");
    for (int i=0; i<60; i++)
    {
      dhtSensor.heat(1000);
      Serial.print(".");
    }
    Serial.println(" done.");

    sensorInitialized = true;
  }
  else
  {
    Serial.println("ERROR: sensor reset failed");
  }
}

void loop()
{
  if (sensorInitialized)
  {
    uint32_t start = millis();
    if (dhtSensor.startAcquisition(dhtSensor.ACQ_TYPE_COMBINED))
    {
      uint32_t stop = millis();
      float temperature = dhtSensor.getTemperature();
      float humidity = dhtSensor.getHumidity();
      Serial.print(temperature);
      Serial.print(" °C ");
      Serial.print(humidity);
      Serial.print(" %RH ");
      Serial.print(stop - start);
      Serial.println(" ms");
    }
    else
    {
      Serial.println("ERROR: sensor not responding");
    }
  }
  else
  {
    Serial.println("sensor not available");
  }
  
  delay(3000);
}