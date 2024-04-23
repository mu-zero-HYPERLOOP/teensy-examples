#include <SPI.h>
#include <SparkFunADXL313.h> //Click here to get the library: http://librarymanager/All#SparkFun_ADXL313
ADXL313 myAdxl;

void setup()
{
  Serial.begin(115200);
  Serial.println("Example 8 - Reading values from ADXL313 over SPI");

  if (myAdxl.beginSPI(10) == false) //Begin communication over SPI with CS pin of 10. note, beginSPI() returns the result of checkPartID().
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
  Serial.print("Sensor is connected properly.");
  
  myAdxl.measureModeOn(); // wakes up the sensor from standby and puts it into measurement mode
}

void loop()
{
  if(myAdxl.dataReady()) // check data ready interrupt, note, this clears all other int bits in INT_SOURCE reg
  {
    myAdxl.readAccel(); // read all 3 axis, they are stored in class variables: myAdxl.x, myAdxl.y and myAdxl.z
    Serial.print("x: ");
    Serial.print(myAdxl.x);
    Serial.print("\ty: ");
    Serial.print(myAdxl.y);
    Serial.print("\tz: ");
    Serial.print(myAdxl.z);
    Serial.println();
  }
  else
  {
    Serial.println("Waiting for dataReady.");
  }  
  delay(50);
}