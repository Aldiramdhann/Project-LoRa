#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>  //libraries untuk pengaksesan i2c
#include <Adafruit_BME280.h> //libraries BME280
#include <Adafruit_Sensor.h> 
#define SEALEVELPRESSURE_HPA (1013.25) //nilai awal untuk pressure

Adafruit_BME280 bme; //penggunaan I2C


// Singleton instance of the radio driver
RH_RF95 rf95;
//RH_RF95 rf95(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W
//RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95
// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

// anemometer parameters
volatile byte rpmcount; // count signals
volatile unsigned long last_micros;
unsigned long timeold;
unsigned long timemeasure = 2.00; // seconds
int timetoSleep = 1;               // minutes
unsigned long timeNow;
int countThing = 0;
int GPIO_pulse = 3; // Arduino = D2
float rpm, rps;     // frequencies
float radius = 0.1; // meters - measure of the lenght of each the anemometer wing
float velocity_kmh; // km/h
float velocity_ms;  //m/s
float omega = 0;    // rad/s
float calibration_value = 2.0;


void setup()
{
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init())
    Serial.println("init failed");
    
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  //  driver.setTxPower(14, true);
   if (!bme.begin(0x76)) {
    Serial.println("tidak ada sensor BME280, Coba cek rangkaianmu!");
    while (1);
  }
  pinMode(GPIO_pulse, INPUT_PULLUP);
  digitalWrite(GPIO_pulse, LOW);

  //Serial.begin(9600);

  detachInterrupt(digitalPinToInterrupt(GPIO_pulse));                         // force to initiate Interrupt on zero
  attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); //Initialize the intterrupt pin
  rpmcount = 0;
  rpm = 0;
  timeold = 0;
  timeNow = 0;
  }
void loop()
{
  //Serial.println("Sending to rf95_server");
  
  anemometer();
  

  
  
  
  // Send a message to rf95_server
  //We change the data we want to send with sensor_value
  

  float suhu = bme.readTemperature();
  float kelembaban = bme.readHumidity();
  float tekanan = bme.readPressure() / 100.0F;
  float ketinggian = bme.readAltitude(SEALEVELPRESSURE_HPA); 

 if (isnan(suhu) || isnan(kelembaban) || isnan (tekanan) || isnan (ketinggian)) {
    Serial.println("Failed to read from bme sensor!");
    return;
 }
 
String data = String(suhu) + "#" + String(kelembaban)+ "#" +String(tekanan)+ "#" + String(ketinggian)+ "#" + String(rpm)+ "#" + String(velocity_kmh); 
 

  int dataLength = data.length(); dataLength++;
  uint8_t total[dataLength]; //variable for data to send
  data.toCharArray(total, dataLength); //change type data from string ke uint8_t
  Serial.println(data);
  delay(2000);
  rf95.send(total, dataLength); //send data
  rf95.waitPacketSent();
  delay(500);
}

  void anemometer()
  { 
    if ((millis() - timeold) >= timemeasure * 1000)
  
  {
    countThing++;
    detachInterrupt(digitalPinToInterrupt(GPIO_pulse)); // Disable interrupt when calculating
    rps = float(rpmcount) / float(timemeasure);         // rotations per second
    rpm = 60 * rps;                                     // rotations per minute
    omega = 2 * PI * rps;                               // rad/s
    velocity_ms = omega * radius * calibration_value;   // m/s
    velocity_kmh = velocity_ms * 3.6;                   // km/h
 /* Serial.print("rps=");
    Serial.print(rps);
    Serial.print("   rpm=");
    Serial.print(rpm);
    Serial.print("   velocity_ms=");
    Serial.print(velocity_ms);
    Serial.print("   velocity_kmh=");
    Serial.print(velocity_kmh);*/
    Serial.println("   ");
    if (countThing == 1) // Send data per 25 seconds
    {
      Serial.println("Send data to server");
      countThing = 0;
    }
    timeold = millis();
    rpmcount = 0;
    attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); // enable interrupt
  }
  }

  void rpm_anemometer()
{
  if (long(micros() - last_micros) >= 5000)
  { // time to debounce measures
    rpmcount++;
    last_micros = micros();
  }
  //   Serial.println("***** detect *****");
}
  
  
