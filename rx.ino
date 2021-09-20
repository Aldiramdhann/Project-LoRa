#include <SPI.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>



SoftwareSerial linkSerial(5, 6); // RX, TX
// Singleton instance of the radio driver
RH_RF95 rf95;

unsigned long int millisBefore;

float suhu, kelembaban, tekanan, ketinggian, rps, rpm, velocity_ms, velocity_kmh;

//variabel array untul parsing data

String arrData[6];
void setup()
{

  // pinMode(led, OUTPUT);
  Serial.begin(9600);
  linkSerial.begin(115200);
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init())
    Serial.println("init failed");



}
void loop()
{
  // Should be a message for us now
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  String data = "";
  if (rf95.waitAvailableTimeout(500))
  {

    while (rf95.recv(buf, &len))
      //{
      // StaticJsonDocument <1000> doc;
      // Serial.print("Received at Server: ");
      //Serial.println((char*)buf);
      //  Serial.print("RSSI: ");
      //  Serial.print(rf95.lastRssi(), DEC);
      //  Serial.println(" dBm");
      //   Serial.print("SNR: ");
      //  Serial.print(rf95.lastSNR(), DEC);
      //  Serial.println(" dB");
      data = (char*)buf;
  }

  //uji data
  if (data != "")
  {
    //parsing data(pecahdata)

    int index = 0;
    for (int i = 0; i <= data.length(); i++)
    {
      char delimiter = '#';
      if (data[i] != delimiter)
        arrData[index] += data[i];

      else
        index++; //variabel index bertambah 1
    }



    //pastikan data yg dkirim lengkap

    if (index == 5)
    {


      // tampilkan nilai sensor ke serial monitor
      Serial.println("data terkirim full " );
      Serial.println("Suhu = " + arrData[0]);
      Serial.println("Kelembaban ="  + arrData[1]);
      Serial.println("Pressure = " + arrData[2]);
      Serial.println("Approxe altitude = " + arrData[3]);
      Serial.println("rpm = " + arrData[4]);
      Serial.println("velocity_kmh = " + arrData[5]);
      Serial.println("--------------");
      ////delay(2000);

      StaticJsonDocument<100> doc;
      suhu = arrData[0].toFloat();
      kelembaban = arrData[1].toFloat();
      tekanan = arrData[2].toFloat();
      ketinggian = arrData[3].toFloat();
      rpm = arrData[4].toFloat();
      velocity_kmh = arrData[5].toFloat();


      doc["suhu"] = suhu;
      doc["kelembaban"] = kelembaban;
      doc["tekanan"] = tekanan;
      doc["ketinggian"] = ketinggian;
      doc["rpm"] = rpm;
      doc["velocity_kmh"] = velocity_kmh;

      //Send data to NodeMCU
      serializeJson(doc, linkSerial);
     // serializeJson(doc, Serial);
      Serial.println();
      //delay(500);

    }

    //     kosongkan data

    arrData[0] = "";
    arrData[1] = "";
    arrData[2] = "";
    arrData[3] = "";

    arrData[4] = "";
    arrData[5] = "";




  }
}
