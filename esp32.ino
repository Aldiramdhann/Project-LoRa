
#include <ArduinoJson.h>
//iclude wifi

#include "WiFi.h"
#include "HTTPClient.h"


//variabel wifi dan pw
const char* ssid = "TachTA";
const char* pass = "tentara098";


//variabel host/ server yg menampung aplikasi web
const char* host = "sensorlora.000webhostapp.com";


//Timer to run Arduino code every 5 seconds
//unsigned long previousMillis = 0;
//unsigned long currentMillis;
//const unsigned long period = 10000;

void setup() {
  // Initialize Serial port
  Serial.begin(115200);
  //Serial2.begin(9600);
  while (!Serial) continue;

  //koneksi ke wifi

  WiFi.begin(ssid, pass);
  Serial.println("Conecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  //apabila connect
  Serial.println("Connected");

}
void loop() {
  //Get current time
  //currentMillis = millis();

  //if ((currentMillis - previousMillis >= period)
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, Serial);

  // Test parsing
  if (error) {
    Serial.println("Invalid JSON Object");
    delay(500);
    return;
  }


  Serial.println("JSON Object Recieved");
  Serial.print("Recieved Suhu:  ");
  float suhu = doc["suhu"];
  Serial.println(suhu);

  Serial.print("Recieved Kelembaban :  ");
  float kelembaban = doc["kelembaban"];
  Serial.println(kelembaban);

  Serial.print("Recieved tekanan:  ");
  float tekanan = doc["tekanan"];
  Serial.println(tekanan);

  Serial.print("Recieved ketinggian:  ");
  float ketinggian = doc["ketinggian"];
  Serial.println(ketinggian);

  Serial.print("Recieved rps:  ");
  float rps = doc["rps"];
  Serial.println(rps);

  Serial.print("Recieved rpm:  ");
  float rpm = doc["rpm"];
  Serial.println(rpm);

  Serial.print("Recieved velocity_ms :  ");
  float velocity_ms = doc["velocity_ms"];
  Serial.println(velocity_ms);

  Serial.print("Recieved velocity_kmh :  ");
  float velocity_kmh = doc["velocity_kmh"];
  Serial.println(velocity_kmh);
  Serial.println("-----------------------------------------");
  //kirim data ke server

  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort))
  {
    Serial.println("Connection Failed");
    return;
  }
  //pasti terkoneksi
  //kirim datasensor ke database / web
  String Link ;
  HTTPClient http;

  Link = "http://" + String(host) + "/kirimdata.php?suhu=" + String(suhu) + "&kelembaban="
         + String(kelembaban) + "&tekanan=" + String(tekanan) + "&ketinggian=" + String(ketinggian) + "&rps=" + String(rps)
         + "&rpm=" + String(rpm) + "&velocity_ms=" + String(velocity_ms) + "&velocity_kmh=" + String(velocity_kmh);
  //eksekusi alamat link
  http.begin(Link);
  http.GET();

  //baca respon setelah berhasil kirim nilai sensor
  String respon = http.getString();
  Serial.println(respon);
  http.end();
  delay(2000);
}
//previousMillis = previousMillis + period;
