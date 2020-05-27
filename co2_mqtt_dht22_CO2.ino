#include <PubSubClient.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

// Uncomment one of the lines bellow for whatever DHT sensor type you're using!
#define DHTTYPE DHT22   // DHT 11
#define LED_BUILTIN 2
#define agrigation_hum 2 //усредняем по 2 значениям 
#define agrigation_temp 2 //усредняем по 2 значениям 
int count_hum = 0;
float sum_hum = 0;
float h_old = 0;
float t_old = 0;
float ppm_old = 0;
float tmhz_old = 0;
float h = 0;
int count_temp = 0;
float sum_temp = 0;
float t = 0;

// CO2 sensor:
SoftwareSerial mySerial(12, 13); // RX,TX
byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
unsigned char response[9];

// Change the credentials below, so your ESP8266 connects to your router
const char* ssid = "имя wifi";
const char* password = "ПАРОЛЬ WiFi";
const char* mqttUser = "логин MQTT";
const char* mqttPassword = "Пароль MQTT";
const char* mqttTopicHumidity = "/ESP_sens1/DHT/HUM";
const char* mqttTopicTemperature = "/ESP_sens1/DHT/TEMP";
const char* mqttTopicCO2 = "/ESP_sens1/DHT/CO2";
const char* mqttTopicmhz_temp = "/ESP_sens1/DHT/mhz_temp";
const char* clientName = "ESP8266_CO2_DHT_spalnya";
const char* mqtt_server = "192.168.1.11"; //адрес mqtt брокера

// DHT Sensor - GPIO 5 = D1 on ESP-12E NodeMCU board
const int DHTPin = 5;


// Timers auxiliar variables
unsigned long now = millis();
unsigned long lastMeasure = 0;
unsigned long resendtime = 60000; //60sec  ПЕРИОД ИЗМЕРЕНИЯ

WiFiClient espClient;
PubSubClient client(espClient);
// Initialize DHT sensor.
DHT dht(DHTPin, DHTTYPE);


void setup() {
  dht.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  pinMode(LED_BUILTIN, OUTPUT);
  client.setCallback(callback);
  Serial.begin(9600);
  mySerial.begin(9600);
}



void loop() {

  if (!client.connected()) {
    reconnect();
  }
  
  //dont know what this string does, but without it lamp does not work
  if (!client.loop())
    client.connect(clientName);

  now = millis();

  // Publishes new temperature and humidity every 30 seconds
  if (now - lastMeasure > resendtime) {
    lastMeasure = now;
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    //Влажность
    h  = dht.readHumidity();
    //накапливаем
    Serial.print ("H=");
    Serial.println (h);
    sum_hum = sum_hum + h;
    count_hum = count_hum + 1;

    // Read temperature as Celsius (the default)
    //Температура
    t = dht.readTemperature();
    //накапливаем
    sum_temp = sum_temp + t;
    count_temp = count_temp + 1;
    Serial.print ("T=");
    Serial.println (t);


    // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
      return;
    }

    //индекс температуры
    // Computes temperature values in Celsius
   
    //если накопили значения,усредняем
    if (count_temp >= agrigation_temp) {
                              t = sum_temp / count_temp;
                              count_temp = 0;
                              sum_temp = 0;
                              t = ceil (t);
                              Serial.print ("T avg=");
                              Serial.println (t);
                              static char temperatureTemp[7];
                                               
                        
                              //если знаение изменилось, отправляем
                              if (t != t_old) {
                              dtostrf(t, 6, 0, temperatureTemp); //0 - количство символов после запятой
                              digitalWrite(LED_BUILTIN, LOW);
                              t_old = t;
                              Serial.print ("T Sending  = ");
                              Serial.println (temperatureTemp);
                              // Publishes Temperature and
                              client.publish(mqttTopicTemperature, temperatureTemp);
                              digitalWrite(LED_BUILTIN, HIGH);
                              }
                              t = 0;

            }
 


    //если накопили значения,усредняем
    if (count_hum >= agrigation_hum) {
      h = sum_hum / count_hum;
                          count_hum = 0;
                          sum_hum = 0;
                          h = ceil (h);
                          Serial.print ("H avg=");
                          Serial.println (h);
                          static char humidityTemp[7];
                    
                          // Publishes  Humidity values
                          //если знаение изменилось, отправляем
                          if (h != h_old) {
                          dtostrf(h, 6, 0, humidityTemp);//0 - количство символов после запятой
                          h_old = h;
                          digitalWrite(LED_BUILTIN, LOW);
                          Serial.print ("H Sending  = ");
                          Serial.println (humidityTemp);
                          client.publish(mqttTopicHumidity, humidityTemp);
                          digitalWrite(LED_BUILTIN, HIGH);
                          }
                          h = 0;

    }



//Займемся СО2
     mySerial.write(cmd, 9);
      memset(response, 0, 9);
      mySerial.readBytes(response, 9);
      int i;
      byte crc = 0;
      for (i = 1; i < 8; i++) crc += response[i];
      crc = 255 - crc;
      crc++;

      

  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error: " + String(crc) + " / " + String(response[8]));
    Serial.println("Sensor CRC error");
  } else {
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    unsigned int temperature = (unsigned int) response[4] - 40;
    unsigned int ppm = (256 * responseHigh) + responseLow;
    Serial.print("T om MHZ16b=");Serial.println(temperature);
    //если температура не гонит и температура измнилась, публикуем в mqtt
    if ((temperature < 200)&&( temperature != tmhz_old)) {
      tmhz_old = temperature;
      Serial.print("T="); Serial.print(temperature);
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print ("T_MHZ19 Sending  = ");
      Serial.println (temperature);
       static char MhzTemp[7];
       dtostrf(temperature, 6, 0, MhzTemp);//0 - количство символов после запятой
      client.publish(mqttTopicmhz_temp, MhzTemp);
      digitalWrite(LED_BUILTIN, HIGH);

      
    }
    ////если СО2 не гонит, и CO2 изменилось публикуем в mqtt
    if ((ppm > 300) && (ppm < 5000)&&( ppm != ppm_old)) {
      ppm_old = ppm;
      Serial.print("CO2="); Serial.print(ppm); Serial.println(";");
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print ("ppm Sending  = ");
      Serial.println (ppm);
      static char ppmTemp[7];
      dtostrf(ppm, 6, 0, ppmTemp);//0 - количство символов после запятой
      client.publish(mqttTopicCO2, ppmTemp);
      digitalWrite(LED_BUILTIN, HIGH);
      
      }
    }
  }
}




void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  }
}


void callback(String topic, byte* message, unsigned int length) {
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
}


// This functions reconnects your ESP8266 to your MQTT broker
void reconnect() {
   while (!client.connected()) {
      if (client.connect(WiFi.hostname().c_str(), mqttUser, mqttPassword)) {
        } else {
      // Wait 5 seconds before retrying
         delay(5000);
    }
  }
}
