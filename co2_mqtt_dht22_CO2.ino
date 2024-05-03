// по умолчанию работаем через локальную wifi cеть , если загрузить с нажатой кнопкой, то работаем через интернет
#include <PubSubClient.h> //Подключаем библиотеку для MQTT
#include <DHT.h> //Подключаем библиотеку для датчика 
#include <ESP8266WiFi.h> //Подключаем библиотеку wifi
#include <SoftwareSerial.h> //Подключаем библиотеку для MHZ19
#include <Adafruit_BME280.h>// Подключаем библиотеку Adafruit_BME280
#include <Adafruit_Sensor.h>   // Подключаем библиотеку Adafruit_Sensor
#include "SH1106Wire.h" //библиотека для мониторчика
#define DHTTYPE DHT22   // DHT 11
#define LED_BUILTIN 2  //встроенный светодиод
#define pin_analog_out 15 //красный светодиод
#define redLevelPPM 1000 //Уровень СО2 для красного светодиода
#define agrigation_hum 2 //усредняем по 2 значениям 
#define agrigation_temp 2 //усредняем по 2 значениям 
#define DHTPin  14 //подклюен датчик DHTPin  14
#define button_pin  16//кнопка для переключения на интрнет. удерживать при включении
//#define SEALEVELPRESSURE_HPA (1013.25) // Давление PHa на уровне моря
Adafruit_BME280 bme;// Установка связи по интерфейсу I2C
//BME280 садим на пины 4,5
SH1106Wire display(0x3c, 4, 5); //параллельно дисплей

int count_hum = 0;
float sum_hum = 0;
float h_old = 0;
float t_old = 0;
static char ppmTemp[7];
float ppm_old = 0;
unsigned int ppm;
float tmhz_old = 0;
float h = 0;
static char humidityTemp[7];
int count_temp = 0;
float sum_temp = 0;
float t = 0;
boolean butt; //состояние кнопки
boolean avtocal = 0; //автокалибровка
boolean displayOn = 1; //вкл выкл мониторчика
//boolean flag_interruptag = 0 ; //для прерывания кнопкой
float davlenie;
float davlenie_old = 0;
static char davlenieTemp[7];
static char temperatureTemp[7];

// для wifi, mqtt и др
const char* ssid = "****"; //для локалки
const char* password = "****";//для локалки
const char* ssid_inet = "Asus";//для интернета, например на мобиле
const char* password_inet = "Asus12345";//для интернета, например на мобиле
const char* mqttUser = "******"; // юзер mqtt брокера
const char* mqttPassword = "******";// пароль mqtt брокера
const char* mqttTopicHumidity = "/ESP_sens1/DHT/HUM";
const char* mqttTopicTemperature = "/ESP_sens1/DHT/TEMP";
const char* mqttTopicCO2 = "/ESP_sens1/DHT/CO2";
const char* mqttTopic_davlenie = "/ESP_sens1/DHT/davlenie";
const char* clientName = "ESP8266_DTH22_spalnya";
char* mqtt_server = "192.138.*.*";//для локалки
char* mqtt_server_inet = "****.ddns.net";////внешний ip или адрес для интернета
char* AEB_txt = "|AEB OFF|"; //Состояние автокаллибровки
char* homeState = "|HOME"; //Через что передаем (домашнюю сеть или инет
char* PPMState = "GOOD|"; //состояние воздуха

// CO2 sensor:
SoftwareSerial mySerial(12, 13); // RX,TX
byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; //запросс со2 с mhz19
unsigned char cmd_off[] = "\xFF\x01\x79\x00\x00\x00\x00\x00\x86"; //ABC logic off
unsigned char cmd_on[] = "\xFF\x01\x79\xA0\x00\x00\x00\x00\xE6";// ABC logic on
unsigned char response[9];

// Timers auxiliar variables
unsigned long now = millis();
unsigned long lastMeasure = 0;
unsigned long resendtime = 60000; //60sec  ПЕРИОД ИЗМЕРЕНИЯ

WiFiClient espClient;
PubSubClient client(espClient);
// Initialize DHT sensor.
DHT dht(DHTPin, DHTTYPE);

void setup() {

  Serial.begin(9600);
  Serial.println("Starting ESP");
  display.init(); //Инициализируем дисплей
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Starting ESP");
  display.display();

  pinMode (button_pin, INPUT_PULLUP);
  pinMode(pin_analog_out, OUTPUT);
  //Если зажата кнопка при включении, то работаем через интрнет
  delay(1000);
  butt = !digitalRead(button_pin);// но кнопку на gnd;
  if (butt == 1 ) {
    ssid = ssid_inet;
    password = password_inet;
    mqtt_server = mqtt_server_inet;
    butt = 0;
    homeState = "|INET";
  }

 //Если зажата кнопка автокалибровки, то вклюаем автокалибровку


  

  dht.begin();
  setup_wifi();
  Serial.print("Connecting to MQTT ");
  display.drawString(0, 32, "Connecting to MQTT: ");
  display.drawString(0, 40, mqtt_server);
  display.display();
  Serial.println(mqtt_server);
  client.setServer(mqtt_server, 1883);
  pinMode(LED_BUILTIN, OUTPUT);
  client.setCallback(callback);
  mySerial.begin(9600);

  // выключаем автоколлибровку CO2
  mySerial.write(cmd_off, 9);
  // mySerial.write(cmd_on, 9);

  if (!bme.begin(0x76))
  { // Проверка инициализации датчика давления
    Serial.println("Could not find a valid BME280 sensor, check wiring!"); // ошибки инициализации.
    display.clear();
    display.drawString(0, 5, "Error of BME280!! ");
    display .display();

  }
}


void loop() {
  butt = !digitalRead(button_pin);// но кнопку на gnd; вкл-выкл автокаллибровки
  if (butt == 1 && displayOn == 0) {
    butt = 0;
    Serial.println("Дисплей ON"); // включаем автокаллибровку кнопкой
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(1, 10 , "Display");
    display.drawString(50, 30 , "ON");
    display.display();
    //   AEB_txt = "|AEB ON|";
    displayOn = 1;
    //    mySerial.write(cmd_on, 9);
    delay (5000);
  }

  if (butt == 1 && displayOn == 1) {
    butt = 0;
    Serial.println("Дисплей OFF"); // выключаем автокаллибровку кнопку
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(1, 10 , "Display");
    display.drawString(50, 30 , "OFF");
    display.display();
    //AEB_txt = "|AEB OFF|";
    displayOn = 0;
    //mySerial.write(cmd_off, 9);
    delay (5000);
    display.clear();
    display.display();
  }

  if (!client.connected()) {
    reconnect();
  }


  if (!client.loop())
    client.connect(clientName);

  now = millis();

  // Publishes new temperature and humidity every N seconds
  if (now - lastMeasure > resendtime) {//интервал измерения
    lastMeasure = now;

    //Влажность
    h  = dht.readHumidity();
    //накапливаем
    Serial.print ("H=");
    Serial.println (h);
    sum_hum = sum_hum + h;
    count_hum = count_hum + 1;

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

    //если накопили значения температуры, усредняем
    if (count_temp >= agrigation_temp) {
      t = sum_temp / count_temp;
      count_temp = 0;
      sum_temp = 0;
      t = ceil (t);
      Serial.print ("T avg=");
      Serial.println (t);


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


    //если накопили значения влажности, усредняем
    if (count_hum >= agrigation_hum) {
      h = sum_hum / count_hum;
      count_hum = 0;
      sum_hum = 0;
      h = ceil (h);
      Serial.print ("H avg=");
      Serial.println (h);


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
    delay(10);
    mySerial.readBytes(response, 9);
    int i;
    byte crc = 0;
    for (i = 1; i < 8; i++) crc += response[i];
    crc = 255 - crc;
    crc++;



    if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {

      display.clear();
      display.drawString(5, 1, "CRC error. ");
      display.drawString(10, 20, "MHZ-19B");
      display.display();
      Serial.println("CRC error: " + String(crc) + " / " + String(response[8]));
      Serial.println("Sensor CRC error");
      while (mySerial.available()) {
        mySerial.read();
      }

    } else
    {
      unsigned int responseHigh = (unsigned int) response[2];
      unsigned int responseLow = (unsigned int) response[3];
      unsigned int temperature = (unsigned int) response[4] - 40;
      ppm = (256 * responseHigh) + responseLow;
      Serial.print("CO2="); Serial.print(ppm); Serial.println(";");
      //если СО2 не гонит, и CO2 изменилось публикуем в mqtt
      if ((ppm > 300) && (ppm < 5000) && ( ppm != ppm_old)) {
        ppm_old = ppm;
        // Serial.print("CO2="); Serial.print(ppm); Serial.println(";");
        digitalWrite(LED_BUILTIN, LOW);
        Serial.print ("ppm Sending  = ");
        Serial.println (ppm);

        dtostrf(ppm, 6, 0, ppmTemp);//0 - количство символов после запятой
        client.publish(mqttTopicCO2, ppmTemp);
        digitalWrite(LED_BUILTIN, HIGH);
        //если высокое значение, включаем красный светодиод
        if (ppm > redLevelPPM)
        {
          digitalWrite(pin_analog_out, 1);
          PPMState = "BAD!!!|";
        }
        //если нормальное значение, выключаем красный светодиод
        if (ppm <= redLevelPPM)
        {
          digitalWrite(pin_analog_out, 0);
          PPMState = "GOOD|";
        }
      }
    }
    //давление
    Serial.print ("PRESSURE= ");
    davlenie = bme.readPressure() / 133, 3;
    davlenie = ceil (davlenie);
    Serial.println(davlenie);

    //фильтруем косячные значенич
    if ((davlenie > 690) && (davlenie < 800) && ( davlenie != davlenie_old)) {
      dtostrf(davlenie, 6, 0, davlenieTemp);//0 - количство символов после запятой
      davlenie_old = davlenie;
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print ("Pressure Sending  = ");
      Serial.println (davlenieTemp);
      client.publish(mqttTopic_davlenie, davlenieTemp);
      digitalWrite(LED_BUILTIN, HIGH);
    }

    //Выводим данные
    display.clear();
    if (displayOn == 1) { //Если флаг монитора включен

      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 0, "********************************");
      display.drawString(0, 9, "T=" + String(temperatureTemp) + "C");
      display.drawString(0, 30, "H=" + String(humidityTemp) + "%");
      display.drawString(50, 9, "P=" + String(davlenieTemp) + "mmHs");
      display.drawString(50, 30, "CO=" + String(ppmTemp) + "ppm");
      display.drawString(0, 40, "********************************");
      display.drawString(40, 50, AEB_txt);
      display.drawString(1, 50, PPMState);
      display.drawString(95, 50, homeState);
      display.display();
    }
  }

}




void setup_wifi() {
  delay(10);
  display.drawString(0, 8, "Connecting to Wifi");
  display.drawString(0, 16, ssid);
  display.display();
  Serial.print("Connecting to WiFi ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("Connected to WiFi");
  display.drawString(0, 24, "Connected to WiFi");
  display.display();
}


void callback(String topic, byte* message, unsigned int length) {
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
}


// This functions reconnects your ESP8266 to your MQTT broker
void reconnect() {
  Serial.println("reconnecting to MQTT ");
  while (!client.connected()) {
    if (client.connect(WiFi.hostname().c_str(), mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT ");
      display.drawString(0, 48, "Connected to MQTT");
      display.display();
    } else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
