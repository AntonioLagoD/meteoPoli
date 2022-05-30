/* https://lastminuteengineers.com/bme280-esp32-weather-station/
  Parece que el ESP32 se cuelga después de varios días con el BME280 encendido permanentemente. No es tal cuelgue, sino que el sistema se queda intentando conectar por wifi indefinidamente. Parece solucionado con un reset si no se conecta en 50 intentos.
  SPEED --> GPIO25 Interrupción a flanco de bajada (Con PULL-UP)
  MOSFET (INTERRUPTOR) --> GPIO23
  SDA --> GPIO21
  SCL (SCK) --> GPIO22
  PLUVIOMETRO --> GPIO27 (Pull-Down externo)
*/
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <esp_sleep.h>
#include <esp_adc_cal.h>

#define POWER 23
#define ELEVACION 25.0
#define VBATPIN 36
#define DIVISORTENSION 2
//#define MULTVOLT 0.125F
//#define PROBANDO
#if defined(PROBANDO)
  #define INTERVALO 20  // Segundos
#else
  #define INTERVALO 1800
#endif

Adafruit_BME280 bme; // I2C : D21 --> SDA  D22 --> SCL
Adafruit_ADS1115 ads; // I2C Dir: 0x48
WiFiMulti wifiMulti;
// Set our wifi name and password
//const char* ssid = "AJOIR";
//const char* password = "riquilante";

//const char* ssid = "AULA27";
//const char* password = "";

String serverName = "https://api.thingspeak.com/update?api_key=ZNPZIIHCRQ2P4VIX";

void setup() {
  //delay(500); // Probando si esto evita el cuelgue
  apagaWiFi();
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER,HIGH);
  Serial.begin(115200);
  ads.begin();
  ads.setGain(GAIN_ONE);    
  if (!bme.begin()) bmeError();
  else Serial.print("BME 280 - SensorID : 0x"); Serial.println(bme.sensorID(),16);
  configura();
  bme.takeForcedMeasurement();
  float temperatura = bme.readTemperature();
  float pre = bme.readPressure() / 100.0F;
  float presion = bme.seaLevelForAltitude(ELEVACION, pre);
  float humedad=bme.readHumidity();
  float vBat=mideTension(); 
  Serial.println(temperatura);
  Serial.println(presion);
  Serial.println(humedad);
  Serial.println(vBat); 
  enciendeWiFi();  
  //WiFi.begin(ssid, password); // Attempt to connect to wifi with our password
  byte notConnectedCounter = 1;
  wifiMulti.addAP("COCO", "12345678");
  wifiMulti.addAP("AJOIR", "riquilante");
  wifiMulti.addAP("AULA27", "");
  wifiMulti.addAP("AULA26", "");
  //wifiMulti.addAP("AJOIRr", "riquilante");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print("Conectando a la wifi. Intento Nº : ");
    Serial.println(notConnectedCounter);
    delay(500);    
    notConnectedCounter++;
    if(notConnectedCounter > 50) { // Resetear placa a los 50 intentos
        Serial.println("Reinicio debido a la imposibilidad de conectar a la wifi.");
        ESP.restart();
    }
  } 
  Serial.println("");  Serial.println("WiFi connected: ");  Serial.println(WiFi.SSID());  Serial.println("IP address: ");  Serial.println(WiFi.localIP());

  if(WiFi.status()== WL_CONNECTED){ // Check to make sure wifi is still connected
    Serial.println("Enviando datos por Wifi");
    sendData(temperatura,presion,humedad,vBat); // Call the sendData function defined below
  }
  else Serial.println("Wifi Desconectada");  

  apagaWiFi();
  digitalWrite(POWER,LOW);
  pinMode(POWER,INPUT);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(INTERVALO * 1000000); 
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);  
  esp_deep_sleep_start();
}
void loop() {   
         // Vacío. El código se ejecuta en Setup()  
}

void bmeError()
{
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"); Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n"); Serial.print("        ID of 0x60 represents a BME 280.\n");  Serial.print("        ID of 0x61 represents a BME 680.\n");    
}

void sendData(double temp, double pres, double hum, double vbat){
  HTTPClient http; // Initialize our HTTP client
  String url = serverName + "&field1=" + temp + "&field2=" + pres + "&field3=" + hum + "&field4=" + vbat; // Define our entire url
  http.begin(url.c_str()); // Initialize our HTTP request
  int httpResponseCode = http.GET(); // Send HTTP request
  Serial.print("Respuesta HTTP: ");
  Serial.println(httpResponseCode);
  http.end();
}

void configura() {
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
}
void apagaWiFi(){
    WiFi.disconnect(true);  // Desconecta de la red
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
}
void enciendeWiFi(){
    WiFi.disconnect(false);  // Reconnect the network
    WiFi.mode(WIFI_STA);
}
float mideTension(){
  int acumulado = 0;
  int i=1;
  for (i=1;i<=50;i++) {
    acumulado+= ads.readADC_SingleEnded(0);
    delay(10);  
    }
  return(DIVISORTENSION*ads.computeVolts(acumulado/i));  
}
