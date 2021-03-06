/* https://lastminuteengineers.com/bme280-esp32-weather-station/
  Parece que el ESP32 se cuelga después de varios días con el BME280 encendido permanentemente. No es tal cuelgue, sino que el sistema se queda intentando conectar por wifi indefinidamente. Parece solucionado con un reset si no se conecta en 50 intentos.
  SPEED --> GPIO25 Interrupción a flanco de bajada (Con PULL-UP)
  MOSFET (INTERRUPTOR) --> GPIO23
  SDA --> GPIO21
  SCL (SCK) --> GPIO22
  PLUVIOMETRO --> GPIO27 (Pull-Down externo)
  Anemómetro --> GPIO25 v=2,25*pulsos/Tiempo(s) (mph) = 3,62*pulsos/Tiempo(s) (km/h)
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
#define DIVISORTENSION 2
#define ANEMOMETRO 25
#define ANTIRREBOTE 15
//#define PROBANDO
#if defined(PROBANDO)
  #define INTERVALO 20  // Segundos
#else
  #define INTERVALO 1800
#endif

Adafruit_BME280 bme; // I2C : D21 --> SDA  D22 --> SCL
Adafruit_ADS1115 ads; // I2C Dir: 0x48
WiFiMulti wifiMulti;

volatile unsigned long pulsos=0;
volatile unsigned long tiempoRebote=0;
volatile unsigned long tiempoPrimerPulso=0;

String serverName = "https://api.thingspeak.com/update?api_key=ZNPZIIHCRQ2P4VIX";

void setup() {
  apagaWiFi();
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER,HIGH);
  Serial.begin(115200);
  ads.begin();
  ads.setGain(GAIN_ONE);
  pinMode(ANEMOMETRO, INPUT);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETRO), isr_pulso, FALLING);
  Serial.println("Estación meteorolóxica do IES Politécnico de Vigo");
  if (!bme.begin()) bmeError();
    else Serial.print("BME 280 - SensorID : 0x"); Serial.println(bme.sensorID(),16);
  configura();
  bme.takeForcedMeasurement();
  float temperatura = bme.readTemperature();
  float pre = bme.readPressure() / 100.0F;
  float presion = bme.seaLevelForAltitude(ELEVACION, pre);
  float humedad=bme.readHumidity();
  float vBat=mideTension();
  int dir= dirViento();
  float vel=0;
 
  enciendeWiFi();  
  //WiFi.begin(ssid, password); // Attempt to connect to wifi with our password
  byte notConnectedCounter = 1;
  wifiMulti.addAP("COCO", "12345678");
  wifiMulti.addAP("AJOIR", "riquilante");
  wifiMulti.addAP("AULA27", "");
  wifiMulti.addAP("AULA26", "");
  wifiMulti.addAP("PARANINFO", "p0l1t3cn1c0");
  wifiMulti.addAP("Sala Profes", "salapr0f3s");
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
  
  if (millis()<6000) delay(4000); // Damos tiempo para medir la velocidad del viento.
  if (pulsos>0) {
    vel = round(36.2 * 1000.0 * 10.0 * (pulsos * 1.0/((millis()-tiempoPrimerPulso)*1.0)))/10.0; // Redondeado a una cifra decimal
  }
  Serial.println("Temperatura(ºC)\tPresión(hPa)\tHumedad\tV. Batería\tDirección\tVelocidad");  
  Serial.print(temperatura);Serial.print("\t");
  Serial.print(presion);Serial.print("\t");
  Serial.print(humedad);Serial.print("\t");
  Serial.print(vBat);Serial.print("\t");
  Serial.print(dir);Serial.print("\t");
  Serial.println(vel); 
  if(WiFi.status()== WL_CONNECTED){ // Check to make sure wifi is still connected
    Serial.println("Enviando datos por Wifi");
    subeDatos(temperatura,presion,humedad,vBat,dir,vel); 
  }
  else Serial.println("Wifi Desconectada");  
  detachInterrupt(digitalPinToInterrupt(ANEMOMETRO));
  apagaWiFi();
  digitalWrite(POWER,LOW);
  pinMode(POWER,INPUT);
  Serial.print("Tiempo empleado (ms) = "); Serial.println(millis()); 
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

void subeDatos(double temp, double pres, double hum, double vbat,int dir,double vel){
  HTTPClient http; // Initialize our HTTP client
  String url = serverName + "&field1=" + temp + "&field2=" + pres + "&field3=" + hum + "&field4=" + vbat + "&field5=" + dir + "&field5=" + vel; 
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
                      Adafruit_BME280::FILTER_OFF);
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
  for (i=1;i<=20;i++) {
    acumulado+= ads.readADC_SingleEnded(0);
    delay(10);  
    }
  return(DIVISORTENSION*ads.computeVolts(acumulado/i));  
}
float dirViento(){
  int acumulado = 0;
  int i=1;
  for (i=1;i<=20;i++) {
    acumulado+= ads.readADC_SingleEnded(2);
    delay(10);  
    }
  return(map((acumulado/i),0,32767,0,359));  
}
// Interrupción activada por pulso descendente en el hilo SPEED del anemómetro
void isr_pulso() {
  if((millis() - tiempoRebote) > ANTIRREBOTE ) { // 15 milisegundos mínimo antirrebote.
    if (pulsos ==0) tiempoPrimerPulso=millis();
    pulsos++;
    tiempoRebote = millis();
  }
} 
