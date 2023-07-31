//1 - Libraries

#include <Arduino.h>


#include <Adafruit_Sensor.h> 
#include <DHT.h> 

//#include "ArduinoJson.h"
//#include "PubSubClient.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "time.h"

//----------------------------------------------


//2 - Definição dos pinos
const byte ALARM_TEMP_PIN = 15;                                 // pino 03 (GPIO_15)
const byte ALARM_HUMI_PIN = 33;                                 // pino 09 (GPIO_33)
const byte ALARM_OPEN_DOOR_PIN = 25;                            // pino 08 (GPIO_25)
//const byte DHT_PIN = 32;               
const byte DOOR_SWITCH_PIN = 34;                                // pino 12 (GPIO_34)

const char* ssid = "ESANTOS";                                       // Entre SSID aqui
const char* password = "250608vi";                         // Insira a senha aqui


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define LED_2 2                                             // Pino 04 (GPIO_02) aciona led azul on board
#define LED_23 23                                           // Pino 15 (GPIO_23) aciona led verm off board
#define LED_19 19                                           // Pino 10 (GPIO_19) aciona led azul off board para indicar sem WiFi
#define DHT_PIN 32                                           // Pino 10-B (GPIO-32) Digital pin connected DHT pin 32 (D32) ADC01 (ñ use ADC02 x WiFi)
#define DHTTYPE    DHT11                                    // DHT 11


//-----------------------------------------


// 3 - Definições do MQTT
const char *ID_MQTT = "Humity";
const char *MQTT_USER = "carlos.carneiro@seizerteck.com.br";
const char *MQTT_PASS = "rhwh35y6ell8wn3d";
const char *MQTT_TOPIC = "carlos.carneiro_Ee0Aqj6p/Humity";
const char *HUMI_TOPIC = "/humi";
const char *TEMP_TOPIC = "/temp";


// 3.1 - Informações do MQTT
const char *BROKER_MQTT = "beenode.io";
const uint16_t BROKER_PORT = 1883;

// 4 - Variáveis das medidas
bool door_open = 0;
float h = 0;
float t = 0;

//---------------------------------------------

// 5 - Global Variables


//Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);



DHT dht(DHT_PIN, DHTTYPE);                            // poderia usar: DHT dht(DHT_PIN, DHT11); em L.35: DHTTYPE = DHT11 . pino 32 lendo o DHT11
WiFiClient wifiClient;
PubSubClient MQTT(wifiClient);



// 5.1 - WiFi

void Connectwifi(){

  Serial.printf("Connecting to %s ");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("....");
  
  }
  
  Serial.println("WiFi conectado ..!");
  Serial.print("IP obtido: "); 
  Serial.println(WiFi.localIP());
  digitalWrite(LED_19, LOW);
  digitalWrite(LED_23, HIGH);                            // Pino 15 (GPIO_23) aciona led verm off board
  delay(2000);
  digitalWrite(LED_23, LOW);

  while (WiFi.status() == WL_CONNECTED) 
  {
    delay(2000);
    digitalWrite(LED_23, HIGH);                   // LED Liga
  }      

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  digitalWrite(LED_19, HIGH);                   // Pino 10 (GPIO_19) aciona led azul off board para indicar sem WiFi
  delay(1500);
} 
   
  // fim connect to WiFi
//------------------------------------------------

//5.2 - Setups


void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(DHT_PIN, INPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_23, OUTPUT);  
  pinMode(LED_19, OUTPUT);
  //dht.begin();                                    // Solicita que inicie a função dht existente na biblioteca DHT
 
  // Pinos de entrada e saída
  pinMode(ALARM_TEMP_PIN, OUTPUT);
  pinMode(ALARM_HUMI_PIN, OUTPUT);
  pinMode(ALARM_OPEN_DOOR_PIN, OUTPUT);
  pinMode(DHT_PIN, INPUT);
  pinMode(DOOR_SWITCH_PIN, INPUT_PULLDOWN);

}


// 5.3 - MQTT


void MQTTconnect() {
  Serial.println("Conectando ao broker ");
  Serial.print(BROKER_MQTT);
  Serial.print(":");
  Serial.print(BROKER_PORT);
  Serial.print("...");
    
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);

  while (!MQTT.connected())
  {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    if (MQTT.connect(ID_MQTT, MQTT_USER, MQTT_PASS))
    {
      Serial.println("Conectado com sucesso. Subscrevendo ao tópico ");
      Serial.print(MQTT_TOPIC);
    }

    else 
    {
      Serial.println("Falha ao se conectar. Reconectando...");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    
  }
  
}

// 5.4 - Sensor
void sendSensor() {
  const char alias_humi[] = "h";
  const char alias_temp[] = "t";

  door_open = digitalRead(DOOR_SWITCH_PIN);
  h = dht.readHumidity();
  t = dht.readTemperature();

  // Verifica se o DHT está com algum erro de leitura
  if(isnan(h)) {
    Serial.println("Não foi possível ler o sensor de umidade.");
  }

  if(isnan(t)) {
    Serial.println("Não foi possível ler o sensor de temperatura.");
  }

  // Mostra as leituras no serial e envia pro broker
  Serial.println("Umidade: ");
  Serial.print(h);
  Serial.print(" %.");
  
  Serial.println("Temperatura: ");
  Serial.print(t);
  Serial.print(" ºC.");

  // Cria o objeto dinamico "json" com tamanho "2" para a biblioteca
  DynamicJsonDocument json(JSON_OBJECT_SIZE(2));

  // Grava as leituras no arquivo JSON
  //json[alias_humi] = humi;                         // ou seria h e t???
  //json[alias_temp] = temp;
  json[alias_humi] = h;                         // ou seria h e t???
  json[alias_temp] = t;
          
  //Mede o tamanho da mensagem "json" e atrela o valor somado em uma unidade ao objeto "tamanho_mensagem"
  size_t msg_size = measureJson(json) + 1;
          
  //Cria a string "mensagem" de acordo com o tamanho do objeto "tamanho_mensagem"
  char msg[msg_size];
         
  //Copia o objeto "json" para a variavel "mensagem" e com o "tamanho_mensagem"
  serializeJson(json, msg, msg_size);
          
  //Publica a variavel "mensagem" no servidor utilizando a variavel "TOPICO"
  Serial.println("Mensagem enviada: ");
  Serial.print(msg);
  MQTT.publish(MQTT_TOPIC, msg);
}

// 5.5 - Alarmes

void alarmConfig() {
  if(door_open == HIGH) {
    Serial.println("Porta aberta!");
    digitalWrite(ALARM_OPEN_DOOR_PIN, HIGH);
  }

  else {
    digitalWrite(ALARM_OPEN_DOOR_PIN, LOW);
  }

  if(h < 80) {
    Serial.println("Alarme de umidade acionado!");
    digitalWrite(ALARM_HUMI_PIN, HIGH);
  }

  else {
    digitalWrite(ALARM_HUMI_PIN, LOW);
  }
  
  if(t > 50) {
    Serial.println("Alarme de temperatura acionado!");
    digitalWrite(ALARM_TEMP_PIN, HIGH);
  }

  else {
    digitalWrite(ALARM_TEMP_PIN, LOW);
  }
}


  


//-----------------------------------------------------------------------


// 5.6 - DHT e Display

void DHT(){
  
    
  // Serial.print("reading DHT");
  

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {          //o método begin() do objeto display, instância de Adafruit_SSD1306 (tensão, end. do Oled I2C)
      Serial.println(F("SSD1306 allocation failed"));

    
      for (;;);
      // Initialising the UI will init the display too.
      //display.init();

    }

    delay(2000);
    display.clearDisplay();
    //display.setTextColor(white);


    //read temperature and humidity
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    //pisca rápido
  


    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
      delay(1000);
    }                                                           // Fechamento da função condicional if

    else{

     // clear display
      display.clearDisplay();

      // display temperature
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("Temperature: ");
      display.setTextSize(2);
      display.setCursor(0, 10);
      display.print(t);
      display.print(" ");
      display.setTextSize(1);
      display.cp437(true);
      display.write(167);
      display.setTextSize(2);
      display.print("C");
      delay (1000);

              
      
      
      // display humidity
      display.setTextSize(1);
      display.setCursor(0, 35);
      display.print("Humidity: ");
      display.setTextSize(2);
      display.setCursor(0, 45);
      display.print(h);
      display.print("  %");
      delay (1000);

      
      //pisca rápido
      digitalWrite(2, HIGH);                   // LED Liga
      delay(20);                               // Espera t
      digitalWrite(2, LOW);                    // LED Desliga
      delay(100);                              // Espera 2t
      digitalWrite(2, HIGH);                   // LED Liga
      delay(20);                               // Espera t
      digitalWrite(2, LOW);                    // LED Desliga
      delay(500);                               // Espera 50t
      
      
      display.display();
      
      Serial.print("Umidade: ");                    // Solicita que seja impresso no monitor serial o texto Umidade
      Serial.print(h);                              // Solicita que seja impresso no monitor serial a variável umidade
      Serial.print("%");                            // Solicita que seja impresso no monitor serial o texto %
      Serial.print("Temperatura: ");                // Solicita que seja impresso no monitor serial o texto Temperatura
      Serial.print(t);                              // Solicita que seja impresso no monitor serial a variável temperatura
      Serial.println("*C");                         // Solicita que seja impresso no monitor serial o texto *C
      
      delay(7000);                                  // Adiciona um atraso de 1000ms para que o Arduino não fique lendo continuamente



  
    }
} 

void loop() {
  MQTT.loop();    // Mantém a conexão com o MQTT sempre ativa
  sendSensor();
  alarmConfig();
  DHT();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

