// This script complements the dissertation titled
// "LOW-COST PRESCRIPTIVE MAINTENANCE FOR THREE-PHASE MACHINES" – UFSJ.
//
// The work proposes a low-cost architecture for monitoring electric machines,
// and introduces a new methodology for detecting low insulation faults.
//
// Note: This code implements MQTT communication with the ESP32,
// as well as the use of the ADXL345 sensor operating in streaming mode.
//
// The script has been segmented into several blocks to improve readability.
// Please pay close attention to the description of each block!

// -------------------- ------------------------librarys----------------------------------------------------------------------------------------

//library for using Wi-Fi
//biblioteca para uso do wifi
#include <WiFi.h>

//library that allows using MQTT
//biblioteca que permite usar o MQTT
#include <PubSubClient.h>

//library for converting files to JSON
//biblioteca que permite transformar aqurivos para tipo JSON
#include <ArduinoJson.h>

//library for SPI communication
//biblioteca para uso da comunicação SPI
#include <SPI.h>

//------------------------------------------------------------------------------------------------------------------------------------------------




//--------------Variables responsible for sending information and the architecture of transmission -----------------------------------------------

//additional variable responsible for maintaining the short-circuit test loop
//variavel adicional responsavel por manter o loop do ensaio de curto
int EC=1;

// Size of the vectors to be constructed
// Tamanho vetores a serem construidos
const int vetorSize = 3000;

int p=0;
int pacotes = 300;
int bytesSent = 0;
int tamanho_pacote = 10;
int pacotesEnviados = 0;
unsigned long tempoPassado;

int16_t A[vetorSize];
int16_t B[vetorSize];
int16_t C[vetorSize];
int16_t Z[vetorSize];
//------------------------------------------------------------------------------------------------------------------------------------------------




//----------------------------------------Variables for ADXL345 vibration sensor------------------------------------------------------------------


//Assign the Chip Select(CS) signal to pin 5.
// Atribui CS ao pino 5
int CS=5;

// List of some of the registers available on the ADXL345.
// lista registradores ADXL345
char POWER_CTL = 0x2D;  // Power Control Register
char DATA_FORMAT = 0x31;// Data format control
char leitura=0x39; // FIFO_STATUS

char DATA_RATE_REG = 0x2C; // Data Rate Register
char DATAX0 = 0x32; // X-Axis Data 0
char DATAX1 = 0x33; // X-Axis Data 1
char DATAY0 = 0x34; // Y-Axis Data 0
char DATAY1 = 0x35; // Y-Axis Data 1
char DATAZ0 = 0x36; // Z-Axis Data 0
char DATAZ1 = 0x37; // Z-Axis Data 1
char FIFO_CTL=0x38;// FIFO control

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int16_t x,y,z;
//------------------------------------------------------------------------------------------------------------------------------------------------




//---------------------------------------------------------Wi-Fi network settings-----------------------------------------------------------------

const char* ssid = "XXXXX"; //Wi-Fi network name
const char* password = "XXXXX"; //Wi-Fi password
const char* mqtt_server = "YYYYYY"; //IP address of your MQTT broker
//------------------------------------------------------------------------------------------------------------------------------------------------




//-----------------------------------------------------------MQTT broker settings-----------------------------------------------------------------

const int mqtt_port = 1883;

//ESP32 identifier for publishing, e.g.: C/3 short/3
//identificador esp32 para publicacao ex: C/3 curto/3
const char* mqtt_topic = "C/N";
const char* mqtt_topic_2 = "curto/N";

//Creates an object called espClient from the WiFiClient class. This object is responsible for managing TCP/IP communication over Wi-Fi
//Cria um objeto chamado espClient da classe WiFiClient. Esse objeto é responsável por gerenciar a comunicação TCP/IP via Wi-Fi
WiFiClient espClient;

//Creates a client object from the PubSubClient library
//Cria um objeto client da biblioteca PubSubClient 
PubSubClient client(espClient);
//------------------------------------------------------------------------------------------------------------------------------------------------

unsigned long inicioAquisicao; 

//------------------------------------------ Function responsible for the Wi-Fi connection--------------------------------------------------------
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());
}
//--------------------------------------------------------------------------------------------------------------------------------------------------



//-----------------------------------function responsible for short-circuit_test_reading------------------------------------------------------------
void leitura_curto(int16_t *vetor){
  int contador = 0;
  inicioAquisicao = micros();
    while (contador < vetorSize) {
      tempoPassado = micros() - inicioAquisicao;
        if (tempoPassado >= 500) {
          inicioAquisicao = micros();
          vetor[contador] = analogRead(32);
          contador++;
        }
     }
   contador = 0;
  }
//--------------------------------------------------------------------------------------------------------------------------------------------------



//-----------------------------Function responsible for sending data in JSON format-----------------------------------------------------------------

void envio(int ch,const char* TOPICO){

  // Create JSON document
  // Criar documento JSON 
  StaticJsonDocument<1024> jsonDocument;
  JsonArray vetor1Array = jsonDocument.createNestedArray("A");
  JsonArray vetor2Array = jsonDocument.createNestedArray("B");
  JsonArray vetor3Array = jsonDocument.createNestedArray("C");
  JsonArray vetor4Array = jsonDocument.createNestedArray("Z");
  JsonArray R = jsonDocument.createNestedArray("R");

  // Add vector data to the JSON and ensure it is sent in packets of 120 items (30 items x 4 vectors)
  // Adicionar dados dos vetores ao JSON e garante que eles sejam enviados em pacotes de 120 itens (30 itens X 4 vetores) 
  while( pacotesEnviados<pacotes){

    for (int i = 0; i < tamanho_pacote && bytesSent < vetorSize; i++) {
      vetor1Array.add(A[bytesSent]);
      vetor2Array.add(B[bytesSent]);
      vetor3Array.add(C[bytesSent]);
      vetor4Array.add(Z[bytesSent]);
      bytesSent++;
    }

    // Identifier -1 corresponds to the end-of-message marker
    // identificador -1 corresponde a marcacao final da mensagem
    pacotesEnviados++;
    if(pacotesEnviados<pacotes){
    R.add(0);
    }
    else{
      R.add(ch);
      }

    // Convert JSON to string and serialize the JSON
    // Converter JSON em string e serializa o Json
    String jsonData;
    serializeJson(jsonDocument, jsonData);

    // Sends the data
    // Envia os dados
    client.publish(TOPICO, jsonData.c_str());
   

    // Clears the JSON vectors
    // Limpa os vetores Json
    vetor1Array.clear();
    vetor2Array.clear();
    vetor3Array.clear();
    vetor4Array.clear();
    R.clear();
  }

  // Check if all packets have been sent and clear the variables
  // Verificar se todos os pacotes foram enviados e limpa as variaveis
  if (pacotesEnviados >= pacotes) {
    pacotesEnviados = 0; // Reiniciar contagem
    bytesSent = 0; // Reiniciar posição de envio dos vetores
    // Limpar vetores após o envio completo
    for (int i = 0; i < vetorSize; i++) {
      A[i] = 0;
      B[i] = 0;
      C[i] = 0;
      Z[i] = 0;
    }
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------------------



//---------------------------------------------------Relay switching functions----------------------------------------------------------------------
void iniciomedicao() {
  delay(8000);
  digitalWrite(21, HIGH); 
  digitalWrite(22, HIGH); 
  delay(2500);           
}

void fimomedicao() {
  digitalWrite(21, LOW); 
  digitalWrite(22, LOW); 
  delay(1000);           
  digitalWrite(16, LOW); 
  digitalWrite(17, LOW); 
  delay(1000);           
}

void chaveamento() {                
  digitalWrite(16, LOW);  
  digitalWrite(17, LOW);  
  leitura_curto(A);
  leitura_curto(B);
  leitura_curto(C);
  leitura_curto(Z);
  envio(-1,mqtt_topic_2);
  
  delay(2500);           
  
  digitalWrite(17, HIGH); 
  
  leitura_curto(A);
  leitura_curto(B);
  leitura_curto(C);
  leitura_curto(Z);
  envio(-1,mqtt_topic_2);
  
  delay(5000);          


  digitalWrite(16, HIGH); 
  leitura_curto(A);
  leitura_curto(B);
  leitura_curto(C);
  leitura_curto(Z);
  envio(-1,mqtt_topic_2);  
  delay(2500);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------



//---------------------------------------------------Current and vibration reading function---------------------------------------------------------
void corrente() {
  
  int contador = 0;
  inicioAquisicao = micros();
  while (contador < vetorSize) {
    tempoPassado = micros() - inicioAquisicao;
    if (tempoPassado >= 500) {
      inicioAquisicao = micros();
      A[contador] = analogRead(32);
      B[contador] = analogRead(33);
      C[contador] = analogRead(34);
      contador++;
    }
  }
 

  
  contador = 0;


    while (contador < vetorSize) {
      byte fifoStatus = readRegister(leitura);
        if(fifoStatus==1<<5){
          
          readRegister1(DATAX0, 6, values);
          x = ((int16_t)values[1]<<8)|(int16_t)values[0];
          //The Y value is stored in values[2] and values[3].
          y = ((int16_t)values[3]<<8)|(int16_t)values[2];
          //The Z value is stored in values[4] and values[5].
          z = ((int16_t)values[5]<<8)|(int16_t)values[4];
          Z[contador]=z;
          contador++;
      }
    }
     
  
  contador = 0;
  envio(-1,mqtt_topic);

}
//--------------------------------------------------------------------------------------------------------------------------------------------------


//---- Callback function: payload different from 1 corresponds to short-circuit test and payload equal to 1 sets continuous acquisition mode--------
void callback(char* topic, byte* payload, unsigned int length) {
    if ((char)payload[0] == '1'&& (EC==1)) {
    corrente();
  }
  else{
    while(true){
      EC=2;
      iniciomedicao();
      chaveamento();
      fimomedicao();
      EC=1;
      break;
      }

    }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------



//----------------------------------------------------------------------- Function to connect to MQTT-----------------------------------------------

// Note: To connect multiple clients, change the name in "client.connect("XXXXXX")"
// to something like "client.connect("ESP32Client3")" and 
// update "client.subscribe("XXXXXX")" to the corresponding topic created in MQTT,
// e.g., "client.subscribe("Topic1")"

void reconnect() {
  // Loop até conectar ao broker MQTT
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    if (client.connect("XXXXXX")) {
      Serial.println("conectado");
      client.subscribe("XXXXXX");
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      delay(5000);
    }
  }
}
//---------------------------------------------------------------------------------------------------------------------------------------------------



//--------------------------------Definition of GPIO pin operation modes and ADXL345 register settings-----------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(21, OUTPUT);  // Configura GPIO1 como saída
  pinMode(22, OUTPUT);  // Configura GPIO1 como saída
  pinMode(16, OUTPUT); // Configura GPIO16 como saída
  pinMode(17, OUTPUT); // Configura GPIO17 como saída
  
  SPI.begin();
 
  //configuração da comunicação SPI.
  SPI.setDataMode(SPI_MODE3);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  writeRegister(DATA_FORMAT, 0x00);
  writeRegister(DATA_RATE_REG, 0x0F);
  writeRegister(POWER_CTL, 0x08);  
  writeRegister(FIFO_CTL, 0x80); 
  SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE3));

  //configiuração WIFI
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------Void loop-------------------------------------------------------------
void loop() {
  // Reconectar se a conexão MQTT cair
  //Reconect MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
//--------------------------------------------------------------------------------------------------------------------------------------------------



// ------------------------------------------------Function to write a value to a register on the ADXL345------------------------------------------


void writeRegister(char registerAddress, char value) {
  
  // Set Chip Select pin low to signal the beginning of an SPI packet.
  // Seta o pino em low para iniciar a cominicação
  digitalWrite(CS, LOW);
  delayMicroseconds(5); 
  // Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  // Transfer the desired register value over SPI.
  SPI.transfer(value);
  // Set the Chip Select pin high to signal the end of an SPI packet.
  delayMicroseconds(5); 
  digitalWrite(CS, HIGH);

 

}
//--------------------------------------------------------------------------------------------------------------------------------------------------



//-----------------------------------------------------Function to read a single register from the ADXL345------------------------------------------
byte readRegister(char registerAddress) {
  byte result;
  
  // Set the Chip Select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
   delayMicroseconds(5); 
  // Transfer the register address with the read flag (most significant bit set).
  SPI.transfer(0x80 | registerAddress);
  // Read the result from the register.
  result = SPI.transfer(0x00);
  // Set the Chip Select pin high to end the SPI packet.
  
  delayMicroseconds(5); 
  digitalWrite(CS, HIGH);
   
 
  
  return result;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------



//-------------------------------------------------------------Function allows reading multiple registers------------------------------------------
void readRegister1(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  delayMicroseconds(5); 
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
 
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    delayMicroseconds(5); 
    values[i] = SPI.transfer(0x00);
  }
  
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
  delayMicroseconds(5); 
}
//--------------------------------------------------------------------------------------------------------------------------------------------------
