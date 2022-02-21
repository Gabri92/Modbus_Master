//////////////////////////////////////
// LIBRERIE
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
//////////////////////////////////////

//////////////////////////////////////
// DEFINE DEI BIT E REGISTRI CONDIVISI
// Bit di comando
#define powerOn_bit 0
#define reset_bit   7
#define homing_bit  8
#define moving_bit  9
// Bit di stato
#define homing_status 8
#define moving_status  9
// Registri condivisi
#define setpointRegister 0
#define cmdRegister 8
#define statusRegister 1000
#define encoderRegister  1001
//////////////////////////////////////

/////////////////////////////////
// DICHIARAZIONE FUNZIONI
uint16_t float_to_word(float);
uint16_t bitUp(uint16_t,int);
uint16_t bitDown(uint16_t,int);
void keyboardInput();
/////////////////////////////////

/////////////////////////////////
// DICHIARAZIONE VARIABILI
float fposition;
uint16_t position_to_send;
uint16_t slave_command;
bool power_on = false;
/////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
// SETTAGGI ETHERNET E MODBUS
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);

EthernetClient ethClient;
ModbusTCPClient modbusTCPClient(ethClient);

IPAddress server(192, 168, 1, 11); // update with the IP Address of your Modbus server
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
// CICLO PRINCIPALE
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
}

void loop() 
{
  // Se il client NON è connesso ritento la connessione, altrimenti eseguo in loop la funzione 'keyboardInput'
  if (!modbusTCPClient.connected()) 
  {
    // client not connected, start the Modbus TCP client
    Serial.println("Attempting to connect to Modbus TCP server");
    
    if (!modbusTCPClient.begin(server, 502)) 
    {
      Serial.println("Modbus TCP Client failed to connect!");
      delay(2000);
    } 
    else 
    {
      Serial.println("Modbus TCP Client connected");
    }
  } 
  else 
  {
    keyboardInput();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
// DEFINIZIONE FUNZIONI
/**
 * @brief 
 * Funzione che legge un carattere da tastiera ed esegue l'azione associata a tale carattere
 */
void keyboardInput()
{
  if(Serial.available())
  {
    char c = Serial.read();
    switch(c)
    {
      case 'm':
      // Generazione automatica di un float compreso tra 0 e 150
      fposition = float(random(0,15000))/100;
      position_to_send = float_to_word(fposition);
      Serial.println(fposition);
      // Alzo il bit che permette il movimento del cartesiano
      slave_command = bitUp(slave_command,moving_bit);
      if (!modbusTCPClient.holdingRegisterWrite(cmdRegister, slave_command)) 
      {
        Serial.print("Failed to write Register! ");
        Serial.println(modbusTCPClient.lastError());
      }
      // Scrivo la posizione desiderata nel registro condiviso
      if (!modbusTCPClient.holdingRegisterWrite(setpointRegister, position_to_send)) 
      {
        Serial.print("Failed to write Register! ");
        Serial.println(modbusTCPClient.lastError());
      }
      // Aspetto finchè non ho un feedback di azione completata dal PLC
      while(!(modbusTCPClient.inputRegisterRead(statusRegister) & 1<<moving_status)){}
      // Riabbasso il bit di comando
      slave_command = bitDown(slave_command,moving_bit);
      if (!modbusTCPClient.holdingRegisterWrite(cmdRegister, slave_command)) 
      {
        Serial.print("Failed to write Register! ");
        Serial.println(modbusTCPClient.lastError());
      }
      Serial.println("Move Done");
      break;

      case 'h':
      // Alzo il bit di comando dell'homing
      slave_command = bitUp(slave_command,homing_bit);
      if (!modbusTCPClient.holdingRegisterWrite(homing_bit, slave_command)) 
      {
        Serial.print("Failed to write Register! ");
        Serial.println(modbusTCPClient.lastError());
      }
      // Aspetto finchè non ho un feedback di homing completato dal PLC
      while(!(modbusTCPClient.inputRegisterRead(statusRegister) & 1<<homing_status)){}
      Serial.println(slave_command);
      Serial.println("Homing Done");
      // Riabbasso il bit di comando dell'homing
      slave_command = bitDown(slave_command,homing_bit);
      if (!modbusTCPClient.holdingRegisterWrite(cmdRegister, slave_command)) 
      {
        Serial.print("Failed to write Register! ");
        Serial.println(modbusTCPClient.lastError());
      }
      break;

      case 'o':
      // Il flag "power_on" gestisce la logica di questo if. Se premo 'o' ed il flag
      // è alzato abbasso il bit di comando, altrimenti lo alzo ed accendo i motori
      if(power_on)
      {
        slave_command = bitDown(slave_command,powerOn_bit);
        power_on = !power_on;
      }
      else
      {
        slave_command = bitUp(slave_command,powerOn_bit);
        power_on = !power_on;
      }
      // Scrivo il bit nel registro
      if (!modbusTCPClient.holdingRegisterWrite(cmdRegister, slave_command)) 
      {
        Serial.print("Failed to write Register! ");
        Serial.println(modbusTCPClient.lastError());
      }
      break;

      case 'r':
      // Alzo il bit di comando del reset e lo scrivo nel registro apposito condiviso
      slave_command = bitUp(slave_command,reset_bit);
      if (!modbusTCPClient.holdingRegisterWrite(cmdRegister, slave_command)) 
      {
        Serial.print("Failed to write Register! ");
        Serial.println(modbusTCPClient.lastError());
      }
      // Aspetto 2 secondi e riabbasso il bit
      delay(2000);
      slave_command = bitDown(slave_command,reset_bit);
      if (!modbusTCPClient.holdingRegisterWrite(cmdRegister, slave_command)) 
      {
        Serial.print("Failed to write Register! ");
        Serial.println(modbusTCPClient.lastError());
      }
      break;

      case 'p':
      // Leggo la posizione del registro condiviso in cui è salvata la posizione letta dall'encoder
      Serial.print("Position: ");
      Serial.print(float(modbusTCPClient.inputRegisterRead(encoderRegister))/100);
      Serial.println(" mm");
      break;
    }
  }
}

/**
 * @brief 
 * Conversione del float ad intero ed in seguito a word (uint16_t)
 * @param fnum 
 * @return uint16_t 
 */
uint16_t float_to_word(float fnum)
{
  uint16_t wnum = 0;
  int inum = fnum*100;
  byte high = 0;
  byte low = 0;

  high = high | (inum >> 8);
  low = low | inum;

  wnum = (wnum | high) << 8;
  wnum = wnum | low;

  return wnum;
}

/**
 * @brief 
 * Alzo il bit 'position' nel registro 'reg' e restituisco il registro aggiornato
 * @param reg 
 * @param position 
 * @return uint16_t 
 */
uint16_t bitUp(uint16_t reg, int position)
{
  uint16_t temp;

  temp = 1 << position;
  reg = reg | temp;

  return reg;
}

/**
 * @brief 
 * Abbasso il bit 'position' nel registro 'reg' e restituisco il registro aggiornato
 * @param reg 
 * @param position 
 * @return uint16_t 
 */
uint16_t bitDown(uint16_t reg, int position)
{
  uint16_t temp;

  temp = 1 << position;
  reg = reg & ~(temp);
  
  return reg;
}
//////////////////////////////////////////////////////////////////////////////////////////////