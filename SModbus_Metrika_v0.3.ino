/*
   The example will use packet1 to read a HOLDING REGISTER from Modbus address (command 0x04) for BADGER, ENDRESS and EQUISYS meters,
   from the arduino master and the slaves with several id´s.
   Sensors are going to be tested with the BIG ENDIANESS system, where the first data received is MOST SIGNIFICATIVE BYTE, and the last are the LEAST SIGNIFICATIVE BYTE.
   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
   Uploaded for read the INPUT REGISTER for MACROMETER (101), EUROMAG Models 608 and 406 Meters that were tested before successfully got the values 
   
   Created by: Hector C. Sandoval 
   October, 20th, 2021
   Modified: November, 24th, 2021
   Ver0.2
   Tested: December 3rd 2021.
   
*/

#include <fp64lib.h>                  // Libreria para manejo datos 64 bits
#include <SimpleModbusMaster.h>       // Modbus master library

// Modbus holding registers aforador MC406
#define      MC406_REG_FR   0x067F    //   3:1663 Modbus Register Reading Input Reg (CMD04) - LSB WORD Tasa de Flujo [m3 / hora]    --Flow Rate instantaneus float 32 bits   LITTLE ENDIAN 
#define      MC406_REG_TV   0x0689     //  3:1673 Modbus Register Reading Input Reg (CMD04) - LSB Tasa de Volumen acumulado [m3]    --Total Positive m3  float 64 bits

// Modbus holding registers aforador MC608
#define     MC608_REG_FR     0x0003       // 3:0003 - LSB Tasa de Flujo [m3 / seg]    --Flow Rate instantaneus [m3/seg], float 32 bits
#define     MC608_REG_TV     0x0009       // 3:0009 - LSB Volumen total + [m3]    -- Total volume + (accumulated) , float 32 bits

// Modbus holding registers aforador MCCROMETER PC-RA2, empieza en el registro 0 para leer 14 Input register
#define      PC101_REG_FR   0x0004    //    3:0005 Modbus Register Reading Input Reg (CMD04) - MSB WORD Tasa de Flujo [m3 / tiempo]  float 32 bits    -- flow rate value in the unit of measure chosen (as can be seen in the display of the instrument)
#define      PC101_REG_TV   0x0008    //    3:0008 Modbus Register Reading Input Reg (CMD04) - MSB WORD Volumen Total + [m3 ], unsigned long 32 bits  -- Totalizer T+ value

// Modbus holding registers aforador MoodMag 2000, Comunicacion Modbus RTU RS485
#define      M2000_REG_TV   0x00CF      //     3,4:0206 Modbus Holding Register CMD04 - MSB WORD Total Volumen + [m3]    -- Volume +T
#define      M2000_REG_FR   0x00ED      //     3,4:0237 Modbus Holding Register CMD04 - MSB WORD Flow Rate [m3/seg]    -- Flow Rate +T
//#define      M2000_REG_UN   0x00F1      //     3,4:0241 Modbus Holding Register CMD04 - MSB WORD Flow Rate User units  -- Flow Rate user units

// Modbus holding registers aforador MoodMag 5000, Comunicacion Modbus RTU RS232
#define      M5000_REG_TV   0x0207      //   3,4:0206   Modbus Holding Register CMD04 - MSB WORD Total Volumen + [m3]    -- Volume +T
#define      M5000_REG_FR   0x0237      //   3,4:0237   Modbus Holding Register CMD04 - MSB WORD Flow Rate [m3/seg]    -- Flow Rate +T
//#define      M5000_REG_UN   0x0241      //   3,4:0241   Modbus Holding Register CMD04 - MSB WORD Flow Rate User units  -- Flow Rate user units

// Modbus holding registers aforador ProMag L400, Comunicacion Modbus RTU RS485
#define      PGL400_REG_TV  0x07D8   //    3:2008 Modbus Register Reading Input Reg CMD04 - MSB WORD Flow Rate [l/h]
#define      PGL400_REG_FR  0x0A31   //    3:2609 Modbus Register Reading Input Reg CMD04 - MSB WORD Total Volumen + [m3]   -- Volume +T

// Modbus holding registers aforador ULTRA TT, EquySIS, Comunicacion Modbus RTU RS232
#define      ULTRATT_REG_TV   0x0200    //    3:0512 (0x0200) Modbus Holding Register CMD04 - MSB WORD Total Volumen + [m3]    -- Volume +T
#define      ULTRATT_REG_FR   0x0400   //    3:1024 (0x0400) Modbus Holding Register CMD04 - MSB WORD Flow Rate [m3/seg]    -- Flow Rate +T

//////////////////// Port information ///////////////////
#define baud 19200
#define timeout 500
#define polling 100 // the scan rate
#define retry_count 10

// used to toggle the receive\transmit pin on the driver
#define TxEnablePin 2 

// The total amount of available memory on the master to store data
#define TOTAL_NO_OF_REGISTERS 10    // REG[1] = _REG_FR 

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.

char * modAfor = ""; 

float flowRate, totalVol;
//uint32_t tmp = 0x00, tmp2 = 0x00;
//uint32_t trans = 0x0000; 

enum
{
  PACKET1,
  PACKET2,
  //PACKET3,
  TOTAL_NO_OF_PACKETS // leave this last entry
};

typedef enum 
{
  NONE,
  MC608,
  MC406,
  PC101,
  M2000,
  M5000,
  L400,
  ULTRATT,
  LAST
} MODEL;

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

typedef Packet* packetPointer;

// Masters register array
uint16_t  regs[TOTAL_NO_OF_REGISTERS];


// Create a packetPointer to access each packet individually. This is not required you can access the array explicitly. E.g. packets[PACKET1].id = 2;
// This does become tedious though...
packetPointer packet1 = &packets[PACKET1];
packetPointer packet2 = &packets[PACKET2];
//packetPointer packet3 = &packets[PACKET3];

//Funciones prototipo
float Hex2Float(uint32_t x);
uint32_t Float2Hex(float y);
boolean setModbus(MODEL modelo);
void datosModbus(MODEL Modelo);

//MODEL Modelo =  MC608;
//MODEL Modelo =  MC408;
MODEL Modelo =  PC101; 
//MODEL Modelo =  M2000;       // Indica el tipo de modelo que se va a usar, con una estructura tipo ENUMERACION definida 
// MODEL Modelo =  L400;
//MODEL Modelo = ULTRATT; 

void setup()
{
  // Initialize each packet
  Serial.begin(19200);                            // USB (UART) Communication with the arduino and the app desktop PC
  Serial3.begin(19200);                            // Modbus RTU RS485 Communication
  Serial.flush();

  Serial.print("  -- Modelo enumeracion = ");
  Serial.println(Modelo);
  if(!setModbus(Modelo)) Serial.println("¡¡¡ Fail Modbus Communication !!!");
  else Serial.println("¡Succesfull Modbus Communication!");
  // Initialize the Modbus Finite State Machine at Serial 3 for MEGA Arduino (TX = 14, RX = 15)
  modbus_configure(&Serial3, baud, SERIAL_8N1, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);
  Serial.print("  --Modelo Caudalimetro = ");
  Serial.println(modAfor);
  delay(1000);
}

void loop()
{
 //flowRate=0.0;
 //totalVol=0.0;  
 if (packet1->failed_requests >= 10) packet1->connection = true;
 if (packet2->failed_requests >= 10) packet2->connection = true;
 modbus_update();
 datosModbus(Modelo);
}


float Hex2Float(uint32_t x)
{
  return (*(float*)&x);
}

uint32_t Float2Hex(float y){
  return (*(uint32_t*)&y);
}


boolean setModbus(MODEL Modelo){
  switch(Modelo)
   {
    case MC608:
      modbus_construct(packet1, 1, READ_INPUT_REGISTERS, MC608_REG_FR, 2, 1);  // ID=1, CMD=4 (Read Input Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      modbus_construct(packet2, 1, READ_INPUT_REGISTERS, MC608_REG_TV, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate  
      break;
      
    case MC406: 
      modbus_construct(packet1, 1, READ_INPUT_REGISTERS, MC406_REG_FR, 2, 1);  // ID=1, CMD=4 (Read Input Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      modbus_construct(packet2, 1, READ_INPUT_REGISTERS, MC406_REG_TV, 4, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate  
      break;
    
    case PC101:
      modbus_construct(packet1, 1, READ_INPUT_REGISTERS, PC101_REG_FR, 2, 1);  // ID=1, CMD=4 (Read Input Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      modbus_construct(packet2, 1, READ_INPUT_REGISTERS, PC101_REG_TV, 2, 3);  // ID=1, CMD=4 (Read Input Reg), ModbusAdrr, Reg_read, Index_MasterReg)
      break;
    
    
    case M2000:                // Modelo MoodMag M2000  Badger    if modAfor == "MC2000" 
      modAfor = "M2000"; 
      modbus_construct(packet1, 1, READ_HOLDING_REGISTERS, M2000_REG_TV, 2, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg 
      modbus_construct(packet2, 1, READ_HOLDING_REGISTERS, M2000_REG_FR, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate
      //modbus_construct(packet3, 1, READ_INPUT_REGISTERS, M2000_REG_UN, 2, 1);  // Packet 3 point to next Modbus regiser, in this case Flow rate
      //modbus_construct(packet3, 1, READ_INPUT_REGISTERS, M2000_REG_UN, 2, 3);  // Packet 3 point to next Modbus regiser, in this case Flow rate
      break;
   
    case M5000:                // Modelo MoodMag M5000  Badger   if modAfor == "MC5000"
      modAfor = "M5000";
      modbus_construct(packet1, 1, READ_HOLDING_REGISTERS, M5000_REG_TV, 2, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg
      modbus_construct(packet2, 1, READ_HOLDING_REGISTERS, M5000_REG_FR, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate
      //modbus_construct(packet1, 1, READ_INPUT_REGISTERS, M5000_REG_TV, 2, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg
      //modbus_construct(packet2, 1, READ_INPUT_REGISTERS, M5000_REG_FR, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate
      break;
   
    case L400:                // Modelo ProMag L400    Endress + Hauseer  if modAfor == "L400"
      modAfor = "ProMag L400";
      modbus_construct(packet1, 1, READ_INPUT_REGISTERS, PGL400_REG_TV, 2, 1);  // ID=1, CMD=4 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg
      modbus_construct(packet2, 1, READ_INPUT_REGISTERS, PGL400_REG_FR, 2, 3);  // Packet 2 point to next Modbus regiser, in this case Flow rate  
      break;
   
    case ULTRATT:                // Modelo ULTRA TT  EQUYSIS       if modAfor == "ULTRATT"
        /*
        * Function Code: 0x03    Date: 0x07 0x5B 0xCD 0x15, mean 0X75BCD15 = 123456789
        * Register: 0x0200, #Points: 0x0003   Date: 0x2C 0x01, mean m³-Unit, One fractional part.
        * CRC16: 0x04 0x73    Positive flow volume is 12345678.9m³
        * 
        */
      modAfor = "Ultra TT EQUYSIS";
      modbus_construct(packet1, 1, READ_HOLDING_REGISTERS, ULTRATT_REG_TV, 3, 1);  // ID=1, CMD=3 (Read HOLDING Reg), ModbusAdrr, Reg_read, Index_MasterReg);  Positive flow volume High-16bit
      modbus_construct(packet2, 1, READ_HOLDING_REGISTERS, ULTRATT_REG_FR, 3, 4);  // Packet 2 point to next Modbus regiser, in this case Flow rate;         Flow Rate High-16 bit
      //modbus_construct(packet1, 1, READ_INPUT_REGISTERS, ULTRATT_REG_TV, 3, 1);  // ID=1, CMD=4 (Read INPUT Reg), ModbusAdrr, Reg_read, Index_MasterReg);  Positive flow volume High-16bit
      //modbus_construct(packet2, 1, READ_INPUT_REGISTERS, ULTRATT_REG_FR, 3, 4);  // Packet 2 point to next Modbus regiser, in this case Flow rate;         Flow Rate High-16 bit
      break;
    
    default:
      return false;
      break;
   }
   return true;
}


void datosModbus(MODEL Modelo){
  uint32_t tmp = 0x00;
  uint32_t trans = 0x00;
  delay(100);
  Serial.println(); 
  // get the new byte:
  Serial.println("______________DEBUG MODBUS________________");
  Serial.print("Peticion Reg Vol: ");
  Serial.println(packet1->requests);
  Serial.print("\tPeticion Exitosa: ");
  Serial.println(packet1->successful_requests);
  Serial.print("\tPeticion fallida: ");
  Serial.println(packet1->failed_requests);
  Serial.print("\tErrores: ");
  Serial.println(packet1->exception_errors);
  Serial.print("\t Conexion  >");
  Serial.println(packet1->connection);
  Serial.print("Peticion Reg FR: ");
  Serial.println(packet2->requests);
  Serial.print("\tExitosa: ");
  Serial.println(packet2->successful_requests);
  Serial.print("\tfallida: ");
  Serial.println(packet2->failed_requests);
  Serial.print("\tErrores: ");
  Serial.println(packet2->exception_errors);
  Serial.print("\tConexion  >");
  Serial.println(packet2->connection);
  Serial.println();   
  switch (Modelo)
  {
     case MC608:                          // Little endian byte format
        Serial.println("_____________FLOW_RATE_______________");
        tmp = regs[2];
        trans = tmp << 16 | regs[1]; // update data to be written to arduino slave
        flowRate = Hex2Float(trans);
        Serial.println("Registros ");
        Serial.print(" -- Reg[1]: ");
        Serial.println(regs[1],HEX);
        Serial.print(" -- Reg [2]: ");
        Serial.println(regs[2],HEX);
        Serial.print("  --> Inst. measured fluid Flow Rate ** MC608 **  [m3/s] = ");
        Serial.println(flowRate,4);
        Serial.println();
        tmp = regs[4];     // HIGH WORD for Total Volumen in m3
        trans = tmp << 16 | regs[3]; // update data to be written to arduino slave
        totalVol = Hex2Float(trans);    
        Serial.println("____________TOTAL_VOLUMEN_______________");
        // Volumen total m3
        Serial.println("Volumen Total [m3]");
        Serial.print(" -- MSB WORD Flow Rate: ");
        Serial.println(regs[4],HEX);
        Serial.print(" -- LSB WORD Flow Rate: ");
        Serial.println(regs[3],HEX);
        Serial.print(" --Total Volumen [m3] = ");
        Serial.println(totalVol);
        Serial.print("  --Total Volumen [m3] float = ");
        Serial.println(totalVol,4);
        Serial.println();
        break;

     case MC406:       // Little endian byte format
        float64_t trans2, tmp2, tmp, trans;
        char *str;
        Serial.println("_____________FLOW_RATE_______________");
        tmp = regs[2];
        trans = tmp << 16 | regs[1]; // update data to be written to arduino slave
        flowRate = Hex2Float(trans);
        Serial.println("Registros ");
        Serial.print(" -- Reg[1]: ");
        Serial.println(regs[1],HEX);
        Serial.print(" -- Reg [2]: ");
        Serial.println(regs[2],HEX);
        Serial.print("  --> Inst. measured fluid Flow Rate ** MC608 **  [m3/s] = ");
        Serial.println(flowRate,4);
        Serial.println();
       
    // Read 4 Input Registers starting at 31673 (MODBUS ADDRESS 1162)
    // Serial.print("  --Inst. measured fluid Flow Rate [m3/h] = ");
    // Serial.println(flowRate);
    // Serial.print("  --Inst. measured fluid Flow Rate [m3/s] float = ");
    // Serial.println(flowRate/3600,5);
    // Serial.print("  --Inst. measured fluid Flow Rate [lts/s] float = ");
    // Serial.println(flowRate/36,5);
      
        tmp2 = regs[6];
        trans2 = tmp2 << 48; 
  //    str = fp64_to_string( trans2, 17, 4);
  //    Serial.print(" String float64: ");
  //    Serial.println(str);
        tmp2 = regs[5];
        tmp2 = tmp2 << 32;
  //    str = fp64_to_string( tmp2, 17, 4);
  //    Serial.print(" String float64: ");
  //    Serial.println(str);
        trans = regs[4];
  //    trans = trans << 16;
  //    str = fp64_to_string( trans, 17, 4);
  //    Serial.print(" String float64: ");
  //    Serial.println(str);
        tmp = regs[3];
        trans2 = trans2 + tmp2 + trans + tmp ;
        str = fp64_to_string( trans2, 17, 4);
        totalVol = fp64_ds(trans2);
   
        Hex2Float(totalVol);
        Serial.println();   
        Serial.println("_____________TOTAL_VOLUME_______________"); 
        Serial.println("Registros ");
        Serial.print(" -- Reg[6]: ");
        Serial.println(regs[6],HEX);
        Serial.print(" -- Reg [5]: ");
        Serial.println(regs[5],HEX);
        Serial.print(" -- Reg[4]: ");
        Serial.println(regs[4],HEX);
        Serial.print(" -- Reg [3]: ");
        Serial.println(regs[3],HEX);
  //      Serial.print(" String float64: ");
  //      Serial.println(str);
        Serial.print(" Total Vol: ");
        Serial.println(totalVol,3);
        break;

    case PC101:
      Serial.println("_____________FLOW_RATE_______________");
      tmp = regs[1];
      trans = tmp << 16 | regs[2]; // update data to be written to arduino slave
      flowRate = Hex2Float(trans);
      Serial.println("Registros ");
      Serial.print(" -- Reg[1]: ");
      Serial.println(regs[1],HEX);
      Serial.print(" -- Reg [2]: ");
      Serial.println(regs[2],HEX);
      Serial.print("  --> Inst. measured fluid Flow Rate ** MC608 **  [m3/s] = ");
      Serial.println(flowRate,4);
      Serial.println();
      tmp = regs[3];     // HIGH WORD for Total Volumen in m3
      trans = tmp << 16 | regs[4]; // update data to be written to arduino slave
      totalVol = Hex2Float(trans);    
      Serial.println("____________TOTAL_VOLUMEN_______________");
      // Volumen total m3
      Serial.println("Volumen Total [m3]");
      Serial.print(" -- MSB WORD Flow Rate: ");
      Serial.println(regs[3],HEX);
      Serial.print(" -- LSB WORD Flow Rate: ");
      Serial.println(regs[4],HEX);
      Serial.print(" --Total Volumen [m3] = ");
      Serial.println(totalVol);
      Serial.print("  --Total Volumen [m3] float = ");
      totalVol = long (trans);
      Serial.println(totalVol,4);
      Serial.println();
      break;

      
        
    case M2000:                // Modelo MoodMag M2000  Badger    
         // Modbus Byte format:  Big endian  HIGH word send it first, then LOW Word
         Serial.println("_____________FLOW_RATE_______________");
         tmp = regs[3];
         trans = (tmp << 16 | regs[4]) ; // update data to be written to arduino slave
         //Serial.print(" --> transition: ");
         //Serial.println(trans, HEX);
         flowRate = Hex2Float(trans);
         Serial.println("Registros ");
         Serial.print(" -- Reg[3]: ");
         Serial.println(regs[3],HEX);
         Serial.print(" -- Reg [4]: ");
         Serial.println(regs[4],HEX);
         Serial.print("  --> Inst. measured fluid Flow Rate ** ModMag M2000**  [m3/s] = ");
         Serial.println(flowRate,4);
         tmp = regs[1];
         trans = tmp  << 16 | regs[2];
         totalVol = Hex2Float(trans);
         Serial.println("_____________TOTAL_VOLUME_______________"); 
         //Serial.print(" --> transition: ");
         //Serial.println(trans, HEX);
         Serial.println("Registros ");                 
         Serial.print(" -- Reg[1]: ");
         Serial.println(regs[1],HEX);
         Serial.print(" -- Reg [2]: ");
         Serial.println(regs[2],HEX);
         Serial.print(" Total Vol: ");
         Serial.println(totalVol,4);
         delay(1000);
         break;
   
    case M5000:                // Modelo MoodMag M5000  Badger   if modAfor == "MC5000"
        // Modbus Byte Order:  Big endian  HIGH word send it first, then LOW Word
         Serial.println("_____________FLOW_RATE_______________");
         tmp = regs[2];
         trans = tmp << 16 | regs[3]; // update data to be written to arduino slave
         flowRate = Hex2Float(trans);
         Serial.println("Registros ");
         Serial.print(" -- Reg[2]: ");
         Serial.println(regs[2],HEX);
         Serial.print(" -- Reg [3]: ");
         Serial.println(regs[3],HEX);
         Serial.print("  --> Inst. measured fluid Flow Rate ** ModMag M5000 ** [m3/s] = ");
         Serial.println(flowRate,4);
         tmp = regs[1];
         trans = tmp << 16 | regs[2];
         totalVol = Hex2Float(trans);
         Serial.println("_____________TOTAL_VOLUME_______________"); 
         Serial.println("Registros ");                 
         Serial.print(" -- Reg[1]: ");
         Serial.println(regs[1],HEX);
         Serial.print(" -- Reg [2]: ");
         Serial.println(regs[2],HEX);
         Serial.print(" Total Vol: ");
         Serial.println(totalVol,4);
         delay(1000);
        break;
   
    case L400:                // Modelo ProMag L400    Endress + Hauseer  if modAfor == "L400"
         Serial.println("_____________FLOW_RATE_______________");
         tmp = regs[1];
         trans = tmp << 16 | regs[2]; // update data to be written to arduino slave
         flowRate = Hex2Float(trans);
         Serial.println("Registros ");
         Serial.print(" -- Reg[1]: ");
         Serial.println(regs[1],HEX);
         Serial.print(" -- Reg [2]: ");
         Serial.println(regs[2],HEX);
         Serial.print("  --> Inst. measured fluid Flow Rate ** PG L400 ** [l/h] = ");
         Serial.println(flowRate,4);
         tmp = regs[3];
         trans = tmp << 16 | regs[4];
         totalVol = Hex2Float(trans);
         Serial.println("_____________TOTAL_VOLUME_______________"); 
         Serial.println("Registros ");                 
         Serial.print(" -- Reg[3]: ");
         Serial.println(regs[3],HEX);
         Serial.print(" -- Reg [4]: ");
         Serial.println(regs[4],HEX);
         Serial.print(" Total Vol: ");
         Serial.println(totalVol,4);
         delay(1000);
        break;
   
    case ULTRATT:                // Modelo ULTRA TT  EQUYSIS       if modAfor == "ULTRATT"
        /*
        * Function Code: 0x03    Date: 0x07 0x5B 0xCD 0x15, mean 0X75BCD15 = 123456789
        * Register: 0x0200, #Points: 0x0003   Date: 0x2C 0x01, mean m³-Unit, One fractional part.
        * CRC16: 0x04 0x73    Positive flow volume is 12345678.9m³
        * 
        */
        Serial.println("_____________FLOW_RATE_______________");
        tmp = regs[6];
        trans = tmp << 16 |  regs[7];
        Serial.println("Registros ");
        Serial.print(" -- Reg[6] HIGH High Word: ");
        Serial.println(regs[6],HEX);
        Serial.print(" -- Reg [7] LOW High Word: ");
        Serial.println(regs[7],HEX);
        Serial.print(" -- Reg[8] HIGH Low Word: ");
        Serial.println(regs[8],HEX);
        uint8_t unidades = regs[8] >> 8;
        Serial.println(unidades,HEX);
        if(unidades == 0x29) Serial.println(" L ");
        if(unidades == 0x2C) Serial.println(" m3 ");
        if(unidades == 0x32) Serial.println(" L/h ");
        if(unidades == 0x35) Serial.println(" m3/h ");
        else Serial.println("Unidad no reconocida");
                
        uint8_t fraction = regs[8] | 0x0F ;           // Like if it was and(|) 0x0F
        if (fraction == 0x00) flowRate = (float) trans * 1;
        else if (fraction == 0x01) flowRate = (float) trans * 0.1;
        else if (fraction == 0x02) flowRate = (float) trans * 0.01;
        else if (fraction == 0x03) flowRate = (float) trans * 0.001;
        else if (fraction == 0x04) flowRate = (float) trans * 0.0001;
        else if (fraction == 0x05) flowRate = (float) trans * 0.00001;
        else if (fraction == 0x06) flowRate = (float) trans * 0.000001;
        else if (fraction == 0x06) flowRate = (float) trans * 0.0000001;
        else if (fraction == 0x07) flowRate = (float) trans * 0.00000001;                
        Serial.print(" -- Reg [8]: fraction");
        Serial.println(fraction,HEX);
        Serial.print("  --> Inst. measured fluid Flow Rate ** ULTRA TT - EQUYSIS ** = ");
        Serial.println(flowRate);
        
        tmp = regs[1] << 16 ; 
        trans = tmp | regs[2];
        Serial.println("Registros ");
        Serial.print(" -- Reg[1] High Word: ");
        Serial.println(regs[1],HEX);
        Serial.print(" -- Reg [2]Low Word: ");
        Serial.println(regs[2],HEX);
        Serial.print(" -- Reg[3] Fractional Word: ");
        Serial.println(regs[3],HEX);
        Serial.print(" -- Reg[3] Units :");
        unidades = regs[3] >> 8;
        Serial.println(unidades,HEX);
        if(unidades == 0x29) Serial.println(" L ");
        if(unidades == 0x2C) Serial.println(" m3 ");
        if(unidades == 0x32) Serial.println(" L/h ");
        if(unidades == 0x35) Serial.println(" m3/h ");
        else Serial.println("Unidad no reconocida");
        
        fraction = regs[3];           // Like if it was and(|) 0x0F
        Serial.print(" -- Reg [3] fraction:");
        Serial.println(fraction,HEX);
        if (fraction == 0x00) totalVol = (float) trans * 1;
        else if (fraction == 0x01) totalVol = (float) trans * 0.1;
        else if (fraction == 0x02) totalVol = (float) trans * 0.01;
        else if (fraction == 0x03) totalVol = (float) trans * 0.001;
        else if (fraction == 0x04) totalVol = (float) trans * 0.0001;
        else if (fraction == 0x05) totalVol = (float) trans * 0.00001;
        else if (fraction == 0x06) totalVol = (float) trans * 0.000001;
        else if (fraction == 0x06) totalVol = (float) trans * 0.0000001;
        else if (fraction == 0x07) totalVol = (float) trans * 0.00000001;               
        Serial.println(totalVol);
        delay(1000);
        break;
    
    default:
      break;
  }
}
