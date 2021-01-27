// Required libraries
#include "variant.h"
#include <due_can.h>
#include <M2_12VIO.h>
#include <pins_arduino.h>
#include <Arduino.h>
#include "M2_IO.h"
#include <pwm_lib.h>
#include <can_common.h>
M2_12VIO M2IO; 

//Leave defined if you use native port, comment if using programming port
//This sketch could provide a lot of traffic so it might be best to use the
//native port
#define Serial SerialUSB

bool EngRun = false;
bool OilLvlLow = false;
bool OilPressLow = false;
bool ReducedPwr = false;
bool VSS_Stat = false;
bool PS_On = false;
bool AC = false;

byte rpm1 = 0;
byte rpm2 = 0;
byte APP = 0;
byte Mux1 = 0;
byte Mux2 = 0;
byte RadFan = 0;          //Porsche Radiator Fan signal - 7 bits long.  Bit 8 is engine run status
byte TPS = 0;             //Maybe GM Controller does not transmit - send APP instead, close enough
byte Coolant = 0;         //Eng Coolant Temp
byte OilLvl = 0;          //Eng Oil Level
byte OilPress = 0;            //Oil Pressure
byte OilTemp = 0;           //Oil Temperature
byte VehSpeed1 = 0;          //Vehicle Speed
byte VehSpeed2 = 0;          //Other Half Veh Speed 
byte SpdL = 0;
byte SpdH = 0; 
byte Cnt2 = 0;
byte EngTorqH = 0;
byte EngTorqL = 0;
byte CmndTorqH = 0;
byte CmndTorqL = 0;
byte Baro = 0;
byte PRNDL = 0x50;
byte Errors1 = 0x00;
byte MIL = 0x00;
byte Flash = 0x00;
byte OilVal = 0x00;
byte OilError = 0x00;
byte ChkEng = 0x00;
byte AC_Req = 0x00;

uint8_t codes[7] = {5, 0x5C, 0xC, 0x62, 0x61, 0x5A, 0x11};     

uint16_t EngTorq = 0;
uint16_t CmndTorq = 0;
uint16_t RPM = 0;


int Rvr_Input = 1;                           //Sets Analog Input 1 to be the input for reverse.  
int Timer = 0;
int value = 0;
int test = 0;
int Spd = 0;   
int test2 = 0;
int RvrSwitch = 1;  
int VSS = 0;
int MPH = 0;
int idx = 0;

float psi;
       
unsigned long PrevTime=0;
unsigned long PreCntTm=0;
unsigned long LoopRate_10ms = 0;
unsigned long LoopRate_1000ms = 0;
unsigned long LoopRate_100ms = 0;
unsigned long LoopRate_25ms = 0;
unsigned long time_now = 0;



void sendPIDRequest(uint32_t id, uint8_t PID)
{
   CAN_FRAME frame;
   frame.id = id;
   frame.extended = 0;
   frame.length = 8;
   for (int i = 0; i < 8; i++) frame.data.bytes[i] = 0xAA;
   frame.data.bytes[0] = 2; //2 more bytes to follow
   frame.data.bytes[1] = 1;
   frame.data.bytes[2] = PID;
   Can0.sendFrame(frame);
}



void processPID(CAN_FRAME &frame)
{

   
   if (frame.data.bytes[1] != 0x41) return; //not anything we're interested in then
   switch (frame.data.bytes[2])
   {
   case 5:
      Coolant = frame.data.bytes[3] - 40;
      SerialUSB.print("Coolant temperature (C): ");
      SerialUSB.println(Coolant);
      break;
   case 0x5C:
      OilTemp = frame.data.bytes[3] -40; //
      SerialUSB.print("Oil Temp (C): ");
      SerialUSB.println(OilTemp);      
      break;
   case 0xC:
      RPM = ((frame.data.bytes[3] * 256) + frame.data.bytes[4])/4;
      SerialUSB.print("Engine RPM: ");
      SerialUSB.println(RPM);  
      RPM = RPM*4;                                                        //Converts whole number into Porsche scaling (0.25RPM per bit)
      rpm2 = lowByte(RPM);                                                //Splits RPM into 2 bytes
      rpm1 = highByte(RPM);                                               //Splits RPM into 2 bytes          
      break;
   case 0x62:
      EngTorq = frame.data.bytes[3] -125;
      SerialUSB.print("Actual Engine Torque (%): ");
      SerialUSB.println(EngTorq);
      break;
   case 0x61:
      CmndTorq = frame.data.bytes[3] -125;
      SerialUSB.print("Drivers Demand Engine Torque (%): ");
      SerialUSB.println(CmndTorq);
      break;
   case 0x5A:
      APP = frame.data.bytes[3] *(100/255);
      SerialUSB.print("Throttle Pedal (%): ");
      SerialUSB.println(APP);
      break;
   case 0x11:
      TPS = frame.data.bytes[3] *(100/255);
      SerialUSB.print("Throttle Position (%): ");
      SerialUSB.println(TPS);
      break;
   }
}



void setup()
{
  
  M2IO.Init_12VIO();  // Initialize the M2I/O library
  //M2IO.Setpin_12VIO(2, ON, SOURCE, PWM_PIN, 2, 20);
  //M2IO.Setpin_12VIO(1, ON, SOURCE);
  
  pinMode(Led5, OUTPUT);    // Set the Green LED PIN as an OUTPUT
  //pinMode(Led4, OUTPUT);    // Set the Yellow LED PIN as an OUTPUT
  //pinMode(Led3, OUTPUT);    // Set the Yellow LED PIN as an OUTPUT
  digitalWrite(Led5, LOW);   // turn the Green LED ON by making the PIN LOW
 // digitalWrite(Led4, HIGH);   // turn the Yellow LED OFF by making the PIN HIGH
 // digitalWrite(Led3, HIGH);   // turn the Yellow LED OFF by making the PIN HIGH
  M2IO.InitButton_12VIO(Rvr_Input);          //Inits the input as a digital input for a button.  Looks for voltage to set true.  Use pullup resistor to use a ground input (ground will set false, so invert the logic)
  //M2IO.Setpin_12VIO(2, ON, SOURCE, PWM_PIN, VSS, 20);
  
  //Serial.begin(115200);
  
  // Initialize CAN0 and CAN1, Set the proper baud rates here
  Can0.begin(CAN_BPS_500K);
  Can1.begin(CAN_BPS_500K);
  
  //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept extended
  //or standard frames
  //Ignore this bit - Unsure what it means.  Copied from setup
  int filter;
  //extended
  for (filter = 0; filter < 3; filter++) {
	Can0.setRXFilter(filter, 0, 0, true);
	Can1.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (int filter = 3; filter < 7; filter++) {
	Can0.setRXFilter(filter, 0, 0, false);
	Can1.setRXFilter(filter, 0, 0, false);
  }  
  


//


}

void printFrame(CAN_FRAME &frame) {
   Serial.print("ID: 0x");
   Serial.print(frame.id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame.length);
   Serial.print(" Data: 0x");
   for (int count = 0; count < frame.length; count++) {
       Serial.print(frame.data.bytes[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\r\n");
}


void loop(){

//------------------------------------------Determine if Gear position is Reverse----------------------------------------------------------
RvrSwitch = M2IO.GetButton_12VIO(Rvr_Input);             //Only for Tiptronic - Illuminates drive or reverse and turns on reverse lights.  Use 10k pullup resistor between pin 23 on 26 pin connector.  Rev switch is ground circuit.  

if (RvrSwitch == 1) {
  PRNDL = 0x50;                                         //Sets PRNDL to Drive normally
} else {
  PRNDL = 0x70;                                         //When Reverse switch grounds, RvrSwitch sets to false, so reverse turns ON.  Inverse logic, be careful here.
}

//------------------------------------------Activate Power Steering Pump ----------------------------------------------------------------


if (RPM > 450 ) {
EngRun = true;
}
else if (RPM < 200 ) {
 EngRun = false;
}

if ((EngRun) && (!PS_On)){
M2IO.Setpin_12VIO(1, ON);                     //Sends 12V to pin 1 Source Output - Activates TRW powersteering pump.  Turns on only after RPM exceeds 450 RPM
PS_On = true; 
}

 if ((!EngRun) && (PS_On)){
 M2IO.Setpin_12VIO(1, OFF);                   //If RPM drops below 200, the Power steering will turn off.  This allows 250rpm hysteresis so pump does not shut off on a bad clutch engagement or other engine stutter.
 PS_On = false;
 }

//-----------------------------------------Disable Overcurrent Trip-----------------------------------------------------------------------

//M2 Overcurrent detection does not seem to function correctly with a fluctuating voltage source?  Unsure why it trips always when key on, key turned, engine speed changes, etc.  Print the overcurrent trip amperage to the serial bus anyway...


if (Over_Current_Trip) {
  
uint32_t Temp_Load = M2IO.Load_Amps();
      //SerialUSB.print("\n**Over_Current Load*** mA = ");
      //SerialUSB.print(Temp_Load);
      Over_Current_Trip = false;
      digitalWrite(IO_Enable, HIGH);  
}


//----------------------------------------Calculate Replicated Transmission Output Speed---------------------------------------------------


VSS = Spd*6.46;
if (VSS <= 0.5 && (VSS_Stat == true)) {
  M2IO.Setpin_12VIO(2, OFF);
  VSS_Stat = false;                        //If VSS is off, set this false so we do not fall in this loop every time
}
else if (VSS > 0.5){
  VSS_Stat = true;
if (VSS < 2  && VSS > 0.5) {                 //M2_12VIO cannot handle less than 2hz VSS.  This is less than 0.5mph, so if the vehicle is moving, we will just call it 2hz until it exceeds 2hz
VSS = 2;
}
if (VSS > 1000) {                            //M2_12VIO cannot handle higher than 1000hz PWM signal
  VSS = 1000;
}

//M2IO.Change_Frequency_12VIO(2,VSS);        //Done later in one of the timers, doing this every loop causes issues
}





//-------------------------------------------Read Porsche Side Frames--------------------------------------------------------------------  
CAN_FRAME incoming;
  
  if (Can1.available() > 0) {          //Also done later, so commented out here.
	Can1.read(incoming); 
	//printFrame(incoming);                //Uncomment this line to view all frames on the Porsche side printed to the serial monitor
  
  }
//------------------------------------------Read GM Side CAN Frames-----------------------------------------------------------------------
  if (Can0.available() > 0) {               //Read Frames coming from GM ECM.  Certain Frames identified and certain important bits assigned to variable
 Can0.read(incoming);
 // printFrame(incoming);                   //Uncomment this line to view all frames on the GM side printed to the serial monitor
      if (incoming.id > 0x7DF && incoming.id < 0x7F0)
      {
        printFrame(incoming);
         processPID(incoming);
      }    
  }





//-------------------------------------------------Radiator Fan----------------------------------------------------------
//The first 7 bits of this byte are used for the radiator fan.  The last bit is an Engine running status, that gets set at the end

if (Coolant < 90){
  RadFan = 0;
  }
if (Coolant > 90){
  RadFan = (Coolant-90)*13;                             //Radiator Fans start to ramp up above 90C coolant temps
}

if (Coolant > 110){
  RadFan = 255;
  }                       //If Coolant Reaches 110, radiator fans are set to max.


  if (EngRun) bitSet(RadFan,7);                        // Puts the left most bit of RadFan to true - In the Porsche this will be used for EngRunning True 
  if (!EngRun) bitClear(RadFan,7);                     //Based on RPM determination earlier, set engine running false if it's... not running.     


//-------------------------------------------------List Frames--------------------------------------------------------
  //The Following is a list of Messages to be sent to the Porsche CANbus
  
  CAN_FRAME DME1;
  CAN_FRAME DME2;
  CAN_FRAME DME3;
  CAN_FRAME DME4;                                                               
  CAN_FRAME DME5;
  CAN_FRAME DME6;
  CAN_FRAME DME7;
  CAN_FRAME DME8;
  CAN_FRAME TCM1;
  CAN_FRAME TCM2;
  CAN_FRAME ECM1;

 
//------------------------------------------------Create Timers and Mux Variables----------------------------------------
   
// The following set of "if" statements is to increment the various Multiplexer variables the Porsche Instrumentation is looking for in addition to a counter of unknown value.  Bit of a guess on these, attempted to copy these from a Porsche CAN log 
// If you are using this in your car, I believe this needs to be copied from your own DME.  Some of it changes depending on configuration. My car is RWD, 3.8L, tiptronic, 911 used in this example.  

unsigned long CntTm = millis();  

    if ((CntTm-PreCntTm) <= 20) {
      Mux1=0;
      Cnt2=0;
      Mux2=66;
    }
      else if ((CntTm-PreCntTm) <= 40 && (CntTm-PreCntTm) > 20) {
        Mux1=0;
        Cnt2=2;
        Mux2=128;
      }      
              
        else if ((CntTm-PreCntTm) <= 60 && (CntTm-PreCntTm) > 40) {
        Mux1=0x42;                   //I think this means 3.8L engine
        Cnt2=0;
        Mux2=66;
      }
                    else if ((CntTm-PreCntTm) <= 80 && (CntTm-PreCntTm) > 60) {
        Mux1=0x42;
        Cnt2=2;
        Mux2=128;
      }
        else if ((CntTm-PreCntTm) <= 100 && (CntTm-PreCntTm) > 80) {
          Mux1=0x81;                 //Automatic Gearbox -> 82 for Manual?
          Cnt2=0;
          Mux2=66;
        }
                else if ((CntTm-PreCntTm) <= 120 && (CntTm-PreCntTm) > 100) {
          Mux1=0x81;       
          Cnt2=2;
          Mux2=128;
        }
          else if ((CntTm-PreCntTm) <= 140 && (CntTm-PreCntTm) > 120) {
            Mux1=0xE5;
            Cnt2=0;
            Mux2=66;
          }
              else if ((CntTm-PreCntTm) <= 160 && (CntTm-PreCntTm) > 140) {
            Mux1=0xE5;
            Cnt2=2;
            Mux2=128;
          }
              else if ((CntTm-PreCntTm) > 160) {
            PreCntTm=CntTm;           
          }
//---------------------------------------------------------------------Send Frames to Porsche DRIVE Bus---------------------------------------------------------------------    

  
  DME1.id = 0x242;
  DME1.length = 8;
  DME1.extended=0;
  DME1.data.bytes[0]= 0x01;           //copied from DME output
  DME1.data.bytes[1]= 0xBE;
  DME1.data.bytes[2]= rpm2;           //Eng RPM  
  DME1.data.bytes[3]= rpm1;           //Eng RPM
  DME1.data.bytes[4]= 0x4F;           //copied from DME output
  DME1.data.bytes[5]= APP;            //Pedal or TPS
  DME1.data.bytes[6]= 0x2E;           //copied from DME output
  DME1.data.bytes[7]= 0x32;
//Can1.sendFrame(DME1);               //This line sends the CAN frame every loop of the arduino controller - do NOT do this unless you are doing some sort of testing, CAN bus could be overwhelmed very quickly.
                                      //  CAN frames typically 12.5ms at the very fastest.  Porsche seems to operate at 32ms.  

DME2.id = 0x245;
  DME2.length = 8; 
  DME2.extended=0;
  DME2.data.bytes[0]= Mux1;           //copied from DME output - "If" loops are above that generate this
  DME2.data.bytes[1]= Coolant;        //Coolant temp 0.75 per *C 
  DME2.data.bytes[2]= 0xC0;           //Bit 7 being true disables gauge flashes OT light. The other 7 - Unsure  C0 seems to something 0x01=FlashingCoolant 0x02=?? 0x04=?? 0x08=?? 0x10=?? 0x20=?? 0x40=?? 0x80=??
  DME2.data.bytes[3]= EngTorq;        //DOn't know what this is.  Looks pretty similar to torque, so that sounds good.
  DME2.data.bytes[4]= 0x5E;           //copied from DME output  5E
  DME2.data.bytes[5]= EngTorq;        //Also looks like Torque.  Just a guess.
  DME2.data.bytes[6]= OilPress;       //Maybe Oil Pressure used by something else?  Could be air flow but the range seen is very small for that.  Oil pressure is later in 0x441
  DME2.data.bytes[7]= EngTorq;        //No idea but torq follows a similar trend
//Can1.sendFrame(DME2);

DME3.id = 0x246;
  DME3.length = 8;
  DME3.extended=0;
  DME3.data.bytes[0]= 0x00;               //Bit 4 if AC Comp on, bits 1-8
  DME3.data.bytes[1]= 0x00;               //Torque Request for gearbox
  DME3.data.bytes[2]= CmndTorq;           //Pedal Torque guessing ~-100 to 860? 4nm per bit  
  DME3.data.bytes[3]= APP;                // TPS or Pedal
  DME3.data.bytes[4]= EngTorq;            //Engine Actual Torque guessing ~-100 to 860?  4nm per bit
  DME3.data.bytes[5]= Baro;               //Ambient Pressure 199(dec)= roughly 900ft above sea level 0.5kpa per bit
  DME3.data.bytes[6]= Cnt2;               //Counter unsure what it means
  DME3.data.bytes[7]= Mux2;               //Don't know what this one does, but I copied it from the DME output
//Can1.sendFrame(DME3);

DME4.id= 0x441;
DME4.length = 8;
DME4.extended=0;
DME4.data.bytes[0]=0x00;                  //0x01 = Flashing MIL 0x02 = Solid MIL  0x04 = Flashing Gas Gauge 0x08 = Reduced Engine Power  0x10 = Failure of Engine Comp Blower 0x20 = Oil temp faulty 0x40 = Low Oil Pressure 0x80 = Warning Battery Generator
DME4.data.bytes[1]=RadFan;                //First 7 bits of Byte 1 are for Radiator Fan speed.  Final bit is for Engine running bit. Ex: 10000000 (0x80) = Fans off Eng Running - Engine must be off for oil level check.  Coolant light and Gas light On when Engine is off?
DME4.data.bytes[2]=0x00;
DME4.data.bytes[3]=test;                   //Constantly incrementing counter.  Resets to 0 at 255
DME4.data.bytes[4]=0x00;
DME4.data.bytes[5]=OilTemp;               //Oil Temp
DME4.data.bytes[6]=OilPress;              //Oil Prsesure
DME4.data.bytes[7]=0x3A;                  //Copied from Tyler - 911UKforum 0x3A  Engine Compartment Temp (1-6) Bit 0 = coolant level.  Bit 7 = Eng Compartment temp senser failure
//Can1.sendFrame(DME4);
 //printFrame(DME4);

  
  DME5.id = 0x513;                    //Contents of this frame unknown.  Broadcasted on slow raster.  
  DME5.length = 8;
  DME5.extended=0;
  DME5.data.bytes[0]= 0x11;           //copied from DME output
  DME5.data.bytes[1]= 0x08;           //copied from DME output
  DME5.data.bytes[2]= 0xE9;           //copied from DME output
  DME5.data.bytes[3]= 0x1A;           //copied from DME output
  DME5.data.bytes[4]= 0x38;           //copied from DME output
  DME5.data.bytes[5]= 0x00;           
  DME5.data.bytes[6]= 0x00;
  DME5.data.bytes[7]= 0x00;
  //Can1.sendFrame(DME5);

   DME6.id = 0x669;                    //Contents of this frame unknown.  Broadcasted on slow raster.
  DME6.extended=0;
  DME6.data.bytes[0]= 0x00;           //copied from DME output
  DME6.data.bytes[1]= 0x00;           //copied from DME output
  DME6.data.bytes[2]= 0x00;           //copied from DME output
  DME6.data.bytes[3]= 0x00;           //copied from DME output
  DME6.data.bytes[4]= 0x00;           //copied from DME output
  DME6.data.bytes[5]= 0x00;           
  DME6.data.bytes[6]= 0x00;
  DME6.data.bytes[7]= 0x00;
  //Can1.sendFrame(DME6);

   DME7.id = 0x66B;                    //Contents of this frame unknown.  Broadcasted on slow raster. 
  DME7.length = 8;
  DME7.extended=0;
  DME7.data.bytes[0]= 0x41;           //copied from DME output
  DME7.data.bytes[1]= 0x42;           //copied from DME output
  DME7.data.bytes[2]= 0x09;           //copied from DME output
  DME7.data.bytes[3]= 0x35;           //copied from DME output
  DME7.data.bytes[4]= 0x53;           //copied from DME output
  DME7.data.bytes[5]= 0x47;           //           
  DME7.data.bytes[6]= 0x41;           //
  DME7.data.bytes[7]= 0x09;           //
  //Can1.sendFrame(DME7);
  
   DME8.id = 0x716;                    //Contents of this frame unknown.  Broadcasted on slow raster.   
  DME8.length = 8;
  DME8.extended=0;
  DME8.data.bytes[0]= 0x07;           //copied from DME output
  DME8.data.bytes[1]= 0x51;           //copied from DME output
  DME8.data.bytes[2]= 0x11;           //copied from DME output
  DME8.data.bytes[3]= 0x10;           //copied from DME output
  DME8.data.bytes[4]= 0x00;           //copied from DME output
  DME8.data.bytes[5]= 0x00;           //  
  DME8.data.bytes[6]= 0x00;           //
  DME8.data.bytes[7]= 0x00;           //
  //Can1.sendFrame(DME8);


TCM1.id= 0x440;                       //Gear position - assuming other status bits as well. 
TCM1.length = 8;
TCM1.extended=0;
TCM1.data.bytes[0]=0x00;               // 
TCM1.data.bytes[1]=PRNDL;              // Gear Position 70/71=R 6F=blinkingN/4 50=TheD
TCM1.data.bytes[2]=0x00;               //
TCM1.data.bytes[3]=0x00;               //
TCM1.data.bytes[4]=0x00;               //
TCM1.data.bytes[5]=0x00;               //
TCM1.data.bytes[6]=0x00;               //
TCM1.data.bytes[7]=0x85;               // Unknown, just copied from TCM output
//Can1.sendFrame(TCM1);

TCM2.id= 0x443;                       //Frame Unknown.  From TCM at faster raster.  Maybe torque requests, etc?
TCM2.length = 5;
TCM2.extended=0;
TCM2.data.bytes[0]=0x00;              // 
TCM2.data.bytes[1]=0x00;              //
TCM2.data.bytes[2]=0x00;              //
TCM2.data.bytes[3]=0x00;              //
TCM2.data.bytes[4]=0x00;              //
  //Can1.sendFrame(TCM2);

ECM1.id= 0xFF;                        // Frame to send AC request on GM ECM (Frame redacted)
ECM1.length = 8;
ECM1.extended=0;
ECM1.data.bytes[0]=0x00;               // 
ECM1.data.bytes[1]=0x00;               // 
ECM1.data.bytes[2]=0x00;               //
ECM1.data.bytes[3]=0x00;               //
ECM1.data.bytes[4]=0x00;               //
ECM1.data.bytes[5]=0x00;               //
ECM1.data.bytes[6]=0x00;               //
ECM1.data.bytes[7]=0x00;               // 


if (Can1.available() > 0) {
  Can1.read(incoming);
  if (incoming.id == 0x24A) {         //This frame comes from the PSM - ABS wheel speed sensors.  Used one of the rear wheel sensors.  Used as replicated TOS earlier in sketch.  0.02kph/bit
    //printFrame(incoming);
    SpdH=incoming.data.bytes[0];
    SpdL=incoming.data.bytes[1];
    }
    if (incoming.id == 0x600) {         //This frame is used for AC status
    //printFrame(incoming);
    AC_Req=incoming.data.bytes[0];     
    }
}
AC =  bitRead(AC_Req,3);               //Checks if the Porsche HVAC is requesting AC ON

Spd=((SpdH<<8) | SpdL);                //Combines the upper and lower byte into a singal uint16 (2byte) variable
Spd = Spd*0.02;                        //Convert from Porsche Scaling - Used earlier in the sketch to create a replicated TOS signal


//-----------------------------------------------------CAN Frame Sending Timers ---------------------------------------------------------------------


//The following are different timers.  Important can messages sent on high speed timers.  Lower importance sent at lower speed. 

if ((millis() - LoopRate_10ms) > 10)             //Sends frames every 10ms
  {
     LoopRate_10ms = millis();

     M2IO.Change_Frequency_12VIO(2,VSS);
     
     Can1.sendFrame(DME1);
     Can1.sendFrame(DME2);
     Can1.sendFrame(DME3);
     Can1.sendFrame(DME4);

     Can1.sendFrame(TCM1);

  }

  if ((millis() - LoopRate_1000ms) > 1000)             //1000ms cycle
  {
     LoopRate_1000ms = millis();
     
     Can1.sendFrame(DME6);
     Can1.sendFrame(DME7);
     Can1.sendFrame(DME8);

     
  }

    if ((millis() - LoopRate_100ms) > 100)             //100ms cycle
  {
     LoopRate_100ms = millis();
     test=test+1;  
SerialUSB.println(rpm1);
SerialUSB.println(rpm2);
SerialUSB.println(RPM);
SerialUSB.println(Coolant);
  
  }
  if ((millis() - LoopRate_25ms) > 25 )                //25ms cycle
  {

         sendPIDRequest(0x7DF, codes[idx]);               //Sends new PID Request every 10ms
     idx = (idx + 1) % 7;
     SerialUSB.println(".");
  LoopRate_25ms = millis();
  
  }





}
