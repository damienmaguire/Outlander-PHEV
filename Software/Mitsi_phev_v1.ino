/*
Basic firmware to run Mitsubishi PHEV inverters/motors

*/
#include <Metro.h>
#include <due_can.h>  
#include <due_wire.h> 
#include <DueTimer.h>  
#include <Wire_EEPROM.h> 




#define Serial SerialUSB
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }


CAN_FRAME outFrame;  //A structured variable according to due_can library for transmitting CAN data.
CAN_FRAME inFrame;    //structure to keep inbound inFrames

//////timers//////////////////////////////
Metro timer_Frames10 = Metro(10);
Metro timer_Frames100 = Metro(80);
Metro timer_wifi = Metro(1100);
Metro timer_hv = Metro(1000);





////////////////////Pin Map////////////////////////////////////////////////////
int led = 13;         //onboard led for diagnosis
#define Throttle1 A0  //Analog throttle channel 1
#define Throttle2 A1  //Analog throttle channel 2
#define Brake 61      //Brake pedal switch. High = brake on
#define IN1 6         //General purpose digital input 1. High =12v
#define IN2 7         //General purpose digital input 2. High =12v
#define OUT1  48      //Low side switched general purpose digital output 1. high = on.
#define OUT2  49      //Low side switched general purpose digital output 2. high = on.
#define OUT3  50      //Low side switched general purpose digital output 3. high = on.
/////////////////////////////////////////////////////////////////////////////////

#define HVPreset 340 //voltage at which to enable main contactor
uint16_t TorqueVal=10000;
uint16_t TorqueReq=0;
byte  TorqueLo=0;
byte  TorqueHi=0;
byte  function=0x00; 

void setup() 
  {
  Can0.begin(CAN_BPS_500K);   // Inverter CAN
  Can1.begin(CAN_BPS_500K);   // Vehicle CAN
  //Can0.watchFor(0x1da); //set message filter for inverter can. Note sure if I can use two seperate values here. it might just pick 1!
  Can0.watchFor();
  Can1.watchFor(0x1ff); //just a blank message to block receive from e46 messages.
    
    Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.
    Serial2.begin(19200);  //Serial comms with ESP32 WiFi module on serial 2
   // Timer3.attachInterrupt(Msgs10ms).start(10000); // 10ms CAN Message Timer
   // Timer4.attachInterrupt(Msgs100ms).start(100000); //100ms CAN Message Timer
    
  pinMode(led, OUTPUT);
  pinMode(Brake, INPUT);
  pinMode(IN1, INPUT);  //T15 input from ign on switch
  pinMode(IN2, INPUT);
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  
  //digitalWrite(led, HIGH);
  digitalWrite(OUT1, LOW);  //precharge
  digitalWrite(OUT2, LOW);  //main contactor
  digitalWrite(OUT3, LOW);  //inverter power
  

 
  }

void handle_wifi(){
  if (timer_wifi.check())
  {
/*
 * 
 * Routine to send data to wifi on serial 2
The information will be provided over serial to the esp8266 at 19200 baud 8n1 in the form :
vxxx,ixxx,pxxx,mxxxx,oxxx,rxxx* where :

v=pack voltage (0-700Volts)
i=current (0-1000Amps)
p=power (0-300kw)
m=motor rpm (0-10000rpm)
o=motor temp (-20 to 120C)
r=inverter temp (-20 to 120C)
*=end of string
xxx=three digit integer for each parameter eg p100 = 100kw.
updates will be every 1100ms approx.

v100,i200,p35,m3000,o20,r100*
*/
  
Serial2.print("v100,i200,p35,m3000,o20,r100*"); //test string


  }
}

  



void Msgs10ms()                       //10ms messages here
{
if(timer_Frames10.check())
{
  
}
    }
    



void Msgs100ms()                      ////100ms messages here
{
if(timer_Frames100.check())
{


digitalWrite(led, !digitalRead(led)); //toggle led everytime we fire the 100ms messages.

        outFrame.id = 0x371;            // Set our transmission address ID
        outFrame.length = 8;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                
        outFrame.data.bytes[0]=0x30;
        outFrame.data.bytes[1]=0x00;  //seems to be a fixed msg.
        outFrame.data.bytes[2]=0x00;
        outFrame.data.bytes[3]=0x00;
        outFrame.data.bytes[4]=0x00;
        outFrame.data.bytes[5]=0x00;
        outFrame.data.bytes[6]=0x00;
        outFrame.data.bytes[7]=0x00;
        Can0.sendFrame(outFrame); 

        outFrame.id = 0x286;            // Set our transmission address ID
        outFrame.length = 8;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                
        outFrame.data.bytes[0]=0x00;
        outFrame.data.bytes[1]=0x00;  //seems to only change a small amount. picking a common value for testing...
        outFrame.data.bytes[2]=0x00;
        outFrame.data.bytes[3]=0x3d;
        outFrame.data.bytes[4]=0x00;
        outFrame.data.bytes[5]=0x00;
        outFrame.data.bytes[6]=0x21;
        outFrame.data.bytes[7]=0x00;
        Can0.sendFrame(outFrame); 


        outFrame.id = 0x285;            // Set our transmission address ID
        outFrame.length = 8;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                
        outFrame.data.bytes[0]=0x00;
        outFrame.data.bytes[1]=0x00;  //seems to only change a small amount. picking a common value for testing...
        outFrame.data.bytes[2]=0x14;
        outFrame.data.bytes[3]=0x39;
        outFrame.data.bytes[4]=0x91;
        outFrame.data.bytes[5]=0xfe;
        outFrame.data.bytes[6]=0x0c;
        outFrame.data.bytes[7]=0x10;
        Can0.sendFrame(outFrame);
        
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Torque Command = 0x287 
//0x2710=0mm
//0x2710=10000 decimal
//torque band = +/- 200nm
//200/10000= 0.02nm/bit
/////////////////////////////////////////////////////   

     //TorqueVal = 10000+TorqueReq;
     TorqueLo=lowByte(TorqueVal);
     TorqueHi=highByte(TorqueVal);

        outFrame.id = 0x287;            // Set our transmission address ID
        outFrame.length = 8;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                
        outFrame.data.bytes[0]=TorqueHi;  //front motor torque part 1
        outFrame.data.bytes[1]=TorqueLo;  //front motor torque part 2
        outFrame.data.bytes[2]=TorqueHi;  //rear motor torque part 1 0x2710=10000=0NM
        outFrame.data.bytes[3]=TorqueLo;  //rear motor torque part 2
        outFrame.data.bytes[4]=TorqueHi;  //generator torque part 1
        outFrame.data.bytes[5]=TorqueLo;  //generator torque part 2
        outFrame.data.bytes[6]=function;  //0x00,0x02=no inverter response to torque
                                      //0x03=front motor responds (possibly rear also)
                                      //0x04=generator only responds to torque
                                      //0x05=generator and front motor respond to torque.
        outFrame.data.bytes[7]=0x00;
        Can0.sendFrame(outFrame);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        
}
}



void readPedals()
{
  

}

void checkforinput()
{ 
  //Checks for keyboard input from Native port 
   if (SerialUSB.available()) 
     {
      int inByte = SerialUSB.read();
      switch (inByte)
         {
          case 'f':            
           SerialUSB.print("Torque set to: ");
           TorqueReq=SerialUSB.parseInt();
           TorqueVal=TorqueVal+TorqueReq;//add for forward dir
            SerialUSB.print(TorqueReq/10);
            SerialUSB.println("NM Forward");
            break;

           case 'r':            
           SerialUSB.print("Torque set to: ");
           TorqueReq=SerialUSB.parseInt();
           TorqueVal=TorqueVal-TorqueReq;//subtract for reverse dir
            SerialUSB.print(TorqueReq/10);
            SerialUSB.println("NM Reverse");
            break;

           case 's':            
           SerialUSB.println("Zero Torque Commanded");
           TorqueVal=10000;//0nm
            break;

           case 't':            
           SerialUSB.print("Function Commanded:");
           function=SerialUSB.parseInt();
           if(function==0x00) SerialUSB.println("Stop");
           if(function==0x03) SerialUSB.println("Motors Only");
           if(function==0x04) SerialUSB.println("Generator Only"); 
           if(function==0x05) SerialUSB.println("Generator and Motors");    
            break;
         }
     }
}



void ProcessRPM() //here we convert motor rpm values received from the leaf inverter into BMW E46 RPM can message.
{
  
}






void CheckCAN()
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//read incomming data from inverter//////////////////
//////////////////////////////////////////////////////
  if(Can0.available())
  {
    Can0.read(inFrame);
    //Serial.println(inFrame.id, HEX);
  

  }

////////////////////////////////////////////////////////////////////////////////////////////////

  
}




  
void loop()
{ 
 
Msgs100ms();  //fire the 100ms can messages
Msgs10ms();   //fire the 10ms can messages
checkforinput();
//readPedals(); //read throttle and brake pedal status.
//CheckCAN(); //check for incoming can
handle_wifi();  //send wifi data

}
