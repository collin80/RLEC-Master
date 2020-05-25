// Required libraries
#include <esp32_can.h>

#define EnerdelCAN	Can0

float minVoltage, maxVoltage;
float leafVolts, leafAmps;

uint8_t rlecs[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
uint8_t rlec_pos;

float voltages[192];
uint8_t balRes[192];
int8_t cellTemps[192];
int8_t rlecTemps[16];

uint32_t lastMillis;

bool debugMode = false;
bool debugCAN = false;

void setup()
{
  rlec_pos = 0;
  lastMillis = 0;
  Serial.begin(1000000);
    
  EnerdelCAN.begin(CAN_BPS_500K);  
  EnerdelCAN.watchFor();
}

void sendBroadcast()
{
  CAN_FRAME outgoing;
 
  outgoing.length = 8;
  outgoing.id = 0x7E1;
  outgoing.extended = false;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = 1; //system state = 1 = Normal
  outgoing.data.byte[1] = 12; //# of cells - always 12
  outgoing.data.byte[2] = 12; //# of temp sensors - always 12
  outgoing.data.byte[3] = 0x1; //cell balancing controlled by MLEC
  outgoing.data.byte[4] = 0x1; //Hybrid balancing enabled
  outgoing.data.byte[5] = 0; //no informational faults or anything
  outgoing.data.byte[6] = 0; //informational min filtered cell voltage
  outgoing.data.byte[7] = 0; //ditto - other byte
  EnerdelCAN.sendFrame(outgoing);
  delay(1);

  outgoing.id = 0x7E2;
  outgoing.extended = false;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = 10; //hybrid balancing upper voltage limit, no idea of proper value
  outgoing.data.byte[1] = 10; //other byte
  outgoing.data.byte[2] = 10; //hybrid balancing lower limit
  outgoing.data.byte[3] = 10;
  outgoing.data.byte[4] = 0; //say there is no charging
  outgoing.data.byte[5] = 0; //charge state, just set to 0
  outgoing.data.byte[6] = 1; //master charge enable flag 
  outgoing.data.byte[7] = 0; //master charge disable at cold temps flag
  EnerdelCAN.sendFrame(outgoing);
  delay(2);

  //RLEC boards use this message for balancing.
  outgoing.id = 0x7E3;
  uint16_t balVoltage = (uint16_t)((minVoltage + 0.025f) / 0.00244f);
  /*if (debugMode) 
  {
      Serial.print("Balance Val: ");
      Serial.println(balVoltage, HEX);
  }*/
  
  outgoing.extended = false;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = balVoltage >> 8;
  outgoing.data.byte[1] = (balVoltage & 0xFF);
  outgoing.data.byte[2] = balVoltage >> 8;
  outgoing.data.byte[3] = (balVoltage & 0xFF);
  outgoing.data.byte[4] = 0;
  outgoing.data.byte[5] = 9; //legacy param - min voltage differential
  outgoing.data.byte[6] = 0;
  outgoing.data.byte[7] = 3; //min cell voltage hysteresis 
  EnerdelCAN.sendFrame(outgoing);
  delay(2);

//only for legacy stuff. all values are hardcoded things that someone else found to work. No idea if the values matter at all.
  outgoing.id = 0x7E4;
  outgoing.extended = false;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = 0;
  outgoing.data.byte[1] = 9;
  outgoing.data.byte[2] = 0x46;
  outgoing.data.byte[3] = 0x2D;
  outgoing.data.byte[4] = 0x0A;
  outgoing.data.byte[5] = 2;
  outgoing.data.byte[6] = 0;
  outgoing.data.byte[7] = 0x4B;
  EnerdelCAN.sendFrame(outgoing);
  delay(2);

//more stuff that doesn't matter
  outgoing.id = 0x7E5;
  outgoing.extended = false;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = 5;
  outgoing.data.byte[1] = 2;
  outgoing.data.byte[2] = 0x3; //absolute voltage lower limit = 2.4v
  outgoing.data.byte[3] = 0x51;
  outgoing.data.byte[4] = 3;  //point where balancing starts to back off = 2.6v
  outgoing.data.byte[5] = 0xAE;
  outgoing.data.byte[6] = 5; //informational voltage of highest cell
  outgoing.data.byte[7] = 0xCA;
  EnerdelCAN.sendFrame(outgoing);
  delay(2);

  outgoing.id = 0x7E6;
  outgoing.length = 2;
  outgoing.extended = false;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = 0x23; //not used by RLECS but max temp
  outgoing.data.byte[1] = 0x1E; //min temp
  EnerdelCAN.sendFrame(outgoing);
  delay(2);  
}

void sendTargeted(int which)
{
   if (which < 0) which = 0;
   if (which > 15) which = 15;

   int offset = 0x20 * which;

   CAN_FRAME outgoing;

  //all zeros for normal operation - a  bunch of overrides
  outgoing.id = 0x406 + offset;
  outgoing.extended = false;
  outgoing.length = 8;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = 0; //normal operation
  outgoing.data.byte[1] = 0;
  outgoing.data.byte[2] = 0x0;
  outgoing.data.byte[3] = 0x0;
  outgoing.data.byte[4] = 0;
  outgoing.data.byte[5] = 0;
  outgoing.data.byte[6] = 0;
  outgoing.data.byte[7] = 0;
  EnerdelCAN.sendFrame(outgoing);
  delay(2);

//bunch more overrides that must be set to 0
  outgoing.id = 0x40A + offset;
  outgoing.extended = false;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = 0x0; //enable Mode 4 balance resistor override
  outgoing.data.byte[1] = 0x0;
  outgoing.data.byte[2] = 0x0;
  outgoing.data.byte[3] = 0x0;
  outgoing.data.byte[4] = 0x0;
  outgoing.data.byte[5] = 0;
  outgoing.data.byte[6] = 0;
  outgoing.data.byte[7] = 0;
  EnerdelCAN.sendFrame(outgoing);
  delay(2);

//once again, overrides and debugging modes. Do not set any bits
  outgoing.id = 0x40B + offset;
  outgoing.extended = false;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = 0;
  outgoing.data.byte[1] = 0;
  outgoing.data.byte[2] = 0;
  outgoing.data.byte[3] = 0;
  outgoing.data.byte[4] = 0;
  outgoing.data.byte[5] = 0;
  outgoing.data.byte[6] = 0;
  outgoing.data.byte[7] = 0;
  EnerdelCAN.sendFrame(outgoing);
  delay(2);

  outgoing.id = 0x40C + offset;
  outgoing.extended = false;
  outgoing.priority = 4; //0-15 lower is higher priority
  outgoing.data.byte[0] = 0;
  outgoing.data.byte[1] = 0;
  outgoing.data.byte[2] = 0;
  outgoing.data.byte[3] = 0;
  outgoing.data.byte[4] = 0;
  outgoing.data.byte[5] = 0;
  outgoing.data.byte[6] = 12;
  outgoing.data.byte[7] = 12;
  EnerdelCAN.sendFrame(outgoing);
  delay(2);
}

void processRLECFrame(CAN_FRAME &frame)
{
   int rlecNum = frame.id / 0x20;
   int msgNum = frame.id & 0xF;
   float tempVal;
   int arrayOffset;

   /*if (rlecNum < 8)*/ arrayOffset = 12 * (rlecNum - 0);
   //else arrayOffset = 12 * (rlecNum - 8);

  //Serial.print("RLEC ");
  //Serial.print(rlecNum);
  //Serial.print(" Msg ");
  //Serial.println(msgNum);

  switch (msgNum)
  {
     case 1: //first four filtered cell voltages in motorola 16 bit format (0.00244v1 per)
        voltages[arrayOffset] = (frame.data.byte[0] * 256 + frame.data.byte[1]) * 0.00244f;
        voltages[arrayOffset + 1] = (frame.data.byte[2] * 256 + frame.data.byte[3]) * 0.00244f;
        voltages[arrayOffset + 2] = (frame.data.byte[4] * 256 + frame.data.byte[5]) * 0.00244f;
        voltages[arrayOffset + 3] = (frame.data.byte[6] * 256 + frame.data.byte[7]) * 0.00244f;
        if (debugCAN)
        {
          Serial.print("R: ");
          Serial.print(rlecNum);
          Serial.print(" V1 ");
          Serial.print((frame.data.byte[0] * 256 + frame.data.byte[1]) * 0.00244f);
          Serial.print(" V2 ");
          Serial.print((frame.data.byte[2] * 256 + frame.data.byte[3]) * 0.00244f);
          Serial.print(" V3 ");
          Serial.print((frame.data.byte[4] * 256 + frame.data.byte[5]) * 0.00244f);
          Serial.print(" V4 ");
          Serial.println((frame.data.byte[6] * 256 + frame.data.byte[7]) * 0.00244f);
        }
        break;
     case 2: //middle 4 filtered voltages
        voltages[arrayOffset + 4] = (frame.data.byte[0] * 256 + frame.data.byte[1]) * 0.00244f;
        voltages[arrayOffset + 5] = (frame.data.byte[2] * 256 + frame.data.byte[3]) * 0.00244f;
        voltages[arrayOffset + 6] = (frame.data.byte[4] * 256 + frame.data.byte[5]) * 0.00244f;
        voltages[arrayOffset + 7] = (frame.data.byte[6] * 256 + frame.data.byte[7]) * 0.00244f;

        if (debugCAN)
        {     
          Serial.print("R: ");
          Serial.print(rlecNum);

          Serial.print(" V5 ");
          Serial.print((frame.data.byte[0] * 256 + frame.data.byte[1]) * 0.00244f);
          Serial.print(" V6 ");
          Serial.print((frame.data.byte[2] * 256 + frame.data.byte[3]) * 0.00244f);
          Serial.print(" V7 ");
          Serial.print((frame.data.byte[4] * 256 + frame.data.byte[5]) * 0.00244f);
          Serial.print(" V8 ");
          Serial.println((frame.data.byte[6] * 256 + frame.data.byte[7]) * 0.00244f);        
        }
        break;
     case 3: //last 4 filtered voltages
        voltages[arrayOffset + 8] = (frame.data.byte[0] * 256 + frame.data.byte[1]) * 0.00244f;
        voltages[arrayOffset + 9] = (frame.data.byte[2] * 256 + frame.data.byte[3]) * 0.00244f;
        voltages[arrayOffset + 10] = (frame.data.byte[4] * 256 + frame.data.byte[5]) * 0.00244f;
        voltages[arrayOffset + 11] = (frame.data.byte[6] * 256 + frame.data.byte[7]) * 0.00244f;

        if (debugCAN)
        {     
          Serial.print("R: ");
          Serial.print(rlecNum);

          Serial.print(" V9 ");
          Serial.print((frame.data.byte[0] * 256 + frame.data.byte[1]) * 0.00244f);
          Serial.print(" V10 ");
          Serial.print((frame.data.byte[2] * 256 + frame.data.byte[3]) * 0.00244f);
          Serial.print(" V11 ");
          Serial.print((frame.data.byte[4] * 256 + frame.data.byte[5]) * 0.00244f);
          Serial.print(" V12 ");
          Serial.println((frame.data.byte[6] * 256 + frame.data.byte[7]) * 0.00244f);        
        }
        break;
     case 4: //max filtered cell V, min filtered cell V, filtered rlec temp, balance resistor states, faults
          //0-1 = max filtered cell V
          //2-3 = min filtered cell V
          //4 = filtered RLEC temp
          //5(3:0) = cell 12 - 9 balance resistor stateus (1 = ON, 0 = OFF)
          //6 = cell 8 - cell 1
          //7:7 = cell 1 voltage fault, 7:6 = Zero cap voltage fault, 7:5 = module voltage adc fault          
          //7:4 = cell voltage adc fault, 7:3 = CV conn fault, 2 = RLEC temp fault
          //7:1 = cell temp adc fault, 7:0 = cell 1 ADC fault
         tempVal = (frame.data.byte[0] * 256 + frame.data.byte[1]) * 0.00244f;
         if (tempVal > maxVoltage) maxVoltage = tempVal;
         tempVal = (frame.data.byte[2] * 256 + frame.data.byte[3]) * 0.00244f;
        if (tempVal < minVoltage) minVoltage = tempVal;
        rlecTemps[rlecNum] = frame.data.byte[4];
        //if (frame.data.byte[5] > 0 || frame.data.byte[6] > 0)
        //{
          if (debugCAN)
          {

            Serial.print("R: ");
            Serial.print(rlecNum);
            Serial.print("Balance Resistors: ");
            Serial.print(frame.data.byte[5], BIN);
            Serial.write('-');
            Serial.println(frame.data.byte[6], BIN);
          }
        //}
        for (int bal = 0; bal < 8; bal++)
        {
            if (frame.data.byte[5] & (1 << bal)) balRes[arrayOffset] = 1;
            else balRes[arrayOffset] = 0;
        }
        for (int bal = 0; bal < 4; bal++)
        {
            if (frame.data.byte[6] & (1 << bal)) balRes[arrayOffset + 8] = 1;
            else balRes[arrayOffset + 8] = 0;
        }
        
        //do something if min/max is too large
        break;
     case 5: //unfiltered cell voltages 1-4 - same scheme as filtered but don't need them
        break;
     case 6: //unfiltered cell voltages 5-8
        break;
     case 7: //unfiltered cell voltages 9-12
        break;
     case 8: //raw ADC value for cell voltages 1-4 //same scheme as previous two but still don't want it
        break;
     case 9: //raw ADC value for cell voltages 5-8
        break;
     case 10: //raw ADC value for cell voltages 9-12
        break;
     case 11: //redundant cell v 1, zero cap raw V, module raw V, module filtered V (V of this whole pack)
        //0-1 = Redundant cell 1 voltage
        //2-3 = zero cap voltage (raw) - Legacy.
        //4-5 = Module voltage Raw (0.0122V per)
        //6-7 = Module Voltage Filtered 
        //        Serial.print("R: ");
        //Serial.print(rlecNum);

        //Serial.print("Module V ");
        //Serial.println((frame.data.byte[6] * 256 + frame.data.byte[7]) * 0.01222f);
        break;
     case 12: //filtered cell temperatures 1-8 (signed C)
        cellTemps[arrayOffset] = frame.data.byte[0];
        cellTemps[arrayOffset + 1] = frame.data.byte[1];
        cellTemps[arrayOffset + 2] = frame.data.byte[2];
        cellTemps[arrayOffset + 3] = frame.data.byte[3];
        cellTemps[arrayOffset + 4] = frame.data.byte[4];
        cellTemps[arrayOffset + 5] = frame.data.byte[5];
        cellTemps[arrayOffset + 6] = frame.data.byte[6];
        cellTemps[arrayOffset + 7] = frame.data.byte[7];
        /*
             Serial.print("R: ");
        Serial.print(rlecNum);

        Serial.print(" T1 ");
        Serial.print(frame.data.byte[0]);
        Serial.print("   T2 ");
        Serial.print(frame.data.byte[1]);
        Serial.print("   T3 ");
        Serial.print(frame.data.byte[2]);
        Serial.print("   T4 ");
        Serial.println(frame.data.byte[3]);
        Serial.print("T5 ");
        Serial.print(frame.data.byte[4]);
        Serial.print("   T6 ");
        Serial.print(frame.data.byte[5]);
        Serial.print("   T7 ");
        Serial.print(frame.data.byte[6]);
        Serial.print("   T8 ");
        Serial.println(frame.data.byte[7]);     */
        break;
     case 13: //filtered cell temps 9-12, min/max temp, heater temp, RLEC build num
        cellTemps[arrayOffset + 8] = frame.data.byte[0];
        cellTemps[arrayOffset + 9] = frame.data.byte[1];
        cellTemps[arrayOffset + 10] = frame.data.byte[2];
        cellTemps[arrayOffset + 11] = frame.data.byte[3];
        //0-3 = remaining temperatures (9-12)
        //4 = max temp (filtered)
        //5 = min temp (filtered)
        //6 = filtered heater temp (legacy, ignored. Should be 0)
        //7 = rlec software ver. - Should be 0xC
        //        Serial.print("R: ");
        //Serial.print(rlecNum);

        //Serial.print(" T9 ");
        //Serial.print(frame.data.byte[0]);
        //Serial.print("   T10 ");
        //Serial.print(frame.data.byte[1]);
        //Serial.print("   T11 ");
        //Serial.print(frame.data.byte[2]);
        //Serial.print("   T12 ");
        //Serial.println(frame.data.byte[3]);     
        break;
  }
}

void loop(){
  CAN_FRAME incoming;
  float minV, maxV, thisV, totalV;
  float minV2, maxV2, totalV2;
  
  
  if (rlec_pos == 0) 
  {
    minVoltage = 10000.0f;
    maxVoltage = 0.0f;
  }
  sendBroadcast();
  sendTargeted(rlecs[rlec_pos]);
  rlec_pos = (rlec_pos + 1);
  if (rlec_pos == 16)
  {
      rlec_pos = 0;
      minV = 100.0f;
      maxV = 0.0f;
      minV2 = 100.0f;
      maxV2 = 0.0f;
      if ((millis() - lastMillis) > 1000)
      {
        lastMillis = millis();
        totalV = 0.0f;
        totalV2 = 0.0f;
        
        if (debugMode)  
        {
          Serial.println("Voltages:");        
          for (int j = 0; j < 16; j++)
          {
              if (j == 8) Serial.println();
              Serial.print("R");
              Serial.print(j);
              Serial.write("(");
              Serial.print(rlecTemps[j]);
              Serial.write(')');
              Serial.print(":   ");
              for (int k = 0; k < 12; k++) {
                thisV = voltages[j * 12 + k];
                Serial.print(thisV);
                Serial.write('(');
                Serial.print(cellTemps[j * 12 + k]);
                Serial.write(')');
                if (balRes[j * 12 + k] == 1) Serial.write('*');
                if (j < 8) 
                {
                  if (thisV > maxV) maxV = thisV;
                  if (thisV < minV && thisV > 0.0) minV = thisV; 
                  totalV += thisV;
                }
                else
                {
                  if (thisV > maxV2) maxV2 = thisV;
                  if (thisV < minV2 && thisV > 0.0) minV2 = thisV; 
                  totalV2 += thisV;                  
                }
                Serial.print(' ');
              }
              Serial.println();
           }
           Serial.print("Max Cell V: " );
           Serial.print(maxV);
           Serial.print("     Min V: ");
           Serial.print(minV);
           Serial.print("   Pack voltage: ");
           Serial.println(totalV);
           Serial.print("Second Max Cell V: " );
           Serial.print(maxV2);
           Serial.print("     Min V: ");
           Serial.print(minV2);
           Serial.print("   Pack voltage: ");
           Serial.println(totalV2);
           
           Serial.println();
           Serial.println();
           
        }
     }
  }
  
	while (EnerdelCAN.available())
	{
		EnerdelCAN.read(incoming);
		processRLECFrame(incoming);      
	}  
	
  if (Serial.available())
  {
     int byt = Serial.read();
     if (byt == 'D') 
     {
         debugMode = !debugMode;
         Serial.print("State of debug mode is now: ");
         Serial.println(debugMode);
     }
     if (byt == 'F') debugCAN = !debugCAN;
  }
}
