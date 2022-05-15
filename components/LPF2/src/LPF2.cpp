#include "LPF2.h"
//#include <Serial.h>



byte pow2(byte n) {
  byte p = 1;
  for (byte i =0; i<n; i++) {
    p*=2;
  }
  return p;
}
/**
 * Create a mode object
**/
EV3UARTMode::EV3UARTMode() {
}

void EV3UARTMode::setCallback(void(*ptr)(byte *, byte) ) {
    myptr=ptr;
    hasCallback=true;
  }

/**
 * Create the sensor emulation with the specified RX and TX pins
**/
EV3UARTEmulation::EV3UARTEmulation(byte RxPin, byte TxPin,  byte type, unsigned long speed) {
  //Serial2.begin(2400, SERIAL_8N1, RxPin, TxPin);
  this->rxpin = RxPin;
  this->txpin = TxPin;
  this->type = type;
  this->speed = speed;
  status = 0;
  modes = 0;
  views = 0;
  current_mode = 0;
}

/**
 * Define the next mode
**/
void EV3UARTEmulation::create_mode(String name, boolean view, 
		                           byte data_type, byte sample_size, 
						           byte figures, byte decimals) {
  EV3UARTMode* mode = new EV3UARTMode();
  mode->name = name;
  mode->view = view;
  mode->data_type = data_type;
  mode->sample_size = sample_size;
  mode->figures = figures;
  mode->decimals = decimals;
   mode->ranges=false; 
  mode->maps=false;
  mode_array[modes] = mode;
  modes++;
  if (view) views++;
}	


void EV3UARTEmulation::create_mode(String name, boolean view, 
                               byte data_type, byte sample_size, 
                       byte figures, byte decimals, 
                       float raw_low, float raw_high,          // Low and high values for raw data
                       float pct_low, float pct_high,
                        float si_low, float si_high,            // Low and high values for SI data
                        String unit) {
  EV3UARTMode* mode = new EV3UARTMode();
  mode->name = name;
  mode->view = view;
  mode->data_type = data_type;
  mode->sample_size = sample_size;
  mode->figures = figures;
  mode->decimals = decimals;
  mode->ranges=true; // send ranges
  mode->maps=false;
  mode->raw_low=raw_low;
  mode->raw_high=raw_high;
  mode->si_low=si_low;
  mode->si_high=si_high;
  mode->pct_low=pct_low;
  mode->pct_high=pct_high;
  mode->unit=unit;
  mode_array[modes] = mode;
  modes++;
  if (view) views++;
} 

void EV3UARTEmulation::create_mode(String name, boolean view, 
                               byte data_type, byte sample_size, 
                       byte figures, byte decimals, 
                       float raw_low, float raw_high,          // Low and high values for raw data
                       float pct_low, float pct_high,
                       float si_low, float si_high,            // Low and high values for SI data
                       String unit,
                       byte mapin,byte mapout) {
  EV3UARTMode* mode = new EV3UARTMode();
  mode->name = name;
  mode->view = view;
  mode->data_type = data_type;
  mode->sample_size = sample_size;
  mode->figures = figures;
  mode->decimals = decimals;
  mode->ranges=true; // send ranges
  mode->raw_low=raw_low;
  mode->raw_high=raw_high;
  mode->si_low=si_low;
  mode->si_high=si_high;
  mode->pct_low=pct_low;
  mode->pct_high=pct_high;
  mode->unit=unit;
  mode->maps=true;
  mode->mapin=mapin;
  mode->mapout=mapout;
  mode_array[modes] = mode;
  modes++;
  if (view) views++;
} 

/**
 * Get the status of the connection
**/
byte EV3UARTEmulation::get_status() {
  return status;
}

/**
 * Reset the connection
**/
void EV3UARTEmulation::reset () {
   union Data {
    unsigned long l;
    float f;
  } data;
  
  for(;;) {
#ifdef DEBUG
      Serial.println("Reset");
#endif
    // toggle Tx pin from 0 to 1
      pinMode(txpin, OUTPUT);  
    digitalWrite(txpin, LOW); // sets the digital pin txpin off
    delay(500);            // waits 0.5 s
    digitalWrite(txpin, HIGH);
	   //delay(100); 
	  Serial2.begin(2400,SERIAL_8N1,rxpin,txpin);
   //delay(100); 
   // empty read buffer
   while(Serial2.available()) Serial2.read();
   Serial2.write('\x00');
	  byte b[11]; // was b[4]
	  b[0] = type;
	  send_cmd(CMD_TYPE, b, 1);
	  b[0] = modes - 1;
	  b[1] = views - 1;
	  send_cmd(CMD_MODES, b, 2);
	  get_long(speed, b);
	  send_cmd(CMD_SPEED, b, 4);
    unsigned long m = millis();
    //while(!Serial2.available() && millis() - m < ACK_TIMEOUT);
    //if (Serial2.available()) {
      
    //}
	  for(int i=modes-1;i>=0;i--) {
		EV3UARTMode* mode = get_mode(i);
		byte l = mode->name.length();
		byte ll = next_power2(l);
		byte bb[ll+2];
		bb[0] = 0;  // name
		mode->name.getBytes(&bb[1],ll+1); // Leave room for null terminator
		byte lll = log2(ll);
		// Send name 
		send_cmd(CMD_INFO | (lll << CMD_LLL_SHIFT) | i, bb, ll+1);
		byte bbb[5];
		bbb[0] = 0x80;
		bbb[1] = mode->sample_size;
		bbb[2] = mode->data_type;
		bbb[3] = mode->figures;
		bbb[4] = mode->decimals;
		send_cmd(CMD_INFO | (2 << CMD_LLL_SHIFT) | i, bbb, 5);
    if (mode->ranges) {
       byte b_range[9];
       b_range[0]=RAW;
       data.f=mode->raw_low;
       get_long(data.l, b_range+1);  
       data.f=mode->raw_high;
       get_long(data.l, b_range+5);  
       send_cmd(CMD_INFO|(3 << CMD_LLL_SHIFT) | i,b_range,9); 
       b_range[0]=SI;
       data.f=mode->si_low;
       get_long(data.l, b_range+1);  
       data.f=mode->si_high;
       get_long(data.l, b_range+5);  
       send_cmd(CMD_INFO|(3 << CMD_LLL_SHIFT) | i,b_range,9); 
       b_range[0]=PCT;
       data.f=mode->pct_low;
       get_long(data.l, b_range+1);  
       data.f=mode->pct_high;
       get_long(data.l, b_range+5);  
       send_cmd(CMD_INFO|(3 << CMD_LLL_SHIFT) | i,b_range,9); 
      //  byte l_unit = mode->unit.length();
		  //  byte ll_unit = next_power2(l_unit);
		  //  byte bb_unit[ll_unit+2];
		  //  bb_unit[0] = SYM;  //  info_unit
      //  memset(bb_unit+1,0,ll_unit); // fill all bytes with 0
		  //  mode->unit.getBytes(&bb_unit[1],l_unit+1); // Leave room for null terminator
		  //  byte lll_unit = log2(ll_unit);
		  // // Send info_unit
      // Serial.printf("l_unit %d, ll_unit %d, lll_unit %d",l_unit,ll_unit,lll_unit);
		  //  send_cmd(CMD_INFO | (lll_unit << CMD_LLL_SHIFT) | i, bb_unit, ll_unit+1);
       
    }
    if (mode->maps) { //send mapin and mapout
      byte b_map[2];
      b_map[0]=FCT;
      b_map[1]=mode->mapin;
      b_map[2]=mode->mapout;
      send_cmd(CMD_INFO|(1 << CMD_LLL_SHIFT) | i, b_map,3);
    }
	  }
#ifdef DEBUG
      Serial.println("Sending ACK");
#endif
	  send_byte(BYTE_ACK);
	  m = millis();
	  while(!Serial2.available() && millis() - m < ACK_TIMEOUT) { delay(5); }
	  if (Serial2.available()) {  
      Serial.println("received something");
  		byte b = read_byte();
      Serial.println(b,HEX);
  		if (b == BYTE_ACK) {
        Serial.println("ACK received");
  		  Serial2.end();
        
        digitalWrite(txpin, LOW); 
        delay(10);
  		  Serial2.begin(speed,SERIAL_8N1, rxpin,txpin);
  		  delay(10);
  		  last_nack = millis();
  		  break;
  		}
      else {
         Serial.println("ACK timed out");
         Serial2.end();
         pinMode(txpin, OUTPUT);  
         digitalWrite(txpin, HIGH);
         delay(200);
         reset();
      }
	  }
	}
}

/**
 * Process incoming messages
**/
void EV3UARTEmulation::heart_beat() {
   union Data {
    unsigned long l;
    float f;
  } data;
  
  if (millis() - last_nack > HEARTBEAT_PERIOD) reset();
  if (Serial2.available()) {
      byte b = Serial2.read();
      if (b == BYTE_NACK) {
#ifdef DEBUG
        Serial.println("Received NACK");
#endif	  
// na nack ontvangen te hebben:
// 46 00 b9
 byte bb[1];
    bb[0]=0;
     send_cmd(CMD_WRITE | current_mode, bb, 1);
	    last_nack = millis();
	  } else {
#ifdef DEBUG
        Serial.print("Received: ");
		Serial.println(b, HEX);
#endif	
         if (b == CMD_SELECT) {
#ifdef DEBUG
        Serial.println("Received SELECT ");
#endif
		   byte checksum = 0xff ^ b;
		   byte mode = read_byte();
		   checksum ^= mode;
		   if (checksum == read_byte()) {
#ifdef DEBUG
        Serial.print("Mode is ");
		Serial.println(mode);
#endif
		     if (mode < modes) current_mode = mode;
		   }
		 } else if (b == 0x46) {
       // Serial.print("String received:");
       byte zero = read_byte();
       byte b9 = read_byte();
       byte ck = 0xff ^ zero ^ b9;
       if ((zero==0) && (b9=0xb9)) {
          ck=0xff;
          byte ch = read_byte(); // size and mode
          byte s = pow2((ch & 0b111000)>>3);
          byte mode = ch & 0b111;
          ck ^= ch;
          byte buf[32] = {0};
          for(byte i=0; i<s; i++) {
            buf[i]=read_byte();
                ck ^= buf[i];
          }
          byte cksm = read_byte();
          if (cksm != ck) {
            Serial.print("***CHSM ERROR***:"); 
            Serial.printf("cksm=%02X ck=%02X\n",cksm,ck);
            Serial.printf("%02X ",ch);
            for(byte i=0; i<s; i++) {
              Serial.printf(" %02X",buf[i]);
              
            }
            //Serial.println();
          }
          else {
          //  Serial.printf("%02X ",ch);
          //  for(byte i=0; i<s; i++) {
          //   Serial.printf(" %02X",buf[i]);
          //  }
            if (ch & CMD_DATA) { // only call callback when CMD_DATA
              EV3UARTMode* cur_mode=get_mode(mode);
              // Serial.printf("current mode %d\n",mode);
              if (cur_mode->hasCallback) {
                cur_mode->myptr(buf,s); // call back
              }
            }
          }
          //Serial.println();
                              

       }
		 }
    } 
  }
}

/**
 * Get the mode object for a specific mode.
**/
EV3UARTMode* EV3UARTEmulation::get_mode(byte mode) {
  return mode_array[mode];
}	

/**
 * Send a single byte t the EV3, as data.
**/
void EV3UARTEmulation::send_data8(byte b) {
  byte bb[1];
  bb[0] = b;
  
  send_cmd(CMD_DATA | current_mode, bb, 1);
}

/**
 * Send a single byte t the EV3, as data.
**/
void EV3UARTEmulation::send_data8(byte *b, int len) {
  
    
  
  send_cmd(CMD_DATA | (log2(len) << CMD_LLL_SHIFT) | current_mode, b, len);
  
}
/**
 * Send a single short to the EV3
**/
void EV3UARTEmulation::send_data16(short s) {
  byte bb[2];
  bb[0] = s & 0xff;
  bb[1] = s >> 8;
  send_cmd(CMD_DATA | (1 << CMD_LLL_SHIFT) | current_mode, bb, 2);
}	

/**
 * Send an array of shorts to the EV3 as data.
 * len must be a power of 2
**/
void EV3UARTEmulation::send_data16(short* s, int len) {
  byte bb[len*2];
  for(int i=0;i<len;i++) {
    bb[2*i] = s[i] & 0xff;
    bb[2*i+1] = s[i] >> 8;   
  }
  send_cmd(CMD_DATA | (log2(len*2) << CMD_LLL_SHIFT) | current_mode, bb, len*2);
}

/**
 * Send a long to the EV3 as data
**/
void EV3UARTEmulation::send_data32(long l) {
  byte bb[4];
  for(int i=0;i<4;i++) {
    bb[i] = (l >> (i * 8)) & 0xff;
    //Serial.printf("%d bb[%d]=%d\n",l,i,bb[i]);
  }
  send_cmd(CMD_DATA | (2 << CMD_LLL_SHIFT) | current_mode, bb, 4);
}

/**
 * Send a float to the EV3 as data
**/
void EV3UARTEmulation::send_dataf(float f) {
  union Data {
    unsigned long l;
    float f;
  } data;
  data.f = f;
  //Serial.printf("send_dataf %f %d\n",f,data.l);
  send_data32(data.l);
}			

/**
 * Send a command to the  EV3
**/
void EV3UARTEmulation::send_cmd(byte cmd, byte* data, byte len) {
  byte checksum = 0xff ^ cmd;
#ifdef DEBUG
      Serial.print("Cmd: ");
	  Serial.print(cmd, HEX);
	  Serial.print(" ");
#endif
  Serial2.write(cmd);
  for(int i=0;i<len;i++) {
    checksum ^= data[i];
#ifdef DEBUG
      Serial.print(data[i],HEX);
	  Serial.print(" ");
#endif
    Serial2.write(data[i]);
  }
  Serial2.write(checksum);
#ifdef DEBUG
      Serial.println(checksum, HEX);
#endif
}

/**
 * Write a single byte to the EV3
**/
void EV3UARTEmulation::send_byte(byte b) {
  Serial2.write(b);
}

/**
 * Utility method to copy a long into a byte array
**/
void EV3UARTEmulation::get_long(unsigned long l, byte* bb) {
  for(int i=0;i<4;i++) {
    bb[i] = (l >> (i * 8)) & 0xff;
  }
}

/**
 * Utility method to return a small power of 2
**/
int EV3UARTEmulation::log2(int val) {
  switch(val) {
    case 1: return 0;
    case 2: return 1;
    case 4: return 2;
    case 8: return 3;
    case 16: return 4;
    case 32: return 5;
    default: return 0;
  }
}

/**
 * Utility method to return the next power of 2 (up to 32)
**/
int EV3UARTEmulation::next_power2(int val) {
  if (val == 1 || val == 2) return val;
  else if (val <= 4) return 4;
  else if (val <= 8) return 8;
  else if (val <= 16) return 16;
  else if (val <= 32) return 32;
  else return 0;
}

/**
 * Utility method to read a byte synchronously
**/
byte EV3UARTEmulation::read_byte() {
  while(!Serial2.available());
  return Serial2.read();
}

/**
 * Get the current mode
**/
byte EV3UARTEmulation::get_current_mode() {
  return current_mode;
}
