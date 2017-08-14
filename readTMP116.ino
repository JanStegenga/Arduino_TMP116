/*  readTMP116.ino
 *      * Sample code for reading out a Texas Instruments TMP116(N) temperature sensor using an Arduino.
 *      * The code simply puts a device in periodic/alarm mode. 
 *      * The code is created on an Arduino M0 platform.
 *   
 *  Copyright (c) 2017, Jan Stegenga, EVAbits BV, <jan@evabits.com>
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#include <Wire.h>
#include <arduino.h>

uint8_t TMPAddressGND = 0x48;   // 1001000 -> 7 bit addresses with ADD0 pin connected to GND/VCC/SDA/SCL
uint8_t TMPAddressVCC = 0x49;   // 1001001
uint8_t TMPAddressSDA = 0x4A;   // 1001010
uint8_t TMPAddressSCL = 0x4B;   // 1001011
uint8_t TMPAddress    = TMPAddressGND;

uint8_t Reg_T         = 0x00;   // register addresses for the commonly used ones
uint8_t Reg_C         = 0x01;
uint8_t Reg_HL        = 0x02;
uint8_t Reg_LL        = 0x03;
uint8_t Reg_ID        = 0x0F;   // device ID
uint8_t Reg_current   = 0x00;   // keep track of the current register pointer

uint8_t mode          = 2;      // MOD  0: continuous conversion, 1: shutdown, 2: continuous conversion, 3: one-shot
uint8_t conv          = 4;      // CONV 0: afap, 1: 0.125s, 2: 0.250s, 3: 0.500s, 4: 1s, 5: 4s, 6: 8s, 7: 16s
uint8_t avg           = 0;      // AVG  0: no_avg (15.5ms), 1: 8 avg(125ms), 2: 32 avg(500ms), 3: 64 avg(1000ms) 
uint8_t flags         = 0;      // FLAGS : last three bits representing: [2: hi alert, 1: lo alert, 0: data ready]

/*
 * sets the register pointer if necessary
 * returns 0 if successful
 */
byte SetPointer( uint8_t reg ) {
  if( Reg_current != reg ) {
    Wire.beginTransmission( TMPAddress );
    uint8_t bytes_written = Wire.write( reg );
    byte return_code = Wire.endTransmission();
    //SerialUSB.println( "bytes written " + String( bytes_written ) + " to " + String( TMPAddress, BIN ) + " with return code " + String( return_code ) );
    Reg_current = reg;
    return return_code;
  }
}

/*
 * reads a T, HL or LL register
 * returns a float with the converted value
 */
float ReadRegister( uint8_t reg ){
  SetPointer( reg );
  uint8_t bytes_returned = Wire.requestFrom( TMPAddress, 2 );
  //SerialUSB.println( "Read " + String( bytes_returned ) + " from register " + String( reg ) );
  if( bytes_returned == 2){ 
    uint8_t c[bytes_returned];
    for( int i=0; i<bytes_returned; i++ ) {    // slave may send less than requested{ 
      c[i] = Wire.read();
      //SerialUSB.println( "\t" + String( c[i], BIN ) + "\t" + String( c[i], DEC ) );
    }
    return float( c[0]*256 + c[1] )*0.0078125;
  }
  return bytes_returned == 2;
}

/*
 * sets a HL or LL register
 * returns 0 if successful
 */
byte SetRegister( uint8_t reg, float value ){
  //set control register-> addr+0, register address, value(s)
  int16_t val = (int16_t) ( value/0.0078125 );
  SerialUSB.println( String(value) + " converted to: " + String( val, BIN ) ); 
  byte data[3];
  data[0] = reg;
  data[1] = (byte) ( val >> 8 ) & 0xFF;
  data[2] = (byte) val & 0xFF;
  SerialUSB.println( "Setting " + String( reg ) + " register with " + String( data[1], BIN ) + String( data[2], BIN ) ); 
  Wire.beginTransmission(TMPAddress);
  uint8_t bytes_written = Wire.write( data, 3 );
  byte return_code = Wire.endTransmission();
  SerialUSB.println( "bytes written " + String( bytes_written ) + " to register " + String( reg, BIN ) + " with return code " + String( return_code ) );
  Reg_current = reg;
  return return_code;
}

/*
 * Sets the control register
 * MOD  0: continuous conversion, 1: shutdown, 2: continuous conversion, 3: one-shot
 * CONV 0: afap, 1: 0.125s, 2: 0.250s, 3: 0.500s, 4: 1s, 5: 4s, 6: 8s, 7: 16s
 * AVG  0: no_avg (15.5ms), 1: 8 avg(125ms), 2: 32 avg(500ms), 3: 64 avg(1000ms) 
 * returns 0 if successful
 */
byte SetControlRegister( uint8_t MOD, uint8_t CONV, uint8_t AVG ){
  uint16_t c_reg = 0x00;
  //SerialUSB.println( "MOD bits: " + String( ( MOD  & 0x03 ) << 10, BIN ) );
  c_reg = c_reg | ( MOD  & 0x03 ) << 10;     //2 bits of mod go to position 11:10
  c_reg = c_reg | ( CONV & 0x07 ) << 7;      //3 of conv go to 9:7
  c_reg = c_reg | ( AVG  & 0x03 ) << 5;      //2 of AVG go to 6:5
  c_reg = c_reg | 1 << 3;                    //always in make alert pin active high
  //set control register-> addr+0, register address, value(s)
  uint8_t data[3];
  data[0] = Reg_C;
  data[1] = uint8_t( c_reg >> 8 );
  data[2] = uint8_t( c_reg && 0xF );
  Wire.beginTransmission( TMPAddress );
  uint8_t bytes_written = Wire.write( data, 3 );
  byte return_code = Wire.endTransmission();
  SerialUSB.println( "Writing control register with: " + String( c_reg , BIN ) ); 
  SerialUSB.println( "bytes written " + String( bytes_written ) + " to register " + String( Reg_C, BIN ) + " with return code " + String( return_code ) );
  Reg_current = Reg_C;
  return return_code;
}

/*
 * reads the control register
 * FLAGS:   uint8_t with last three bits representing: [2: hi alert, 1: lo alert, 0: data ready]
 * returns 0 if successful
 */
byte ReadControlRegister( uint8_t *MOD,  uint8_t *CONV,  uint8_t *AVG,  uint8_t *FLAGS ){
  SetPointer( Reg_C );
  byte bytes_returned = Wire.requestFrom( TMPAddress, 2 );
  //SerialUSB.println( "Read " + String( bytes_returned ) + " from register " + String( Reg_C ) );
  if( bytes_returned == 2){ 
    uint8_t c[bytes_returned];
    for( int i=0; i<bytes_returned; i++ ) {    // slave may send less than requested{ 
      c[i] = Wire.read();
      //SerialUSB.println( "\t" + String( c[i], BIN ) );
    }
    uint16_t c_reg = uint16_t( c[0] ) << 8 | c[0] ;
    //SerialUSB.println( c_reg, BIN );
    *FLAGS = ( c_reg >> 13 ) & 0x07;
    *MOD   = ( c_reg >> 10 ) & 0x03;
    *CONV  = ( c_reg >> 7  ) & 0x07;
    *AVG   = ( c_reg >> 5  ) & 0x03; 
  }
  return bytes_returned == 2;
}

/*
 * ---------------------------------------------------------------------------------------------
 */
void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  Wire.begin();
  delay(3000);
  SerialUSB.println( "start program" );
  SetRegister( Reg_HL, 29.25 );
  SetRegister( Reg_LL, 19.00 );
  if( SetControlRegister( mode, conv, avg ) ){
    SerialUSB.println( "Control Register Set" );
  }
}

/*
 * ---------------------------------------------------------------------------------------------
 */
void loop() {
  // put your main code here, to run repeatedly:

  ReadControlRegister( &mode, &conv, &avg, &flags );
  // SerialUSB.println( "mode:        " + String( mode ) );
  // SerialUSB.println( "conv:        " + String( conv ) );
  // SerialUSB.println( "avg:         " + String( avg  ) );
  // SerialUSB.println( "flags:       " + String( flags) );
 
  if( (flags >> 2) & 0x01 ) {                             //hi alert
    SerialUSB.println( "hi alert" );
  }
  if( (flags >> 1) & 0x01 ){                              //lo alert
    SerialUSB.println( "lo alert" );
  }
  if( flags & 0x01 ){                                     //data ready
    SerialUSB.println( String( millis()/1000 ) + "\ttemperature: \t" + String( ReadRegister( Reg_T ) ) );
  }
  //SerialUSB.println( "loop" );
  delay( 100 );
}

/*
 * ---------------------------------------------------------------------------------------------
 */





