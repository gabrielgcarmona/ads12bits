#include <Wire.h>
#include <Adafruit_ADS1015.h>

// Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */

uint8_t i2cAddress=0x48;
uint16_t result = 0;

static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(ADS1015_REG_POINTER_CONVERT);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)2);
  return ((Wire.read() << 8) | Wire.read());  
}

static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)(value>>8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

void captura()
{
  sei();
  result = readRegister(i2cAddress, ADS1015_REG_POINTER_CONVERT) >> 4;
  Serial.println(result*3);
}

void setup(void) 
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  //cli(); //stop interrupts
  

  uint16_t config = ADS1015_REG_CONFIG_CQUE_1CONV    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_3300SPS   | // 1600 samples per second (default)
                    GAIN_TWOTHIRDS | //Ganho
                    ADS1015_REG_CONFIG_MUX_SINGLE_0 | //Mux porta 0
                    ADS1015_REG_CONFIG_MODE_CONTIN;
  // Escrevendo no registrador de configuração                  
  writeRegister(i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Setando pino ready
  writeRegister(i2cAddress, ADS1015_REG_POINTER_HITHRESH, 0x4000);
  writeRegister(i2cAddress, ADS1015_REG_POINTER_LOWTHRESH, 0);
  //result = readRegister(i2cAddress, ADS1015_REG_POINTER_CONVERT) >> 4;
  
  /*TCCR2B = (TCCR2B & B11111000) | 0x03;
  TIMSK2 = (TIMSK2 & B11111110) | 0x01;*/
  //attachInterrupt(0,captura,RISING);

  //set timer1 interrupt at 500Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1Hz increments
  OCR1A = 1250;// = (16*10^6) / (8*500) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bits for 8 prescaler
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  //sei(); //allow interrupts               
  
  //Serial.println("Getting single-ended readings from AIN0..3");
  //Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
  //ads.begin();
}

void loop(void) 
{
  //result = readRegister(i2cAddress, ADS1015_REG_POINTER_CONVERT) >> 4;
  //Serial.println(result*3);
  /*
  int16_t adc0;//, adc1, adc2, adc3;

  adc0 = ads.readADC_SingleEnded(0);
  
  /*Serial.print("AIN0: "); Serial.println(adc0*3);*/
  
}
void sendint16(uint16_t mensagem){
  uint16_t mask   = B11111111;          // 0000 0000 1111 1111
  uint8_t first_half   = mensagem >> 8;   // >>>> >>>> 0001 0110
  uint8_t sencond_half = mensagem & mask; // ____ ____ 0100 0111
Serial.write(sencond_half);
Serial.write(first_half);

}

ISR(TIMER1_COMPA_vect){
   sei();
   result = readRegister(i2cAddress, ADS1015_REG_POINTER_CONVERT) >> 4;
   //Serial.println(result*3);
   sendint16(result);
}
