#include "Define.h"
void SPI_writeRegister(uint8_t address, uint16_t data,int SCS)
{
    volatile uint16_t reg_value = 0;
    uint8_t msb_byte=0;
    uint8_t lsb_byte=0;

  //reg_value &=~ SPI_RW_BIT_MASK;                                        /*Clear R/W bit*/
  reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK);         /* Adding register address value*/
  reg_value |= ((data << SPI_DATA_POS) & SPI_DATA_MASK);                  /*Adding data value */

  digitalWrite(SCK_PIN, LOW);
  delay(1);
  digitalWrite(SCS,LOW);
  delay(1);
  msb_byte = (uint8_t)((reg_value>>8) & 0xFF);
  //Serial.print("MSB= ");Serial.println(msb_byte, BIN);
  for (uint8_t i = 0; i < 8; i++)  
  {
    digitalWrite(SDI_PIN, msb_byte & (1 << (7 - i)));
    delay(1);    
    digitalWrite(SCK_PIN, HIGH);  // LOW
    delay(1);
    digitalWrite(SCK_PIN, LOW); // HIGH
    delay(1);         
  } 

  lsb_byte = (uint8_t)(reg_value & 0xFF);
  //Serial.print("LSB= ");Serial.println(lsb_byte, BIN);
  for (uint8_t i = 0; i < 8; i++)  
  {
    digitalWrite(SDI_PIN, lsb_byte & (1 << (7 - i)));
    delay(1);  
    digitalWrite(SCK_PIN, HIGH);  // LOW
    delay(1);
    digitalWrite(SCK_PIN, LOW); // HIGH
    delay(1);         
  }
  
  delay(1);
  digitalWrite(SCS,HIGH);
  delay(5);
}
uint16_t SPI_readRegister(uint8_t address,int SCS)
{
    volatile uint16_t reg_value = 0;
  volatile uint8_t dataMSB    = 0;
  volatile uint8_t dataLSB    = 0;
  volatile uint8_t rxMSB    = 0;
  volatile uint8_t rxLSB    = 0;
  reg_value |= SPI_RW_BIT_MASK;                                      /*Set R/W bit*/
  reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK);    /* Configure register address value*/
  digitalWrite(SCK_PIN, LOW);
digitalWrite(SCS,LOW);
  //nscs_LOW;//SPI set
   dataMSB = (uint8_t)((reg_value>>8) & 0xFF);
  //Serial.print("MSB= ");Serial.println(msb_byte, BIN);
  //transfer msb byte
  for (uint8_t i = 0; i < 8; i++)  
  {
    digitalWrite(SDI_PIN, dataMSB & (1 << (7 - i)));
    
    delay(1);    
    digitalWrite(SCK_PIN, HIGH);  // LOW
    
    delay(1);
    rxMSB = (rxMSB|(digitalRead(SDO_PIN)<<(7-i)));
    digitalWrite(SCK_PIN, LOW); // HIGH
    delay(1);         
  } 

  //transfer lsb byte
    dataLSB = (uint8_t)(reg_value & 0xFF);
  //Serial.print("LSB= ");Serial.println(lsb_byte, BIN);
  for (uint8_t i = 0; i < 8; i++)  
  {
    digitalWrite(SDI_PIN, dataLSB & (1 << (7 - i)));
    delay(1);  
    digitalWrite(SCK_PIN, HIGH);  // LOW
    
    delay(1);
    rxLSB = (rxLSB|(digitalRead(SDO_PIN)<<(7-i)));
    Serial.println(digitalRead(SDO_PIN));
    digitalWrite(SCK_PIN, LOW); // HIGH
    delay(1);         
  }
  // Serial.print("LSB= ");Serial.println(rxLSB, BIN);
  /// wait receiver lsb byte
  delay(1);
  digitalWrite(SCS,HIGH);
  reg_value = ((((rxMSB<<8) | rxLSB)& SPI_DATA_MASK)>>SPI_DATA_POS); 
  
  return (reg_value);                                    
}

void check_err()
{

    error_code= SPI_readRegister(SPI_STATUS_CTRL1,SCS10);
    Serial.print("error_code:");Serial.println(error_code,BIN);

  DRV8889.openload = (uint16_t)error_code & 0x0001;
  DRV8889.overheat = (uint16_t)error_code & 0x0002;
  DRV8889.overcurrent = (uint16_t)error_code & 0x0008;
  DRV8889.undervol = (uint16_t)error_code & 0x0030;
  DRV8889.spi_err= (uint16_t)error_code & 0x0040;
  DRV8889.fault_bit= (uint16_t)error_code & 0x0080;

}
void Step(int Steps,int _delay)
{
  for(int i=0;i<Steps;i++)
   {
     digitalWrite(STEP,HIGH);
      delayMicroseconds(_delay);
      digitalWrite(STEP,LOW);
      delayMicroseconds(_delay);
   }
}
void IC_74595(int i)
{
  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin,MSBFIRST, dataOutPut[i]);
  digitalWrite(latchPin, 1);
}
void WriteFullStep(int No)
{
int SCS=0;
  switch(No)
  {
    case 1:
     SCS=SCS01;
    break;
    case 2:
   SCS=SCS02;
    break;
    case 3:
     SCS=SCS03;
    break;
    case 4:
   SCS=SCS04;
    break;
    case 5:
     SCS=SCS05;
    break;
    case 6:
    SCS=SCS06;
    break;
    case 7:
   SCS=SCS07;
    break;
    case 8:
     SCS=SCS08;
    break;
    case 9:
    SCS=SCS09;
    break;
    case 10:
   SCS=SCS10;
    break;
    
  }
    SPI_writeRegister(SPI_REG_CTRL4,n_fault_setting,SCS);
  delay(100);
  SPI_writeRegister(Tblank_CTRL1, tblank_860,SCS);
  delay(10);
  SPI_writeRegister(decay_CTRL2, smart_ripple,SCS);
  delay(10);
  delay(10);
  SPI_writeRegister(SPI_REG_CTRL3,MODE_FULL_STEP_100,SCS);
   SPI_writeRegister(SPI_REG_CTRL4,n_fault_setting,SCS);
}
void Sleep (int No,bool IsOn)
{
  if(IsOn)
  {
    
  
  switch(No)
  {
    case 1:
     IC_74595(0);
    break;
    case 2:
    IC_74595(1);
    break;
    case 3:
    IC_74595(2);
    break;
    case 4:
    IC_74595(3);
    break;
    case 5:
    IC_74595(4);
    break;
    case 6:
    IC_74595(5);
    break;
    case 7:
    IC_74595(6);
    break;
    case 8:
    IC_74595(7);
    break;
    case 9:
    IC_74595(8);
    digitalWrite(SLEEP09,HIGH);
    break;
    case 10:
    IC_74595(8);
    digitalWrite(SLEEP10,HIGH);
    break;
    
  }
  }
  else
  {
     IC_74595(8);
    digitalWrite(SLEEP10,0);
     digitalWrite(SLEEP09,0);
  }
}
void RunStepper(int No,int Steps,int _delay )
{
  
   Sleep(No,true);
   digitalWrite(DIR,HIGH);
   Step(Steps,_delay);
   check_err();
   digitalWrite(MOTOR_ON,1);
   delay(1000);
   digitalWrite(MOTOR_ON,0);
   digitalWrite(DIR,LOW);
   Step(Steps,_delay);
   digitalWrite(MOTOR_ON,1);
   delay(1000);
   digitalWrite(MOTOR_ON,1);
   delay(100);
   Sleep(No,false);
}
