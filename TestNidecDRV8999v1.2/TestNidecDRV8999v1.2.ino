//#include <Button.h>
#include "FuntionSPI.h"

void setup() {
  lcd.init(); 

  pinMode(Fault01, INPUT_PULLUP); 
  pinMode(Fault02, INPUT_PULLUP); 
  pinMode(Fault03, INPUT_PULLUP); 
  pinMode(Fault04, INPUT_PULLUP); 
  pinMode(Fault05, INPUT_PULLUP); 
  pinMode(Fault06, INPUT_PULLUP); 
  pinMode(Fault07, INPUT_PULLUP); 
  pinMode(Fault08, INPUT_PULLUP); 
  pinMode(Fault09, INPUT_PULLUP); 
  pinMode(Fault10, INPUT_PULLUP); 

  pinMode(ExtX0, INPUT_PULLUP); 
  pinMode(ExtX1, INPUT_PULLUP);
  pinMode(ExtX2, INPUT_PULLUP);
  pinMode(ExtX3, INPUT_PULLUP);
  pinMode(ExtX4, INPUT_PULLUP);
  pinMode(ExtX5, INPUT_PULLUP);
  pinMode(ExtX6, INPUT_PULLUP);
  pinMode(ExtX7, INPUT_PULLUP);
  pinMode(ExtX8, INPUT_PULLUP);
  pinMode(ExtX9, INPUT_PULLUP);
  pinMode(ExtX10, INPUT_PULLUP); 
  pinMode(ExtX11, INPUT_PULLUP);
  pinMode(ExtX12, INPUT_PULLUP);
  pinMode(ExtX13, INPUT_PULLUP);
  pinMode(ExtX14, INPUT_PULLUP);
  pinMode(ExtX15, INPUT_PULLUP);
  pinMode(ExtX16, INPUT_PULLUP);
  pinMode(ExtX17, INPUT_PULLUP);
  pinMode(ExtX18, INPUT_PULLUP);
  pinMode(ExtX19, INPUT_PULLUP);
  pinMode(ExtX20,INPUT_PULLUP);
  pinMode(ExtX21, INPUT_PULLUP);
  pinMode(ExtX22, INPUT_PULLUP);
  pinMode(ExtX23, INPUT_PULLUP);
  pinMode(ExtX24, INPUT_PULLUP);
  pinMode(ExtX25, INPUT_PULLUP);
  pinMode(ExtX26, INPUT_PULLUP);
  pinMode(ExtX27, INPUT_PULLUP);
  
  pinMode(ExtY7, OUTPUT); 
  pinMode(ExtY6, OUTPUT); 
  pinMode(ExtY5, OUTPUT); 
  pinMode(ExtY4, OUTPUT);
  digitalWrite(ExtY4,HIGH);
  digitalWrite(ExtY5,HIGH);
  digitalWrite(ExtY6,HIGH);
  digitalWrite(ExtY7,HIGH);

  pinMode(SCS01, OUTPUT); 
  pinMode(SCS02, OUTPUT); 
  pinMode(SCS03, OUTPUT); 
  pinMode(SCS04, OUTPUT); 
  pinMode(SCS05, OUTPUT); 
  pinMode(SCS06, OUTPUT); 
  pinMode(SCS07, OUTPUT); 
  pinMode(SCS08, OUTPUT); 
  pinMode(SCS09, OUTPUT); 
  pinMode(SCS10, OUTPUT); 
  
  pinMode(SDI_PIN, OUTPUT); 
  pinMode(SCK_PIN, OUTPUT);
  pinMode(SDO_PIN, INPUT);
  pinMode(SDO_PIN, INPUT);

  pinMode(SLEEP10,OUTPUT);
  pinMode(SLEEP09,OUTPUT);
   
  pinMode(VREF,OUTPUT);
 
  pinMode(STEP,OUTPUT);
  pinMode(MOTOR_ON,OUTPUT);
  pinMode(DIR,OUTPUT);   
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(RL_VM, OUTPUT);
  
/*  dataRED = 0x00; //11111111
  digitalWrite(DIR,HIGH);
  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin,MSBFIRST, dataRED);
  digitalWrite(latchPin, 1);*/
  
 // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Bee Eyes");
  lcd.setCursor(6,1);
  lcd.print("Automation");
  
  delay (100);
  digitalWrite(RL_VM,HIGH);
  Serial.begin(9600);
  delay(100);

  //digitalWrite(SLEEP10,1);//enable_ON;
  //digitalWrite(MOTOR_ON,0);//dis_out_ON;
  //Viết thông số xuống Driver
  for(int i=1;i<=10;i++)
  {
    Serial.println("Write para Driver " +i);
     Sleep(i,true);
     WriteFullStep(i);
     delay(500);
     Sleep(i,false);
  }
  
 //Write Full Step for Driver
 

 //Set Value VREF
  delay(10);
  analogWrite(VREF,ValVREF);///VRef=0.77;   56 cho board tu ngoai bien,39 cho boarr nguoc o giua
  delay(500); 
    //frq_start=1800;frq_max=1800;pul_set=1354;pul_acc_set=0;time_delay=100;

     
     pul_max=pul_set*2;
     pul_acc=pul_acc_set*2;
     
 /* if(pul_acc_set==0){pul_acc_set=1;}
     pre_count_low=(long)(65535-(((1/(float)frq_start)/2)/0.0000000625))+90;
     pre_count_high=(long)(65535-(((1/(float)frq_max)/2)/0.0000000625))+90;
     pre_count_diffence=pre_count_high-pre_count_low;
     time_pulse=pre_count_diffence/pul_acc_set;
     pul_stop=pul_max-pul_acc;
     pre_count=pre_count_low;
     Serial.print("pre_count_low:");
      Serial.println(pre_count_low);
      Serial.print("pre_count_high:");
      Serial.println(pre_count_high);
      Serial.print("pre_count_diffence:");
      Serial.println(pre_count_diffence);
      Serial.print("time_pulse:");
      Serial.println(time_pulse);
       Serial.print("pul_stop:");
      Serial.println(pul_stop);
     digitalWrite(SLEEP10,0);//enable_OFF;
     delay(100);
     digitalWrite(SLEEP10,1);//enable_ON;
    delay(100);//reset err
    SPI_writeRegister(SPI_REG_CTRL4,n_fault_setting,SCS10);

 check_err();*/
    for(int i=1;i<=10;i++)//Chạy từng tự Driver 1 đến 5
  {
      Sleep(i,true);
  }
}
int No=1;
void loop(void) {
  
  int ValueADC=  analogRead(ADC_VM);
  Serial.print("ADC VALUE : ");
  Serial.println(ValueADC);
  
  bool InPut= digitalRead(ExtX1);//Đổi chân Input
  Serial.print("Input : ");
  Serial.println(InPut);

  for(int i=1;i<=5;i++)//Chạy từng tự Driver 1 đến 5
  {
     // Sleep(i,true);
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
  // Sleep(i,false);
 
  }
}
