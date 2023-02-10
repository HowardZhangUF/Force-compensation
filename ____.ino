#include "imu_data_decode.h"
#include "packet.h"
#include<Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
uint32_t old_frame_ctr = 0;
double p00 =      -51.37 ;
double       p10 =       1.376 ;
double       p01 =       35.32 ;// (34.61, 36.04)
double       p20 =     -0.2455 ;//(-0.2641, -0.2269)
double       p11 =       1.541 ;// (1.378, 1.705)
double       p02 =     -0.4436 ;// (-0.6076, -0.2796)
double       p30 =     0.01736 ;//; (0.01634, 0.01838)
double       p21 =    -0.08198 ;//(-0.09097, -0.07298)
double       p12 =    0.008287 ;//(-0.02824, 0.04481)
double       p40 =  -0.0004016 ;//(-0.0004245, -0.0003787)
double       p31 =   0.0009451 ;// (0.000787, 0.001103)
double       p22 =  -0.0006841 ;//(-0.002341, 0.000973)
double       p00a =       1.477 ;// (1.473, 1.482);
double       p10a =     -0.1014 ;//(-0.1038, -0.09888);
double       p01a =     0.02936 ;//(0.0291, 0.02963);
double       p20a =     0.01234 ;//(0.01187, 0.01281);
double       p11a =   -0.001219 ;// (-0.001283, -0.001155);
double       p02a =   8.392e-06 ;//(3.427e-06, 1.336e-05);
double       p30a =  -0.0007103 ;//(-0.0007447, -0.0006758);
double       p21a =   7.226e-05 ;//(6.697e-05, 7.755e-05);
double       p12a =  -4.372e-07 ; //(-1.414e-06, 5.393e-07);
double       p40a =   1.546e-05 ;// (1.463e-05, 1.629e-05);
double       p31a =  -1.079e-06 ;//(-1.242e-06, -9.157e-07);
double       p22a =   2.336e-08; //(-1.851e-08, 6.524e-08);

double w = 27.6;
 
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  //LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();        
  lcd.setCursor(0,0);
  
  imu_data_decode_init();
}

void loop() {
  /*--------馬達程式--------------*/
  
  Serial2.println("k");   //對 TX 送出字元 en

  double f = w*sin((receive_imusol.eul[1]+0.64)*3.14/180);
  double x = 0.038;
  double kt  = p00a + p10a*x + p01a*f + p20a*pow( x, 2 ) + p11a*x*f + p02a*pow( f, 2 ) + p30a*pow( x, 3 ) + p21a*f*pow( x, 2 ) + p12a*x*pow( f, 2 ) + p40a*pow( x, 4 ) + p31a*f*pow( x, 3 ) + p22a*pow( x, 2 )*pow( f, 2 );    //zz 也是 25×2 的矩陣  
  double A = (f*0.02)/kt;
  char B[10];
  dtostrf(A,6,3, B);//浮點數轉字串
  String force = String("t") + B;
  Serial2.println(force);
  Serial.print("電流: ");
  Serial.print(B);
  Serial.print("     角度: ");
  Serial.println((receive_imusol.eul[1]+0.64)*3.14/180);
  
  /*----------imu程式----------*/
  while (Serial3.available()) {
    char c = Serial3.read();
    packet_decode(c);
  }


  if (frame_count > old_frame_ctr) {
    old_frame_ctr = frame_count;
    if (receive_gwsol.tag != KItemGWSOL) {
     /* Serial.println(F("\t三軸:"));
      Serial.print(receive_imusol.eul[0]);
      Serial.print(',');
      Serial.print(receive_imusol.eul[1]);
      Serial.print(',');
      Serial.println(receive_imusol.eul[2]);*/
      //Serial.println("time per data:");
      //Serial.println(millis());


      //LCD顯示
      lcd.setCursor(0, 0);
      lcd.print(F("X:"));
      lcd.print(receive_imusol.eul[0]);

      lcd.setCursor(8, 0);
      lcd.print(F("Y:"));
      lcd.print(receive_imusol.eul[1]);

      lcd.setCursor(0, 1);
      lcd.print(F("Z:"));
      lcd.print(receive_imusol.eul[2]);      

      lcd.setCursor(12,1);
      lcd.print(F("axis"));
;    }
  }
  
}
