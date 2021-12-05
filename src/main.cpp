#include<Arduino.h>
#include <SFE_BMP180.h>
#include<SPI.h>
#include <ESP32Servo.h>
#include <SBUS2.h>
#include <SimpleKalmanFilter.h>
#include <TinyGPS++.h>
#include "receiver.h"
#include "heightHold.h"
#include "gpsHold.h"
#include "config.h"
#include <HMC5883L.h>
#include <Ticker.h>

#define M_PI		3.14159265358979323846
TinyGPSPlus gps;
SimpleKalmanFilter pressureKalmanFilter(0.5, 0.5, 0.05);
SFE_BMP180 pressure;
HMC5883L compass;
Ticker readGPS;


float baseline; // baseline pressure
double lastAltitude;
double currentAltitude;
double gndPressure;
float baseAlti; //độ cao ban đầu , được sử dụng để hiệu chỉnh độ cao
double setAltitude = 0.5; //để độ cao mặc định là 1m 

 


// create four servo objects 
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
int16_t saveThrottle; // biến này hoạt động để nhớ lại giá trị ga trước khi bị mất tín hiệu với TX



//biến xác nhận drone đang hoạt động
bool activeStatus = false;
ESP32PWM pwm;

  //đặt lại tất cả các giá trị kênh về mặc định
  void resetChannels()
  {
      channel[0] = 1500;
      channel[1] = 1500;
      channel[2] = 1000;
      channel[0] = 1500; 
  }
//////////////////////////////////////// CHẾ ĐỘ HẠ CÁNH ///////////////////////////////////////////

  /*
  Khi máy bay bị mất tín hiệu và các chế độ khác chưa được kích hoạt,
  safeMode này đc kích hoạt, nó sẽ tiến hành giảm ga từ từ từ để hạ cánh  
  */
 bool safeModeStatus = false;
  void safeMode()
  {
    //Xóa hết trạng thái tín hiệu rx
    resetChannels();
    //gán throttle = biến  saveThrottle và từ từ giảm ga dần dần
    channel[2] =saveThrottle;
    //saveThrottle sẽ giảm đi 100 sau mỗi lần hàm này được gọi, nhưng sẽ giới hạn luôn lớn hơn 1200
    if(saveThrottle >1300) saveThrottle-= 10;
    delay(100);
  }


///////////////////////////////////////// Chế độ giữ độ cao ///////////////////////////////////////////
Ticker readBMP;
float find_height()
{
  digitalWrite(trigPin1,LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPin1,HIGH);  
  delayMicroseconds(10); 
  digitalWrite(trigPin1,LOW);   
  temp_height= pulseIn(echoPin1,HIGH);
  temp_height= (temp_height/2)/29.1; 
  if(temp_height>2 && temp_height <200)return(temp_height);   
  else return -1;
}
double getPressure() {
  char status;
  double T,P;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0) {
          return(P);
        }
      } 
    }  
  } 
}
void readAltitude(){
  //tính toán độ cao hiện tại bằng bmp180 và bộ lọc Kalman
  float p = getPressure();
  float altitude = pressure.altitude(p,baseline);
  temp_altitude = (pressureKalmanFilter.updateEstimate(altitude) - baseAlti)*100; //m -> cm
 
}

////////////////////////////////////    COMPASS     /////////////////////////////////////////////
void compass_Configuration() {
    // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
}
void readCompass()
{
   Vector norm = compass.readNormalize();
  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  //float headingDegrees = heading * 180/M_PI; 
  azimuth = heading * 180/M_PI; 
  // // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print( azimuth);
  Serial.println();
}
////////////////////////////////////////     GPS        ///////////////////////////////////////////////
void displayGPS()
{
 
  // if (gps.location.isValid())
  // {
  //   latitude = gps.location.lat();
  //   longtitude = gps.location.lng();
  // } 
  // DEBUG_PRINT(F("  Date/Time: "));
  // if (gps.date.isValid())
  // {
  //   DEBUG_PRINT(gps.date.month());
  //   DEBUG_PRINT(F("/"));
  //   DEBUG_PRINT(gps.date.day());
  //   DEBUG_PRINT(F("/"));
  //   DEBUG_PRINT(gps.date.year());
  // }
  // else
  // {
  //   DEBUG_PRINT(F("INVALID"));
  // }

  // DEBUG_PRINT(F(" "));
  // if (gps.time.isValid())
  // {
  //   if (gps.time.hour() < 10) Serial.print(F("0"));
  //   DEBUG_PRINT(gps.time.hour()+7);
  //   DEBUG_PRINT(F(":"));
  //   if (gps.time.minute() < 10) Serial.print(F("0"));
  //   DEBUG_PRINT(gps.time.minute());
  //   DEBUG_PRINT(F(":"));
  //   if (gps.time.second() < 10) Serial.print(F("0"));
  //   DEBUG_PRINT(gps.time.second());
  //   DEBUG_PRINT(F("."));
  //   if (gps.time.centisecond() < 10) Serial.print(F("0"));
  //   DEBUG_PRINT(gps.time.centisecond());
  // }
  // else
  // {
  //   DEBUG_PRINT(F("INVALID"));
  // }

  // DEBUG_PRINTLN();
}
void gpsReader()
{
   // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial2.available() > 0) gps.encode(Serial2.read());
  if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
    {
      digitalWrite(15,stat);
      if(stat==0)stat=1;
      else stat=0;
      //save previous location point first
      previous_lat=current_lat;
      previous_lng=current_lng;
      current_lat=gps.location.lat();      //These actual values get updated in every 200ms i.e 5hz
      current_lng=gps.location.lng();

    }
}
void setup() {

    // put your setup code here, to run once:
#if defined (ESP32)
  Serial.begin(115200);
  
  Serial.println("Setup SBUS2...");
#endif // ESP32

// BMP180 Pressure sensor start
  if (!pressure.begin()) {
    Serial.println("BMP180 Pressure Sensor Error!");
    while(1); // Pause forever.
  }
  baseline = getPressure();
  for(int i=0; i<10;i++)
{
  float p = getPressure();
  float altitude = pressure.altitude(p,baseline);
  baseAlti += pressureKalmanFilter.updateEstimate(altitude);
}
  baseAlti/=10;
//for UltraSonic Sensor
  pinMode(echoPin1,INPUT);
  pinMode(trigPin1,OUTPUT);
   pinMode(15,OUTPUT); // indicator LED for GPS 
  digitalWrite(15,0 );
  SBUS2_Setup(25,26);     // For ESP32 set RX and TX Pin Number

  compass_Configuration();
  ///Tạo hàm đọc gps bật sau 2s
  readGPS.attach(5, gpsReader);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //Tạo hàm đọc BMP180 sau 1s
  readBMP.attach(1,readAltitude);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);      // Standard 50hz servo
  servo2.setPeriodHertz(50);      // Standard 50hz servo
  servo3.setPeriodHertz(50);      // Standard 50hz servo
  servo4.setPeriodHertz(50);      // Standard 50hz servo
  //servo5.setPeriodHertz(50);      // Standard 50hz servo

servo1.attach(servo1Pin, minUs, maxUs);
servo2.attach(servo2Pin, minUs, maxUs);
servo3.attach(servo3Pin, minUs, maxUs);
servo4.attach(servo4Pin, minUs, maxUs);
//Khởi tạo giá trị ban đầu chô các kênh
channel[0] = 1500;
channel[1] = 1500;
channel[2] = 1000;
channel[3] = 1500;
channel[4] = 1000;
channel[5] = 1000;
}



void loop() {

  //read RX signal  
  if(SBUS_Ready()){                               // SBUS Frames available -> Ready for getting Servo Data
      for(uint8_t i = 0; i<10; i++){
        channel[i] = map(SBUS2_get_servo_data(i),200,1800,1000,2000);        // Channel = Servo Value of Channel 5
      }
    }  
  input_THROTTLE1 = channel[2];
  //nếu vẫn còn tín hiệu từ TX thì biến saveThrottle vs được gán giá trị
  if(safeModeStatus!=true && channel[2] >1100 && channel[2] <2000) saveThrottle = channel[2]; 
 
  //khi bật chế độ arm , biến activeStatus = true
  if(channel[3]<1300 && channel[2] <1050) activeStatus=true;
  //khi tắt chế độ arm , biến activeStatus = false
  if(channel[3]>1700 && channel[2] <1050) 
  {
    activeStatus=false;
    safeModeStatus =false;
  }

//===================================== BẬT CHẾ ĐỘ ARM ==============================================
if(activeStatus)
{
    Serial.print("Active! ");
    readAltitude();

  //**********************************  SAFE MODE *************************************************** 
  //Nếu drone đang hoạt động nhưng bị mất sóng chế độ safeMode sẽ được bật
  if(channel[2] <1000)  safeModeStatus =true;
  //nếu đang trong chế độ saveMode
  if(safeModeStatus)
  {
    safeMode();
    Serial.print("Safe Mode ON -> THROTTLE : ");
    Serial.println(channel[2]);
  }
  //nếu đang trông safeMode nhưng đã có sóng và muốn tắt safeMode , gạt trục Yaw về bên trái
  if(safeModeStatus==true && channel[3]>1700)  safeModeStatus =false;
  
  //******************************* Altitude HOLD MODE ***********************************************
  //Bật chế độ giữ độ cao bằng CH6
  if(channel[5] >1500  ) altitudeHoldStatus =true;
  else 
  {
    error=0; pre_error=0; derror=0;ierror=0;
    ////version 1
    // if(find_height() != -1 )
    // {
    //    //đo độ cao bằng sonar
    //  temp1_height=find_height() ;//đo mốc độ cao
    // }
    // else
    // {
    //   //đo độ cao bằng bmp180
    //   temp1_height=readAltitude() ;//đo mốc độ cao
    // }
    
    // if( temp1_height<1000) Required_height=temp1_height;    //keep updating the required height in normal mode  
    //version 2 : đọc độ cao sau từng 1s
    temp1_height = temp_altitude;
    altitudeHoldStatus = false;
  }
  //Chế độ giữ độ cao được kích hoạt
  if(altitudeHoldStatus) 
  {
    ////version 1
    //  if(find_height() != -1 )
    //   {
    //     //đo độ cao bằng sonar
    //     temp2_height=find_height() ;//đo mốc độ cao
    //   }
    // else
    //   {
    //     //đo độ cao bằng bmp180
    //     temp2_height=readAltitude() ;//đo mốc độ cao
    //   }
    
    // if(temp2_height<1000) current_height=temp2_height;
     //version 2 : đọc độ cao sau từng 1s
     temp2_height = temp_altitude;            
    //PID implementation  //update values
    error=Required_height-current_height;     derror=error-pre_error;     pre_error=error;    ierror+=error;
              
    p_value=p*(error);      I_value=I*(ierror);     D_value=D*(derror);
              
    total=p_value+I_value+D_value;
    input_THROTTLE1+=total;

    // channel[2] =constrain( channel[2]+calculateThrottleAltHold(),1000,1700); // giới hạn giá trị throttle không vượt quá 1500
     Serial.print(" Altitude HOLD ON! ");
     Serial.print(temp2_height);
     Serial.print(" Throttle : ");
     Serial.println(input_THROTTLE1);
     

    // Serial.print(currentAltitude);
    // Serial.println(" meters");
  }

  //****************************** GPS HOLD MODE *****************************************************
    p_val=map(channel[4],1100,2000,0,10);
    if(p_val<0)
     p_val=0;
    d_lat=-1*1000000,d_lng=1*1000000; 
    p_lat=1*p_val*1000000,p_lng=-1*p_val*1000000;

    // while(Serial.available())//While there are characters to come from the GPS
    // {
    //   gps.encode(Serial.read());//This feeds the serial NMEA data into the library one char at a time
    // }
    // if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
    // {
    //   digitalWrite(15,stat);
    //   if(stat==0)stat=1;
    //   else stat=0;
    //   //save previous location point first
    //   previous_lat=current_lat;
    //   previous_lng=current_lng;
    //   current_lat=gps.location.lat();      //These actual values get updated in every 200ms i.e 5hz
    //   current_lng=gps.location.lng();
    // }


    switch (int((channel[4]-1000)/300))//covert output of the channel 5 to 0,1,2,3   //Mode change using channel5
    {
      case 0: //normal mode (initial state should always be front facing north)
              //compass value updated till mode changed              
              readCompass();
              compass_initial=azimuth;  //make sure Front is facing towards north for GPS Hold feature to work
              initial_lat=current_lat;
              initial_lng=current_lng;
              break;
      case 1://yaw hold mode
              //compass value update for direction hold             
              readCompass();
              Serial.print("YAW hold mode ! " + String(azimuth) +" " );
              compass_current=azimuth;
              yaw_change=p_mag*(compass_current-compass_initial);
              channel[3]+=yaw_change;  
              initial_lat=current_lat;
              initial_lng=current_lng;
              break;
     case 2:case 3: //GPS Hold Mode
              //compass value update  for direction hold             
              readCompass();
              Serial.print("GPS hold mode ! ");
               Serial.println(String(current_lat,6) + " " + String(current_lng,6));
              compass_current=azimuth;
              yaw_change=p_mag*(compass_current-compass_initial);
              channel[3]+=yaw_change;  

              
              //GPS position based calculations for Aileron and Elevation to control the left/right, front/back motion
              Ail_change=p_lng*(current_lng-initial_lng) + d_lng*(current_lng-previous_lng);
              Ele_change=p_lat*(current_lat-initial_lat) + d_lat*(current_lat-previous_lat);
              limit_val=200;
              //Limit the maximum corrections to limit_val=300, so that the bot does not tilt too much
              if(abs(Ail_change)>limit_val){
               Ail_change=limit_val*(abs(Ail_change)/Ail_change);
               }
              if(abs(Ele_change)>limit_val){
               Ele_change=limit_val*(abs(Ele_change)/Ele_change);
               }

              
              channel[0]+=Ail_change;  
              channel[1]+=Ele_change;
              break;
    }
}
 




    // Serial.print("Servo Channels");
    // for(uint8_t i = 0; i<6; i++){
    //   Serial.print(" ");
    //   Serial.print(i);
    //   Serial.print(": ");
    //   Serial.print(channel[i]);
    // }
    Serial.println();

   /// điều khiển các servo esc
    servo1.writeMicroseconds(channel[0]); // ROLL
    servo2.writeMicroseconds(channel[1]); //PITCH
    servo3.writeMicroseconds(input_THROTTLE1); //THROTTLE
    servo4.writeMicroseconds(channel[3]); //YAW
    delay(80);
}

