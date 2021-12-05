//================================ HEIGHT HOLD =============================================================
//PID Variables 
float p=1.5;float p_value=0;
float I=0.00;float I_value=0;
float D=1;float D_value=0;
float total=0;
float error=0;float pre_error=0;float derror=0;float ierror=0;


bool altitudeHoldStatus = false;
float temp_altitude=0;
// float KP_altitude = 400;
// float KD_altitude = 30; 
// float KI_altitude = 10;
// float altitude_error =0;
// float lastAltitude_error =0;
// float P,I,D;
//initial value 1000

float input_THROTTLE=1000;//channel 3 input
float input_THROTTLE1;
//variables for UltraSonic Sensor
int echoPin1=32,trigPin1=33;
int duration1;
float current_height = 0.0;
float temp_height = 0.0;  float temp1_height = 0.0; float temp2_height = 0.0;
float Required_height=0;//(in cm)