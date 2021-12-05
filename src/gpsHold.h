//============================================ COMPASS VALUES =================================
float x,y,z,azimuth,compass_initial,compass_current;
int p_mag=5,yaw_change=0;     //direction control works pretty well with just P controller
//================================================ GPS value ==================================================
float initial_lat=0,initial_lng=0,current_lat=0,current_lng=0,previous_lat=0,previous_lng=0,actual_lat=0,actual_lng=0;

float p_lat=0,p_lng=0,d_lat=0,d_lng=0,p_val=0;      //First adjust p value keeping d as 0 .....Next as oscillation starts  set d value to add drag . 
float Ail_change=0,Ele_change=0,lat_add=0,lng_add=0;
int stat=1,c=0,tim=1,counter=0;
int limit_val=500;