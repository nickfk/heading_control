
/*
///this program test the communication between the beaglebone black and glider and mbed
//this program log the sonar data from mbed and talk to glider via serial.
// April 15, 2015. working with MBED_BEALGE_MICRON///
// April 15, 2015. communication tested with MBED and glider simulator. refinement is needed.
//May 5, 2015 added range extraction function
//May 5, 2015 adding heading controller in the main loop
//May 12, 2015 working ok with mbed and MATLAB (AUV iceberg map_beagle)
//May 15, 2015 copied from "talk_to_glider_mbed0505.cpp"
//May 20, 2015 setting up  for glider simulator
//May 21,2015 working with glider simulator and matlab simulation
//May 22, 2015 re-organize the program for real sonar sampling and glider simulator and 
//June 2,2015 add c_heading start when depth over 20 meters otherwise the c_heading=yaw
//June 3,2015 change the R_TO_VEHICLE INTO 180-current_angle to fake the retunrs.
//June 4,2015 change the R_TO_VEHICLE INTO 270-current_angle
//	      switch the m_x, m_y,   glider: m_x->east m_y->north   heading controller-> m_x->north m_y->east
// 	      log glider.roll
// 	      ignore the last bin which is 0A -> end-flag
// 	      changed loging float precision
//	      Added receiving initial heading from glider sci
//June 10, 2015 only use heading_controller when the sonar is scanning doward
//June 25, 2015 updated rotation matrix
//June 29, 2015 added conditions for heading controller
//		receive chars from mbed at the beginning to define the minimum depth, sonar sectors.
		the conditions will be used to activate the heading controller.
	        changed minimum range to be 10 meters.
//June 30, 2015 define the minimum depth sonar sectors from the glider
//July 13, 2015  update ambiguity flag and controls
//September 1, 2015 change the bug of ambiguity flag
//September 11, 2015 GitHub test
//////sepeotoefgjf
*/
#include<iostream>
#include<stddef.h>
#include<fstream>
#include<strings.h>
#include"fcntl.h"
#include<termios.h>
#include<stdio.h>
#include<time.h>
#include"sys/types.h"
#include"sys/time.h"
#include<unistd.h>
#include <termios.h>
#include <string>
#include <math.h>

using namespace std;
#define BONEPATH	"/sys/devices/bone_capemgr.9/slots"  
#define PI 3.14159265
int pauseNanoSec(long nano);
void pauseSec(int sec);
int fd_glider;			//file number of glider serial
int fd_mbed;			//file number of mbed serial
char serial_glider[30] = "/dev/ttyO1";   //glider serial path
char serial_mbed[30]="/dev/ttyO2";
FILE *uart;

int gettimeofday(struct timeval *tv, struct timezone *tz);

struct termios uart1,old;

int pauseNanoSec(long nano)
{
	struct timespec tmr1,tmr2;

	tmr1.tv_sec = 0;
	tmr1.tv_nsec = nano;

	if(nanosleep(&tmr1,&tmr2) < 0)
	{
		printf("Nano second pause failed\n");
		return -1;
	}
}

void pauseSec(int sec)
{
	time_t now,later;

	now = time(NULL);
	later = time(NULL);

	while((later - now) < (double)sec)
		later = time(NULL);
}
/////////////////////////OPEN SERIAL UART1 for glider//////////////////
void serial_init_glider()
{

	uart = fopen(BONEPATH, "w");
	fseek(uart,0,SEEK_SET);

	fprintf(uart, "BB-UART1");
	fflush(uart);
	fclose(uart);

	//open uart1 for tx/rx
	fd_glider = open(serial_glider, O_RDWR | O_NOCTTY);
	if(fd_glider < 0) printf("port failed to open\n");

	//save current attributes
	tcgetattr(fd_glider,&old);
	bzero(&uart1,sizeof(uart1)); 

	uart1.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	uart1.c_iflag = IGNPAR | ICRNL;
	uart1.c_oflag = 0;
	uart1.c_lflag = 0;

	uart1.c_cc[VTIME] = 0;
	uart1.c_cc[VMIN]  = 1;

	//clean the line and set the attributes
	tcflush(fd_glider,TCIFLUSH);
	tcsetattr(fd_glider,TCSANOW,&uart1);
}

/////////////////////////OPEN SERIAL UART2 for MBED//////////////////
void serial_init_mbed()
{

	uart = fopen(BONEPATH, "w");
	fseek(uart,0,SEEK_SET);

	fprintf(uart, "BB-UART2");
	fflush(uart);
	fclose(uart);

	//open uart1 for tx/rx
	fd_mbed = open(serial_mbed, O_RDWR | O_NOCTTY);
	if(fd_mbed < 0) printf("port failed to open\n");

	//save current attributes
	tcgetattr(fd_mbed,&old);
	bzero(&uart1,sizeof(uart1)); 

	uart1.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	uart1.c_iflag = IGNPAR | ICRNL;
	uart1.c_oflag = 0;
	uart1.c_lflag = 0;

	uart1.c_cc[VTIME] = 0;
	uart1.c_cc[VMIN]  = 1;

	//clean the line and set the attributes
	tcflush(fd_mbed,TCIFLUSH);
	tcsetattr(fd_mbed,TCSANOW,&uart1);
}
///////////////Convert the range into vehicle coordinate/////
void R_to_vehicle(float current_range, float alpha, float beta, float Pv[3])
{
	alpha=270-alpha;// looking starboard 270/////
	Pv[0]=current_range*(-cos(alpha*PI/180.0)*sin(beta*PI/180.0));
	Pv[1]=current_range*cos(alpha*PI/180.0)*cos(beta*PI/180.0);
	Pv[2]=current_range*sin(alpha*PI/180.0);
}

/////////////////Convert sonar measured range to earth 
void vehicle_to_earth(int roll, int pitch, int yaw, int x, int y, int z, float Pv[3], float Pe[3])
{
	float phi=roll*PI/18000.00; //roll rad
	float theta=pitch*PI/18000.00;//pitch rad
	float psi=yaw*PI/18000.00; // yaw rad
	float R_v_e[3][3];
	//column by column
	R_v_e[0][0]=cos(psi)*cos(theta);
	R_v_e[1][0]=sin(psi)*cos(theta);
	R_v_e[2][0]=-sin(theta);
	R_v_e[0][1]=-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
	R_v_e[1][1]=cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi);
	R_v_e[2][1]=cos(theta)*sin(phi);
	R_v_e[0][2]=sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
	R_v_e[1][2]=-cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
	R_v_e[2][2]=cos(theta)*cos(phi);
	Pe[0]=R_v_e[0][0]*Pv[0]+R_v_e[0][1]*Pv[1]+R_v_e[0][2]*Pv[2]+x/10.0;
	Pe[1]=R_v_e[1][0]*Pv[0]+R_v_e[1][1]*Pv[1]+R_v_e[1][2]*Pv[2]+y/10.0;
	Pe[2]=R_v_e[2][0]*Pv[0]+R_v_e[2][1]*Pv[1]+R_v_e[2][2]*Pv[2]+z/10.0;
//	printf("R=%f %f %f;\r\n",R_v_e[0][0],R_v_e[0][1],R_v_e[0][2]);
//	printf(" =%f %f %f;\r\n",R_v_e[1][0],R_v_e[1][1],R_v_e[1][2]);
//	printf(" =%f %f %f;\r\n",R_v_e[2][0],R_v_e[2][1],R_v_e[2][2]);
}

void earth_to_body(int roll, int pitch, int yaw, float Pv[3], float Pe[3])
{
	float phi=roll*PI/18000.00; //roll rad
	float theta=pitch*PI/18000.00;//pitch rad
	float psi=yaw*PI/18000.00; // yaw rad
	float R_v_e[3][3];
	//column by column
	R_v_e[0][0]=cos(psi)*cos(theta);
	R_v_e[0][1]=sin(psi)*cos(theta);
	R_v_e[0][2]=-sin(theta);
	R_v_e[1][0]=-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
	R_v_e[1][1]=cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi);
	R_v_e[1][2]=cos(theta)*sin(phi);
	R_v_e[2][0]=sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
	R_v_e[2][1]=-cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
	R_v_e[2][2]=cos(theta)*cos(phi);
	Pv[0]=R_v_e[0][0]*Pe[0]+R_v_e[0][1]*Pe[1]+R_v_e[0][2]*Pe[2];
	Pv[1]=R_v_e[1][0]*Pe[0]+R_v_e[1][1]*Pe[1]+R_v_e[1][2]*Pe[2];
	Pv[2]=R_v_e[2][0]*Pe[0]+R_v_e[2][1]*Pe[1]+R_v_e[2][2]*Pe[2];
}

/////Range extraction function/////
float range_extract(char raw[500], char result[500], int binbytes, int range_threshold, int range, int window, int intensity_threshold)
{
//window to be even number
///Minimum distance threshold
int min_bin;
int half_span=window/2;
int i;
int valid_count;
int j;
int max_intensity_loc=0;
int max_intensity=0;
float range_find;
min_bin=(int)(range_threshold*binbytes/range);
float range_per_bin=range*1.000/binbytes;
//printf("range per bin=%f\r\n",range_per_bin);
////assign zero to the minimum bins of the result array

for(i=0;i<min_bin;i++)
{
result[i]=0;
}

//printf("minimum range filter=%d\r\n",min_bin);
///process the data with a moving window
for(i=min_bin+half_span;i<binbytes-half_span;i++)
{
	valid_count=0;
	///temp size= window
	for(j=0;j<window;j++)
	{
		//count the number of the current window above intensity threshold
		if(raw[i-half_span+j]>intensity_threshold)
		{
		valid_count++;
		}
	}
//	printf("%d\r\n",valid_count);
	///if half of the value are larger than the intensity_threshold
	if(valid_count>half_span)
	{
	result[i]=raw[i];
	}
	else
	{
	result[i]=0;
	}
	////find the maximum
	if(result[i]>=max_intensity)
	{
	max_intensity=result[i];
	max_intensity_loc=i;
	}
}
	//printf("max_loc=%d\r\n",max_intensity_loc);
	range_find=max_intensity_loc*range_per_bin;
//	printf("range_find=%f\r\n",range_find);
return range_find;
}
//////////////////////////PID Heading controller//////////////////
void HeadingPID(float range_vy,float offset,float d_time, float &d_heading, float &error_east0, float &error_east, float &i_range, float &i_range0)
{
float Kp=3;
float Kd=0.5;
float Ki=0.01;
float d_range;
error_east=range_vy-offset;
i_range=i_range0+error_east;
d_range=error_east-error_east0;
//printf("error east=%f",error_east);
d_heading=Kp*error_east+Ki*(i_range)*d_time+Kd*d_range/d_time;

if(d_heading>35)
{
d_heading=35;
}
else if(d_heading<-35)
{
d_heading=-35;
}
printf("\r\ninfucntion d_heading=%f\r\n",d_heading);
}



/////////////////////////////////Main////////////////////////
/////////////////////////////////Main////////////////////////
/////////////////////////////////Main////////////////////////
int main()
{
int simulator_mode=0;
int glider_enable=1;

int count;
int beta=-35; //forward looking angle
///////////////////values to and from glider com/////////////
char tritech_setting[12];
int range=70;
int leftlim=225;   //-45
int rightlim=315;  //+45
/////////////////////////initial heading//////////////////
float c_heading=101;
int16_t c_heading_glider=0;
//////////////////////////////////
////////////////////////////////
float range_filtered=50; //from mbed and to glider
float current_angle=225; //from mbed and to glider
char sonar_data[600];
char sonar_bin[600];
char result_data[600];
int bytecount;
int binbytes;///size of sonar bin data

string start="initalization finished";
char endline=0x0D;
char newline=0x0A;

unsigned char glider_feed[10];
char glider_header;
//define glider return data sets//
signed char a_pitch[2]; signed char a_roll[2]; char a_heading[2];
int m_heading; int16_t m_pitch; int16_t m_roll;
int32_t a_x[4]; int32_t a_y[4];char a_depth[2];
int32_t m_x; int32_t m_y; int16_t m_depth;
float m_xf,m_yf,m_depthf,m_headingf,m_pitchf,m_rollf;
////prior define for glider com.
glider_feed[0]=0;
glider_feed[1]=4;
glider_feed[8]=13;

//////////////Values for mbed com////////////////
char mbed_data[15];
char mbed_header;
//////////Values for heading controller////////
float Pv[3];
float Pe[3];
float range_result;
float temp_ix[50];
float temp_iy[50];
float temp_iz[50];
int ff=0;
int invalid=0;
int goin=0;
int backup=0;
int valid=0;
float ambiguity_flag=0;
int yaw0=0;
int range_filtered0=0;
float d_yaw;
float d_range;
float distanceP=1000;
float distancePc;
int distancePindex=0;
float vehicle_to_point[3];
float vehicle_to_point_body[3];
float d_time;
struct timeval t_now;
long t0,tc;
int count1;
float offset=35.0;
float d_heading=0, error_east=0, error_east0=0, i_range=0, i_range0=0,d_heading_to;
int temp_full=0;
char send_to_mbed=0x55;
/////////////////////heading controller constrains/////////////////
char active_depth;
char active_sectorL[2];
char active_sectorR[2];
int16_t active_L;
int16_t active_R;
float sum_vx=0,sum_vy=0,sum_vz=0;
int down_flag=0,up_flag=0,center_flag=0;
float range_vy0=70;
/////////log file and time stuff/////////////////
FILE *logfd;
///////////////////////////////////////////////////////////
////////////////////end of defining variables//////////////
///////////////////////////////////////////////////////////

///////////////////testing area/////////////////////////////

//start the serials///
serial_init_glider();
serial_init_mbed();

///inital log files///
//logfd=fopen("/media/UNTITLED/tritech_log.txt","a");
logfd=fopen("tritech_log.txt","a");
fprintf(logfd,"good now\r\n");
fclose(logfd);
//logfd=fopen("/media/UNTITLED/tritech_log.txt","a");
logfd=fopen("tritech_log.txt","a");

///Get heading controller setup////
///get minimum depth
/*
	while(!(read(fd_mbed,&active_depth,1)>0))
	{
	pauseNanoSec(2000000);
	}
///get sector
	for (count=0;count<2;count++)
	{
	while(!(read(fd_mbed,&active_sectorL[count],1)>0))
		{
		pauseNanoSec(2000000);
		}
	}
	for (count=0;count<2;count++)
	{
	while(!(read(fd_mbed,&active_sectorR[count],1)>0))
		{
		pauseNanoSec(2000000);
		}
	}
	active_L=active_sectorL[0]*256+active_sectorL[1];
	active_R=active_sectorL[0]*256+active_sectorL[1];
*/
////get sonar setup/////
///receive the next 6 bytes which are the sonar setting
if(glider_enable==1)
{
//printf("waiting for glider setup\r\n");
	while(!(glider_header==0x56))
	{
	while(!(read(fd_glider,&glider_header,1)>0))
	{
	pauseNanoSec(2000000);
	}
	}
	for(count=0;count<12;count++)
	{
		while(!(read(fd_glider,&tritech_setting[count],1)>0))
		{
		pauseNanoSec(2000000);
		}
	//	printf("%d;",tritech_setting[count]);
	}

	range=tritech_setting[0];
	leftlim=tritech_setting[1]+tritech_setting[2]*256;
	rightlim=tritech_setting[3]+tritech_setting[4]*256;
	c_heading_glider=tritech_setting[5]+tritech_setting[6]*256;
	c_heading=c_heading_glider;
	active_depth=tritech_setting[7];
	active_L=tritech_setting[8]+tritech_setting[9]*256;
	active_R=tritech_setting[10]+tritech_setting[11]*256;
	//printf("range=%d;leftlim=%d;rightlim=%d;c_heading=%f\r\n",range,leftlim,rightlim,c_heading);

	////send a string to let glider jump into proglet run() function.
	write(fd_glider,&start[0],22);
	write(fd_glider,&endline,1);
	write(fd_glider,&newline,1);
	///send to mbed to setup the sonar
	write(fd_mbed,&tritech_setting[0],5);
}//end of glider_enable==1

///////////////////////////////send sonar setup to mbed///////////////

//printf("I'm alive\r\n");

fprintf(logfd,"Start: range=%d; leflim=%d; rightlim=%d;c_heading=%f\r\n",range,leftlim,rightlim,c_heading);
fprintf(logfd,"m_roll;m_pitchf;m_headingf;north;east;m_depthf;range_filtered;current_angle;c_heading\r\n");
fclose(logfd);
//////////////////////Start the looping//////////////////////
while(1)
{
	//logfd=fopen("/media/UNTITLED/tritech_log.txt","a");
	logfd=fopen("tritech_log.txt","a");
////////////////////////////////////////////////////////
///////////get vehicle information from the glider/////
///////////////////////////////////////////////////////
	while(!(glider_header==0x55))
	{
	while(!(read(fd_glider,&glider_header,1)>0))
	{
	pauseNanoSec(2000000);
	}
//	printf("%X\r\n",glider_header);
	}
	glider_header=0; //zero glider_header

	///get the following chars////////////
	//get depth
	for(count=0;count<2;count++)
	{
	while(!(read(fd_glider,&a_depth[count],1)>0))
	{
	pauseNanoSec(2000000);
	}
	}
	//get pitch
	for(count=0;count<2;count++)
	{
	while(!(read(fd_glider,&a_pitch[count],1)>0))
	{
	pauseNanoSec(2000000);
	}
	}
	//get roll
	for(count=0;count<2;count++)
	{
	while(!(read(fd_glider,&a_roll[count],1)>0))
	{
	pauseNanoSec(2000000);
	}
	}
	//get heading
	for(count=0;count<2;count++)
	{
	while(!(read(fd_glider,&a_heading[count],1)>0))
	{
	pauseNanoSec(2000000);
	}
	}
	//get x
	for(count=0;count<4;count++)
	{
	while(!(read(fd_glider,&a_x[count],1)>0))
	{
	pauseNanoSec(2000000);
	}
	}
	//get y
	for(count=0;count<4;count++)
	{
	while(!(read(fd_glider,&a_y[count],1)>0))
	{
	pauseNanoSec(2000000);
	}
	}

	///Convert the numbers////
	m_depth=(a_depth[0]*256)+a_depth[1];  //in center-meters
	m_pitch=(a_pitch[0]*256)+a_pitch[1];  //in centi-degree
	m_roll=(a_roll[0]*256)+a_roll[1];     //in centi-degree
	m_heading=(a_heading[0]*256)+a_heading[1];  //in centi-degree
///switch m_y -> east    m_x ->north	
	m_y=((a_x[0]<<24)&0xFF000000)|((a_x[1]<<16)&0x00FF0000)|((a_x[2]<<8)&0x0000FF00)|(a_x[3]&0x000000FF);  //in cm
	m_x=((a_y[0]<<24)&0xFF000000)|((a_y[1]<<16)&0x00FF0000)|((a_y[2]<<8)&0x0000FF00)|(a_y[3]&0x000000FF); //in cm

	m_xf=m_x/10.0;
	m_yf=m_y/10.0;
	m_depthf=m_depth/10.0;
	m_headingf=m_heading/100.00;
	m_rollf=m_roll/100.00;
	m_pitchf=m_pitch/100.00;

/////////////////////////////////////////////////
/////////////////get sonar sample///////////////
////////////////////////////////////////////////
	if(simulator_mode==0)
	{
		sonar_data[0]=0;
		write(fd_mbed,&sonar_data[0],1);
		//until saw the header
		while(!(sonar_data[0]==0x40))
		{
		while(!(read(fd_mbed,&sonar_data[0],1)>0))
		{
		pauseNanoSec(2000000);
		}
		}//end read the header
		sonar_data[0]=0;
		//printf("got header\r\n");
		///receive next six byte to calcualte the output length
		for(count=1;count<7;count++)
		{
		while(!(read(fd_mbed,&sonar_data[count],1)>0))
		{
		pauseNanoSec(2000000);
		}
		}
		bytecount=sonar_data[5]+sonar_data[6]*256+6;
		//printf("%d\r\n",bytecount);
		////read the rest///
		for(count=7;count<bytecount;count++)
		{
		while(!(read(fd_mbed,&sonar_data[count],1)>0))
		{
		pauseNanoSec(2000000);
		}
		//printf("%X;",sonar_data[count]);
		}//end read the rest
		//get the current_angle
		current_angle=(sonar_data[40]+sonar_data[41]*256)/400.00*360/16.00;

		binbytes=sonar_data[42]+sonar_data[43]*256;
		//printf("%d",binbytes);

		/////log the sonar_data
		for(count=44;count<bytecount;count++)
		{
		fprintf(logfd,"%d;",sonar_data[count]);
		sonar_bin[count-44]=sonar_data[count];
		}
		fprintf(logfd,"\r\n");
//get range		
	range_filtered=range_extract(sonar_bin, result_data,binbytes-1, 10, range, 20, 0);///minimum range to be 10 meters
	//printf("current_angle=%f;range_filtered=%f\r\n",current_angle,range_filtered);

	}

	//printf("range_filtered=%d;%d;%d;%d;current_angle=%d;%d;%d;%d;\r\n",mbed_data[0],mbed_data[1],mbed_data[2],mbed_data[3],mbed_data[5],mbed_data[6],mbed_data[7],mbed_data[8]);
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////Calculate the c_heading?/////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//if(m_depthf>20)
//{
	R_to_vehicle(range_filtered, current_angle, beta,Pv);
	//convert to earth
	vehicle_to_earth(m_roll,m_pitch,m_heading,m_x,m_y,m_depth,Pv,Pe);

	//update valid measurement update the temp array
	if(m_depthf>active_depth)//&current_angle<280&current_angle>240)
	{
		if(current_angle<active_R&current_angle>active_L)
		{
			center_flag++;
			///when the sonar entering the usefull sector-calculate the ambiguity flag
			if(down_flag==1|up_flag==1)
			{
			ff=0;
			d_yaw=m_headingf-yaw0;
			yaw0=m_headingf;
			d_range=Pv[1]-range_vy0;
			range_vy0=Pv[1];
			//d_range=range_filtered-range_filtered0;
			//range_filtered0=range_filtered;
			ambiguity_flag=d_range*d_yaw;
			}
			down_flag=0;
			up_flag=0;
			//printf("%d\r\n",temp_length);
						
			temp_ix[ff]=Pv[0];
			temp_iy[ff]=Pv[1];
			temp_iz[ff]=Pv[2];
			ff++;		
		//	printf("temp_iy=%f\r\n",temp_iy[ff]);		
			//if(ff>temp_length)
			//{
			//ff=0;
			//temp_full=1;
			//}
		//	printf("ff=%d",ff);
			
		}
		
		if(current_angle>active_R)
		{
		up_flag=1;
		}
		if(current_angle<active_L)
		{
		down_flag=1;
		}
		///calculate the d_heading
		if((down_flag==1|up_flag==1)&center_flag>0)
		{

			sum_vy=0;
			for(count=0;count<ff;count++)
			{
			sum_vx=sum_vx+temp_ix[count];
			sum_vy=sum_vy+temp_iy[count];
			sum_vz=sum_vz+temp_iz[count];
			//printf("%f;",temp_iy[count]);
			}
			//int c = getchar();
			sum_vy=sum_vy/(ff-1);
		//	printf("sum_vy=%f\r\n",sum_vy);
			if(ambiguity_flag>0)
			{
			d_heading=-10;
			}			
			else
			{
			
			HeadingPID(sum_vy,offset,0.2,d_heading, error_east,error_east0,i_range,i_range0);
			error_east0=error_east;
			i_range0=i_range;
			}
			
		}
	c_heading=m_headingf+d_heading;
	}


	if(c_heading<0)
	{
	c_heading=360+c_heading;
	}
	if(c_heading>360)
	{
	c_heading=c_heading-360;
	}
//////////////////////////////////////////////////////////////////////////////////////////
	////////////////////send to glider//////////////////////////
////////////////////////////////////////////////////////////////////////////
	glider_feed[2]=(int) range_filtered+15;
	glider_feed[3]=(range_filtered-((int)range_filtered))*100+15;
	glider_feed[4]=((int)(current_angle/10.00))+15;
	glider_feed[5]= (current_angle/10.00-((int)(current_angle))/10)*100+15;
	glider_feed[6]= ((int)(c_heading/10.00))+15;
	glider_feed[7]= (c_heading/10.00-((int)(c_heading))/10)*100+15;
	for (count=0;count<9;count++)
	{
	write(fd_glider,&glider_feed[count],1);
	}
	//printf("sent to glider\r\n");
fprintf(logfd,"%.2f;%.2f;%.2f;%.1f;%.1f;%.1f;%.2f;%.1f;%.2f;\r\n",m_rollf,m_pitchf,m_headingf,m_xf,m_yf,m_depthf,range_filtered,current_angle,c_heading);

fclose(logfd);
}

}


