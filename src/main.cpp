
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <termio.h>


#include <opencv2/highgui/highgui.hpp>

#include "CqUsbCam.h"
#include "SensorCapbablity.h"


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <boost/assign/list_of.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/sync_queue.hpp>

#define MAIN_CAPTURE			'l'

#define MAIN_EXIT_NORMAL		'z'

string sensor = "MT9V034";
unsigned int g_width=752;
unsigned int g_height=480;

pthread_mutex_t mutexDisp;
pthread_mutex_t mutexCam;

cv::Mat curFrame;
void Disp(void* frameData)
{
	pthread_mutex_lock(&mutexDisp);
    curFrame = cv::Mat(g_height, g_width, CV_8UC1, (unsigned char*)frameData);
	pthread_mutex_unlock(&mutexDisp);

}



CCqUsbCam cam0,  *pCamInUse;


unsigned short hex2dec(char *hex)

{

	unsigned short  number=0;

	char *p=hex;

	for(p=hex;*p;++p)
	{
		if((hex[p-hex]<='z')&&(hex[p-hex]>='a'))
			hex[p-hex]=hex[p-hex]-32;
		number=number*16+(hex[p-hex]>='A'?hex[p-hex]-'A'+10:hex[p-hex]-'0');
	}

	return number;

}

void checkspeed()
{
	unsigned int speed = 0;
	cam0.GetUsbSpeed(speed);
	if(speed==LIBUSB_SPEED_SUPER)
	{
		printf("USB 3.0 device found on cam0!\n");
		cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_SUPER);
	}
	else if(speed==LIBUSB_SPEED_HIGH)
	{
		printf("USB 2.0 device found on cam0!\n");
		cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_HIGH);
	}
	else
	{
		printf("Device speed unknown on cam0!\n");
	}

}

void timerFunction(int sig)
{

	unsigned long iByteCntPerSec=0;
	unsigned long iFrameCntPerSec=0;

	pthread_mutex_lock(&mutexCam);

	cam0.GetRecvByteCnt(iByteCntPerSec);
	cam0.ClearRecvByteCnt();
	cam0.GetRecvFrameCnt(iFrameCntPerSec);
	cam0.ClearRecvFrameCnt();

	printf("cam0: %ld Fps     %0.4f MBs\n", iFrameCntPerSec, float(iByteCntPerSec)/1024.0/1024.0);


	alarm(1);

	pthread_mutex_unlock(&mutexCam);

}



int main(int argc, char *argv[])
{
	cq_int32_t ret;
	ret =pthread_mutex_init(&mutexDisp, NULL);
	if(ret!=0)
		printf("pthread_mutex_init failed");
	ret =pthread_mutex_init(&mutexCam, NULL);
	if(ret!=0)
		printf("pthread_mutex_init failed");

	cam0.SelectSensor(sensor);

	int usbCnt=CCqUsbCam::OpenUSB();
	printf("%d usb device(s) found!\n", usbCnt);
	if(usbCnt<=0)
	{
		printf("exiting ...\n");
		return -1;
	}
	cam0.ClaimInterface(0);
	//cam0.InitSensor(); //not required by 	mt9v034

	checkspeed();

	pCamInUse=&cam0;


    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~"); // to get the private params
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("camera", 1);

    double fps = 30;
    ros::Rate r(fps);
    while(1)
    {
        printf("Please input your choice ...\n");
        printf("\
				\'a\':	Select resolution\n\
				\'c\':	Select proc type\n\
				\'d\':	Check speed\n\
				\'e\':	Set auto trig mode\n\
				\'f\':	Set fpga trig mode\n\
				\'g\':	Set fpga trig frequency\n\
				\'h\':	Set expo value\n\
				\'i\':	Set gain value\n\
				\'j\':	Set auto-gain-expo function\n\
				\n\
				\'l\':	Start capture\n\
				\n\
				\'F\':	Write sensor\n\
				\'G\':	Read sensor\n\
				\'H\':	Write FPGA\n\
				\'I\':	Read FPGA\n\
				\n\
				\'z\':	Exit\n"\
		      );
        char ch=getchar();
        getchar();
        printf("Your choice is %c\n", ch);
        switch(ch)
        {

            case MAIN_CAPTURE: {


                cam0.StartCap(g_height, g_width, Disp);
//
                signal(SIGALRM, timerFunction);
                alarm(1);


                pthread_mutex_lock(&mutexCam);
                alarm(0);
                cam0.StopCap();

                pthread_mutex_unlock(&mutexCam);

                pthread_mutex_lock(&mutexDisp);
                cv::imshow("image", curFrame);
                cv::waitKey(30);

                pthread_mutex_unlock(&mutexDisp);


                break;
            }

            case MAIN_EXIT_NORMAL:
                cam0.ReleaseInterface();
                CCqUsbCam::CloseUSB();

                printf("Exiting ...\n");
                exit(0);
                break;
            default:
                printf("Bad inut ...\n");
        }


    }



    return 0;

}