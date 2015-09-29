/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    ((char *)"../../../Data/motion_1024.bin")
#else
#define MOTION_FILE_PATH    ((char *)"../../../Data/motion_4096.bin")
#endif
#define INI_FILE_PATH       ((char *)"../../../Data/config.ini")

#define M_INI   ((char *)"../../../Data/slow-walk.ini")
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

int isRunning = 1;
LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

// Define the exit signal handler
void signal_callback_handler(int signum)
{
    //LinuxCamera::~LinuxCamera();
    printf("Exiting program; Caught signal %d\r\n",signum);
    isRunning = 0;
}

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    //Register signal and signal handler
    signal(SIGINT, signal_callback_handler);

    printf( "\n===== Head tracking Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

//////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);
    usleep(100);
    MotionManager::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    //MotionManager::GetInstance()->StartThread();
    //LinuxMotionTimer::Initialize(MotionManager::GetInstance());
    LinuxMotionTimer linuxMotionTimer;
        linuxMotionTimer.Initialize(MotionManager::GetInstance());
        linuxMotionTimer.Start();

	MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
	MotionManager::GetInstance()->SetEnable(true);
	/////////////////////////////////////////////////////////////////////

	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);


    while(isRunning)
    {
        //usleep(10000);
        Point2D pos;
        LinuxCamera::GetInstance()->CaptureFrame();	

				rgb_ball = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
				float red_avg = 0;
				float green_avg = 0;
			  float blue_avg = 0;
				int bright_cnt = 1;
				int threshold = 128;
				
				int bounds_height = 48;
				int bounds_width = 64;
				
				static int bounds_y_min = (Camera::HEIGHT / 2) - (bounds_height / 2 );
				static int bounds_y_max = (Camera::HEIGHT / 2) + ( bounds_height / 2 );
				static int bounds_x_min = (Camera::WIDTH / 2)-(bounds_width / 2);
				static int bounds_x_max = (Camera::WIDTH / 2)+(bounds_width / 2);
        for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
        {
        	  int y_pos = i / 320;
        	  int x_pos = i % 320;
        	  
        	  if ( y_pos > bounds_y_min && y_pos < bounds_y_max && x_pos > bounds_x_min && x_pos < bounds_x_max )
            /*
            if ( rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] > threshold
            	|| rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] > threshold
            	|| rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] > threshold )
            */
            {
            	red_avg += rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0];
            	green_avg += rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1];
            	blue_avg += rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2];
            	++bright_cnt;
            	
            	rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
            	rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255;
            	rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 255;
            	
            }            
        }
        
        red_avg /= bright_cnt;
        green_avg /= bright_cnt;
        blue_avg /= bright_cnt;
        
        cm730.WriteByte(85, 0, (int)red_avg, 0); //red
        //usleep(10000);
        cm730.WriteByte(85, 1, (int)green_avg, 0); //green
        //usleep(10000);
        cm730.WriteByte(85, 2, (int)blue_avg, 0); //blue
        
        
        //printf("%4.0f %4.0f %4.0f\n",red_avg,green_avg,blue_avg);
        
        streamer->send_image(rgb_ball);
    }

    return 0;
}
