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

#define INI_FILE_PATH       ((char *)"../../../Data/config.ini")
#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

int isRunning = 1;
LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

// Define the exit signal handler
void signal_callback_handler(int signum)
{
    printf("Exiting program; Caught signal %d\r\n",signum);
    
    cm730.DXLPowerOn(false); //NOTE: Turning off Dynamixel bus because it's convenient for me (quite cooling fans)
    
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

    printf( "\n===== RGB Color Sensing for HR-OS1 =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_frame = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    RGBHands hand_ctrl( &cm730 );

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

    MotionManager::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->SetEnable(false);
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    LinuxMotionTimer linuxMotionTimer;
    linuxMotionTimer.Initialize(MotionManager::GetInstance());
    linuxMotionTimer.Stop();

    //Specify color sensing area for sampling
    int bounds_height = 48; //20%
    int bounds_width = 64; //20%

    int bounds_y_min = ( Camera::HEIGHT / 2 ) - ( bounds_height / 2 );
    int bounds_y_max = ( Camera::HEIGHT / 2 ) + ( bounds_height / 2 );
    int bounds_x_min = ( Camera::WIDTH / 2 ) - ( bounds_width / 2 );
    int bounds_x_max = ( Camera::WIDTH / 2 ) + ( bounds_width / 2 );

    while(isRunning)
    {
        LinuxCamera::GetInstance()->CaptureFrame();	
        rgb_frame = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
        
        int red_avg = 0;
        int green_avg = 0;
        int blue_avg = 0;
        int sample_cnt = 1;
        int threshold = 42;

        for(int i = 0; i < rgb_frame->m_NumberOfPixels; i++)
        {
            int y_pos = i / 320;
            int x_pos = i % 320;

            //Determine if current pixel is within our color sensing bounds
            if ( y_pos > bounds_y_min && y_pos < bounds_y_max && x_pos > bounds_x_min && x_pos < bounds_x_max )
            
            //Optional threshold value can be used
            if ( rgb_frame->m_ImageData[i*rgb_frame->m_PixelSize + 0] > threshold
            	|| rgb_frame->m_ImageData[i*rgb_frame->m_PixelSize + 1] > threshold
            	|| rgb_frame->m_ImageData[i*rgb_frame->m_PixelSize + 2] > threshold )
            
            {
            	red_avg += rgb_frame->m_ImageData[i*rgb_frame->m_PixelSize + 0];
            	green_avg += rgb_frame->m_ImageData[i*rgb_frame->m_PixelSize + 1];
            	blue_avg += rgb_frame->m_ImageData[i*rgb_frame->m_PixelSize + 2];
            	++sample_cnt;
            	
            	rgb_frame->m_ImageData[i*rgb_frame->m_PixelSize + 0] = 255;
            	rgb_frame->m_ImageData[i*rgb_frame->m_PixelSize + 1] = 255;
            	rgb_frame->m_ImageData[i*rgb_frame->m_PixelSize + 2] = 255;
            	
            }            
        }
        
        red_avg /= sample_cnt;
        green_avg /= sample_cnt;
        blue_avg /= sample_cnt;
        
        hand_ctrl.SetRGB( red_avg, green_avg, blue_avg );

        printf("Sensing R%d G%d B%d\n",red_avg,green_avg,blue_avg);
        
        streamer->send_image(rgb_frame);
    }

    return 0;
}
