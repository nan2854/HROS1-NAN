 /*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    ((char *)"../../../Data/motion_1024.bin")
#else
#define MOTION_FILE_PATH    ((char *)"../../../Data/motion_4096.bin")
#endif
#define INI_FILE_PATH       ((char *)"../../../Data/config.ini")

#define M_INI	((char *)"../../../Data/slow-walk.ini")
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

int isRunning = 1;
// Define the exit signal handler
void signal_callback_handler(int signum)
{
    //LinuxCamera::~LinuxCamera();
    printf("Exiting program; Caught signal %d\r\n",signum);
    isRunning = 0;
}

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);
int GetCurrentPosition(CM730 &cm730);
////////////////////////////////////////////
Action::PAGE Page;
Action::STEP Step;
////////////////////////////////////////////
int change_current_dir()
{
    char exepath[1024] = {0};
    int status = 0;
		if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        status = chdir(dirname(exepath));
		return status;
}

int main(int argc, char *argv[])
{
	//int trackerSel;    
	change_current_dir();

	minIni* ini = new minIni(INI_FILE_PATH); 
	minIni* ini1 = new minIni(M_INI); 
	StatusCheck::m_ini = ini;
	StatusCheck::m_ini1 = ini1;

/* RL - Soccer Demo! */

	//Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
	Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    //ColorFinder* ball_finder = new ColorFinder(0, 15, 45, 0, 0.3, 50.0);
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();

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
    //MotionManager::GetInstance()->LoadINISettings(ini);

    int firm_ver = 0,retry=0;
    //important but allow a few retries
	while(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        retry++;
		if(retry >=3) exit(1);// if we can't do it after 3 attempts its not going to work.
    }

    if(0 < firm_ver && firm_ver < 27)
    {
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
    }
    else
        exit(0);

	Walking::GetInstance()->LoadINISettings(ini);   
	MotionManager::GetInstance()->LoadINISettings(ini); 

    Walking::GetInstance()->m_Joint.SetEnableBody(false);
    Head::GetInstance()->m_Joint.SetEnableBody(false);
    Action::GetInstance()->m_Joint.SetEnableBody(true);
    MotionManager::GetInstance()->SetEnable(true);
              
    cm730.WriteByte(CM730::P_LED_PANNEL, 0x02, NULL);

    if(PS3Controller_Start() == 0)
			printf("PS3 controller not installed.\n");
	cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(1,1,1),0);
	
	//determine current position
	StatusCheck::m_cur_mode = GetCurrentPosition(cm730);
	
	//LinuxActionScript::PlayMP3("../../../Data/mp3/ready.mp3");
	
	if((argc>1 && strcmp(argv[1],"-off")==0) || (StatusCheck::m_cur_mode == SITTING))
	{
		cm730.DXLPowerOn(false);
		//for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
		//	cm730.WriteByte(id, MX28::P_TORQUE_ENABLE, 0, 0);
	}
	else
	{
		Action::GetInstance()->Start(15);
		while(Action::GetInstance()->IsRunning()) usleep(8*1000);
	}
			
	if ( cm730.WriteWord(CM730::ID_BROADCAST, MX28::P_MOVING_SPEED_L, 1023, 0) != CM730::SUCCESS )
	{
		printf( "Warning: TODO\r\n");
	}
	usleep( 100000 );
	cm730.WriteByte(85, 0, 0, 0); //red off
	cm730.WriteByte(85, 1, 0, 0); //green off
	cm730.WriteByte(85, 2, 0, 0); //blue off

    while(isRunning)
	{  
		LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

		//usleep( 10000 );

		StatusCheck::Check(cm730);

		if(StatusCheck::m_cur_mode == SOCCER)
        {
            tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

            for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
            {
                if(ball_finder->m_result->m_ImageData[i] == 1)
                {
                    rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                    rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 0;
                    rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                }
            }
        }

        streamer->send_image(rgb_output);

		if(StatusCheck::m_is_started == 0) continue;

		switch(StatusCheck::m_cur_mode)
        {
        case READY:
            break;
        case SOCCER:
            if(Action::GetInstance()->IsRunning() == 0)
            {
                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                follower.Process(tracker.ball_position);

                if(follower.KickBall != 0)
                {
                    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                    Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                    if(follower.KickBall == -1)
                    {
                    	cm730.WriteByte(85, 0, 0, 0); //red off
                    	cm730.WriteByte(85, 1, 0, 0); //green off
                    	cm730.WriteByte(85, 2, 200, 0); //blue on
                        Action::GetInstance()->Start(12);   // RIGHT KICK
                        //fprintf(stderr, "RightKick! \n");
                        printf( "Right Kick!\r\n" );
                    }
                    else if(follower.KickBall == 1)
                    {
                    	cm730.WriteByte(85, 0, 0, 0); //red off
                    	cm730.WriteByte(85, 2, 0, 0); //blue off
                    	cm730.WriteByte(85, 1, 200, 0); //green on
                        Action::GetInstance()->Start(13);   // LEFT KICK
                        //fprintf(stderr, "LeftKick! \n");
                        printf( "Left Kick!\r\n" );
                    }
                    else
                    {
                    	cm730.WriteByte(85, 2, 0, 0); //blue off
                    	cm730.WriteByte(85, 1, 0, 0); //green off
                    }
                }
            }
            break;
        case MOTION:
            //if(LinuxActionScript::m_is_running == 0)
            //    LinuxActionScript::ScriptStart(SCRIPT_FILE_PATH);
            break;
        case VISION:
        	/*
            int detected_color = 0;
            detected_color |= (red_pos.X == -1)? 0 : VisionMode::RED;
            detected_color |= (yellow_pos.X == -1)? 0 : VisionMode::YELLOW;
            detected_color |= (blue_pos.X == -1)? 0 : VisionMode::BLUE;

            if(Action::GetInstance()->IsRunning() == 0)
                VisionMode::Play(detected_color);
               */
            break;
        }
	}

    return 0;
}

int GetCurrentPosition(CM730 &cm730)
{
	int m=Robot::READY,p,j,pos[31];
	int dMaxAngle1,dMaxAngle2,dMaxAngle3;
	double dAngle;
	int rl[6] = { JointData::ID_R_ANKLE_ROLL,JointData::ID_R_ANKLE_PITCH,JointData::ID_R_KNEE,JointData::ID_R_HIP_PITCH,JointData::ID_R_HIP_ROLL,JointData::ID_R_HIP_YAW };
	int ll[6] = { JointData::ID_L_ANKLE_ROLL,JointData::ID_L_ANKLE_PITCH,JointData::ID_L_KNEE,JointData::ID_L_HIP_PITCH,JointData::ID_L_HIP_ROLL,JointData::ID_L_HIP_YAW };

	for(p=0;p<31;p++) 
		{
		pos[p]	= -1;
		}
	for(p=0; p<6; p++)
		{
		if(cm730.ReadWord(rl[p], MX28::P_PRESENT_POSITION_L, &pos[rl[p]], 0) != CM730::SUCCESS)
			{
			printf("Failed to read position %d",rl[p]);
			}
		if(cm730.ReadWord(ll[p], MX28::P_PRESENT_POSITION_L, &pos[ll[p]], 0) != CM730::SUCCESS)
			{
			printf("Failed to read position %d",ll[p]);
			}
		}
	// compare to a couple poses
	// first sitting - page 48
	Action::GetInstance()->LoadPage(48, &Page);
	j = Page.header.stepnum-1;
	dMaxAngle1 = dMaxAngle2 = dMaxAngle3 = 0;
	for(p=0;p<6;p++)
		{
		dAngle = abs(MX28::Value2Angle(pos[rl[p]]) - MX28::Value2Angle(Page.step[j].position[rl[p]]));
		if(dAngle > dMaxAngle1)
			dMaxAngle1 = dAngle;
		dAngle = abs(MX28::Value2Angle(pos[ll[p]]) - MX28::Value2Angle(Page.step[j].position[ll[p]]));
		if(dAngle > dMaxAngle1)
			dMaxAngle1 = dAngle;
		}				
	// squating - page 15
	Action::GetInstance()->LoadPage(15, &Page);
	j = Page.header.stepnum-1;
	for(int p=0;p<6;p++)
		{
		dAngle = abs(MX28::Value2Angle(pos[rl[p]]) - MX28::Value2Angle(Page.step[j].position[rl[p]]));
		if(dAngle > dMaxAngle2)
			dMaxAngle2 = dAngle;
		dAngle = abs(MX28::Value2Angle(pos[ll[p]]) - MX28::Value2Angle(Page.step[j].position[ll[p]]));
		if(dAngle > dMaxAngle2)
			dMaxAngle2 = dAngle;
		}				
	// walkready - page 9
	Action::GetInstance()->LoadPage(9, &Page);
	j = Page.header.stepnum-1;
	for(int p=0;p<6;p++)
		{
		dAngle = abs(MX28::Value2Angle(pos[rl[p]]) - MX28::Value2Angle(Page.step[j].position[rl[p]]));
		if(dAngle > dMaxAngle3)
			dMaxAngle3 = dAngle;
		dAngle = abs(MX28::Value2Angle(pos[ll[p]]) - MX28::Value2Angle(Page.step[j].position[ll[p]]));
		if(dAngle > dMaxAngle3)
			dMaxAngle3 = dAngle;
		}				
	if(dMaxAngle1<20 && dMaxAngle1<dMaxAngle2 && dMaxAngle1<dMaxAngle3)
		m = Robot::SITTING;
	if(dMaxAngle2<20 && dMaxAngle2<dMaxAngle1 && dMaxAngle2<dMaxAngle3)
		m = Robot::READY;
	if(dMaxAngle3<20 && dMaxAngle3<dMaxAngle1 && dMaxAngle3<dMaxAngle2)
		m = Robot::SOCCER;
	printf("Sitting = %d, Squating = %d, Standing = %d\n",dMaxAngle1,dMaxAngle2,dMaxAngle3);
	printf("Robot is %s\n",m==Robot::READY?"Ready":m==Robot::SOCCER?"Soccer":m==Robot::SITTING?"Sitting":"None");
	return m;
}
