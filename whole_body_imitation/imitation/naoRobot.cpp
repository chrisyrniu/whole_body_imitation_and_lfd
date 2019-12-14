#include "naoRobot.h"
#include <iostream>
#include <fstream>
#include <alcommon/albroker.h>
#include <stdio.h>  
#include <windows.h>  
#include <time.h> //time_t time()  clock_t clock()
#include <Mmsystem.h>             //timeGetTime()  
#pragma comment(lib, "Winmm.lib")   //timeGetTime()  


naoRobot::naoRobot(boost::shared_ptr<AL::ALBroker> broker,const std::string& name): AL::ALModule(broker, name)
{
	// Describe the module here. This will appear on the web page
	setModuleDescription("My own custom module.");

	functionName("startImitation", getName(), "The major process of imitation.");
	BIND_METHOD(naoRobot::startImitation);

	functionName("setJointAngles", getName(), "Set the angles of Nao joints");
	BIND_METHOD(naoRobot::setJointAngles);

	functionName("getBodyAngles", getName(), "Calculate joint angles.");
	BIND_METHOD(naoRobot::getBodyAngles);

	functionName("getVector", getName(), "Get vectors that consist of joint points.");
	BIND_METHOD(naoRobot::getVector);

	functionName("getVectorDiff", getName(), "Get the position difference of two points.");
	BIND_METHOD(naoRobot::getVectorDiff);

	functionName("getCrossVector", getName(), "Get the cross vector.");
	BIND_METHOD(naoRobot::getCrossVector);

	functionName("getCos", getName(), "Get cosine value of included angle between two vectors.");
	BIND_METHOD(naoRobot::getCos);

	isLeftHandOpen = false;
	isRightHandOpen = false;

}
naoRobot::~naoRobot()
{
}
void naoRobot::startImitation(_kinectData kinectData)
{
	
	getVector(kinectData);
	getBodyAngles(kinectData);
	setJointAngles();
	ctrHandState(kinectData);
	evaluation();
}
void naoRobot::setJointAngles()
{

	float fractionMaxSpeed = 1.0f;
	//clock time
	s=s+1;
	if(s==1)
	{
		clockInitial=clock();
		robotProxy.goToPosture("StandInit", 0.3);
		clockRandom=clock();
		time_random=clockRandom-clockInitial;
		outfile.open("exp1Data2.txt");
	}
	//AngleNames
	const AL::ALValue ArmNames=AL::ALValue::array("LShoulderRoll","LShoulderPitch","LElbowYaw","LElbowRoll","RShoulderRoll","RShoulderPitch","RElbowYaw","RElbowRoll");
	const AL::ALValue LegNames=AL::ALValue::array("LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll","RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll");
	const AL::ALValue LegWithoutAnkleRollNames=AL::ALValue::array("LHipRoll","LHipPitch","LKneePitch","LAnklePitch","RHipRoll","RHipPitch","RKneePitch","RAnklePitch");
	const AL::ALValue LegWithoutAnklePitchAndRollNames=AL::ALValue::array("LHipRoll","LHipPitch","LKneePitch","RHipRoll","RHipPitch","RKneePitch");
	const AL::ALValue LLegLowerNames=AL::ALValue::array("RAnklePitch","RKneePitch","RHipPitch");
	const AL::ALValue RLegLowerNames=AL::ALValue::array("LAnklePitch","LKneePitch","LHipPitch");
	const AL::ALValue HeadNames=AL::ALValue::array("HeadPitch","HeadYaw");
	const AL::ALValue LAnkleNames=AL::ALValue::array("LAnklePitch","LAnkleRoll");
	const AL::ALValue RAnkleNames=AL::ALValue::array("RAnklePitch","RAnkleRoll");
	//AngleLists
	AL::ALValue ArmAngleLists;
	AL::ALValue LCOMAngleLists;
	AL::ALValue RCOMAngleLists;
	AL::ALValue LLegLiftAngleLists;
	AL::ALValue LLegLiftAngleLists1;
	AL::ALValue LLegLiftAngleLists2;
	AL::ALValue RLegLiftAngleLists;
	AL::ALValue RLegLiftAngleLists1;
	AL::ALValue RLegLiftAngleLists2;
	AL::ALValue LLegLowerAngleLists;
	AL::ALValue RLegLowerAngleLists;
	AL::ALValue DoubleLegsAngleLists;
	AL::ALValue DoubleLegsAngleLists1;
	AL::ALValue DoubleLegsAngleLists2;
	AL::ALValue HeadAngleLists;
	//AngleListsSize
	ArmAngleLists.arraySetSize(8);
	LCOMAngleLists.arraySetSize(10);
	RCOMAngleLists.arraySetSize(10);
	LLegLiftAngleLists.arraySetSize(10);
	LLegLiftAngleLists1.arraySetSize(8);
	LLegLiftAngleLists2.arraySetSize(6);
	RLegLiftAngleLists.arraySetSize(10);
	RLegLiftAngleLists1.arraySetSize(8);
	RLegLiftAngleLists2.arraySetSize(6);
	LLegLowerAngleLists.arraySetSize(3);
	RLegLowerAngleLists.arraySetSize(3);
	DoubleLegsAngleLists.arraySetSize(8);
	DoubleLegsAngleLists1.arraySetSize(6);
	DoubleLegsAngleLists2.arraySetSize(10);
	HeadAngleLists.arraySetSize(2);
	//AngleListsSet
	ArmAngleLists[0]=naoJointAngles[NAO_LEFT_SHOULDER_ROLL];
	ArmAngleLists[1]=naoJointAngles[NAO_LEFT_SHOULDER_PITCH];
	ArmAngleLists[2]=naoJointAngles[NAO_LEFT_ELBOW_YAW];
	ArmAngleLists[3]=naoJointAngles[NAO_LEFT_ELBOW_ROLL];	
	ArmAngleLists[4]=naoJointAngles[NAO_RIGHT_SHOULDER_ROLL];
	ArmAngleLists[5]=naoJointAngles[NAO_RIGHT_SHOULDER_PITCH];
	ArmAngleLists[6]=naoJointAngles[NAO_RIGHT_ELBOW_YAW];
	ArmAngleLists[7]=naoJointAngles[NAO_RIGHT_ELBOW_ROLL];

	RCOMAngleLists[0]=0.12;
	RCOMAngleLists[1]=-0.34;
	RCOMAngleLists[2]=0.59;
	RCOMAngleLists[3]=-0.21;
	RCOMAngleLists[4]=-0.26;
	RCOMAngleLists[5]=0.13;
	RCOMAngleLists[6]=-0.52;
	RCOMAngleLists[7]=0.95;
	RCOMAngleLists[8]=-0.38;
	RCOMAngleLists[9]=-0.28;

	LCOMAngleLists[0]=-0.13;
	LCOMAngleLists[1]=-0.52;
	LCOMAngleLists[2]=0.95;
	LCOMAngleLists[3]=-0.38;
	LCOMAngleLists[4]=0.28;
	LCOMAngleLists[5]=-0.12;
	LCOMAngleLists[6]=-0.34;
	LCOMAngleLists[7]=0.59;
	LCOMAngleLists[8]=-0.21;
	LCOMAngleLists[9]=0.26;

	LLegLiftAngleLists[0]=naoJointAngles[NAO_LEFT_HIP_ROLL];
	LLegLiftAngleLists[1]=naoJointAngles[NAO_LEFT_HIP_PITCH];
	LLegLiftAngleLists[2]=naoJointAngles[NAO_LEFT_KNEE_PITCH];
	LLegLiftAngleLists[3]=-0.22;
	LLegLiftAngleLists[4]=-0.09;
	LLegLiftAngleLists[5]=naoJointAngles[NAO_RIGHT_HIP_ROLL];
	LLegLiftAngleLists[6]=naoJointAngles[NAO_RIGHT_HIP_PITCH];
	LLegLiftAngleLists[7]=naoJointAngles[NAO_RIGHT_KNEE_PITCH];
	LLegLiftAngleLists[8]=-0.22;
	LLegLiftAngleLists[9]=-0.09;


	LLegLiftAngleLists1[0]=naoJointAngles[NAO_LEFT_HIP_ROLL];
	LLegLiftAngleLists1[1]=naoJointAngles[NAO_LEFT_HIP_PITCH];
	LLegLiftAngleLists1[2]=naoJointAngles[NAO_LEFT_KNEE_PITCH];
	LLegLiftAngleLists1[3]=-0.16;
	LLegLiftAngleLists1[4]=naoJointAngles[NAO_RIGHT_HIP_ROLL];
	LLegLiftAngleLists1[5]=naoJointAngles[NAO_RIGHT_HIP_PITCH];
	LLegLiftAngleLists1[6]=naoJointAngles[NAO_RIGHT_KNEE_PITCH];
	LLegLiftAngleLists1[7]=-0.16;

	LLegLiftAngleLists2[0]=naoJointAngles[NAO_LEFT_HIP_ROLL];
	LLegLiftAngleLists2[1]=naoJointAngles[NAO_LEFT_HIP_PITCH];
	LLegLiftAngleLists2[2]=naoJointAngles[NAO_LEFT_KNEE_PITCH];
	LLegLiftAngleLists2[3]=naoJointAngles[NAO_RIGHT_HIP_ROLL];
	LLegLiftAngleLists2[4]=naoJointAngles[NAO_RIGHT_HIP_PITCH];
	LLegLiftAngleLists2[5]=naoJointAngles[NAO_RIGHT_KNEE_PITCH];

	RLegLiftAngleLists[0]=naoJointAngles[NAO_LEFT_HIP_ROLL]+0.45;
	RLegLiftAngleLists[1]=naoJointAngles[NAO_LEFT_HIP_PITCH];
	RLegLiftAngleLists[2]=naoJointAngles[NAO_LEFT_KNEE_PITCH];
	RLegLiftAngleLists[3]=-0.22;
	RLegLiftAngleLists[4]=0.18;
	RLegLiftAngleLists[5]=naoJointAngles[NAO_RIGHT_HIP_ROLL]+0.20;
	RLegLiftAngleLists[6]=naoJointAngles[NAO_RIGHT_HIP_PITCH];
	RLegLiftAngleLists[7]=naoJointAngles[NAO_RIGHT_KNEE_PITCH];
	RLegLiftAngleLists[8]=-0.18;
	RLegLiftAngleLists[9]=0.18;

	RLegLiftAngleLists1[0]=naoJointAngles[NAO_LEFT_HIP_ROLL];
	RLegLiftAngleLists1[1]=naoJointAngles[NAO_LEFT_HIP_PITCH];
	RLegLiftAngleLists1[2]=naoJointAngles[NAO_LEFT_KNEE_PITCH];
	RLegLiftAngleLists1[3]=-0.26;
	RLegLiftAngleLists1[4]=naoJointAngles[NAO_RIGHT_HIP_ROLL];
	RLegLiftAngleLists1[5]=naoJointAngles[NAO_RIGHT_HIP_PITCH];
	RLegLiftAngleLists1[6]=naoJointAngles[NAO_RIGHT_KNEE_PITCH];
	RLegLiftAngleLists1[7]=-0.26;

	RLegLiftAngleLists2[0]=naoJointAngles[NAO_LEFT_HIP_ROLL]+0.20;
	RLegLiftAngleLists2[1]=naoJointAngles[NAO_LEFT_HIP_PITCH];
	RLegLiftAngleLists2[2]=naoJointAngles[NAO_LEFT_KNEE_PITCH];
	RLegLiftAngleLists2[3]=naoJointAngles[NAO_RIGHT_HIP_ROLL]+0.15;
	RLegLiftAngleLists2[4]=naoJointAngles[NAO_RIGHT_HIP_PITCH];
	RLegLiftAngleLists2[5]=naoJointAngles[NAO_RIGHT_KNEE_PITCH];

	DoubleLegsAngleLists[0]=naoJointAngles[NAO_LEFT_HIP_ROLL];
	DoubleLegsAngleLists[1]=naoJointAngles[NAO_LEFT_HIP_PITCH];
	DoubleLegsAngleLists[2]=naoJointAngles[NAO_LEFT_KNEE_PITCH];
	DoubleLegsAngleLists[3]=-0.11;
	DoubleLegsAngleLists[4]=naoJointAngles[NAO_RIGHT_HIP_ROLL];
	DoubleLegsAngleLists[5]=naoJointAngles[NAO_RIGHT_HIP_PITCH];
	DoubleLegsAngleLists[6]=naoJointAngles[NAO_RIGHT_KNEE_PITCH];
	DoubleLegsAngleLists[7]=-0.11;

	DoubleLegsAngleLists1[0]=naoJointAngles[NAO_LEFT_HIP_ROLL];
	DoubleLegsAngleLists1[1]=naoJointAngles[NAO_LEFT_HIP_PITCH];
	DoubleLegsAngleLists1[2]=naoJointAngles[NAO_LEFT_KNEE_PITCH];
	DoubleLegsAngleLists1[3]=naoJointAngles[NAO_RIGHT_HIP_ROLL];
	DoubleLegsAngleLists1[4]=naoJointAngles[NAO_RIGHT_HIP_PITCH];
	DoubleLegsAngleLists1[5]=naoJointAngles[NAO_RIGHT_KNEE_PITCH];

	DoubleLegsAngleLists2[0]=naoJointAngles[NAO_LEFT_HIP_ROLL];
	DoubleLegsAngleLists2[1]=naoJointAngles[NAO_LEFT_HIP_PITCH];
	DoubleLegsAngleLists2[2]=naoJointAngles[NAO_LEFT_KNEE_PITCH];
	DoubleLegsAngleLists2[3]=-0.18;
	DoubleLegsAngleLists2[4]=0;
	DoubleLegsAngleLists2[5]=naoJointAngles[NAO_RIGHT_HIP_ROLL];
	DoubleLegsAngleLists2[6]=naoJointAngles[NAO_RIGHT_HIP_PITCH];
	DoubleLegsAngleLists2[7]=naoJointAngles[NAO_RIGHT_KNEE_PITCH];
	DoubleLegsAngleLists2[8]=-0.18;
	DoubleLegsAngleLists2[9]=0;

	HeadAngleLists[0]=naoJointAngles[NAO_HEAD_PITCH]+0.20;
	HeadAngleLists[1]=naoJointAngles[NAO_HEAD_YAW];

	//set stiffness of joints
	motionProxy.setStiffnesses("LArm", 1.0f);
	motionProxy.setStiffnesses("RArm", 1.0f);
	//Imitation
	try 
	{	
		//lift left foot
		if(LLCount>2)
		{
			DoubleSupportTimes=0;
			if(m1<1)
			{
				motionProxy.angleInterpolationWithSpeed(LegNames,RCOMAngleLists,0.05f);
				motionProxy.wbEnable(true);
				motionProxy.wbGoToBalance("RLeg",0.5f);
			}
			m1=m1+1;

			motionProxy.wbFootState("Fixed","RLeg");
			motionProxy.wbFootState("Free","LLeg");
			motionProxy.wbEnableBalanceConstraint(true,"RLeg");
			if((m1%1000)==1)motionProxy.angleInterpolationWithSpeed(LegNames,LLegLiftAngleLists,0.04f);
			motionProxy.setAngles(LegWithoutAnklePitchAndRollNames,LLegLiftAngleLists2,0.06f);
			motionProxy.setAngles("RAnkleRoll",-0.45, 0.10f);
			motionProxy.setAngles(ArmNames, ArmAngleLists, 0.15f);
			motionProxy.setAngles(HeadNames,HeadAngleLists,0.10f);
			motionProxy.setAngles("LWristYaw",-0.37,0.15f);
			motionProxy.setAngles("RWristYaw",0.37,0.15f);
		}

		//lift right foot
		else if(RLCount>2)
		{
			DoubleSupportTimes=0;
			if(m2<1)
			{
				motionProxy.angleInterpolationWithSpeed(LegNames,LCOMAngleLists,0.07f);
				motionProxy.wbEnable(true);
				motionProxy.wbGoToBalance("LLeg",0.5f);
			}
			m2=m2+1;

			motionProxy.wbFootState("Fixed","LLeg");
			motionProxy.wbFootState("Free","RLeg");
			motionProxy.wbEnableBalanceConstraint(true,"LLeg");
			if((m2%1000)==1)motionProxy.angleInterpolationWithSpeed(LegNames,RLegLiftAngleLists,0.04f);
			motionProxy.setAngles(LegWithoutAnklePitchAndRollNames,RLegLiftAngleLists2,0.06f);
			motionProxy.setAngles("LAnkleRoll",0.45, 0.10f);
			motionProxy.setAngles(ArmNames,ArmAngleLists,0.15f);
			motionProxy.setAngles(HeadNames,HeadAngleLists,0.10f);
			motionProxy.setAngles("LWristYaw",-0.37,0.15f);
			motionProxy.setAngles("RWristYaw",0.37,0.15f);
		}
		else 
		{
			//left foot landing
			if(m1>0)
			{
				RHP=motionProxy.getAngles("RHipPitch",false);
				motionProxy.angleInterpolationWithSpeed("RHipPitch",RHP[0]-0.15,0.10f);	
				HR=10;
				HL=0;

				while(HR-HL>-5)
				{
					RAR=motionProxy.getAngles("RAnkleRoll",false);
					RAP=motionProxy.getAngles("RAnklePitch",false);
					RKP=motionProxy.getAngles("RKneePitch",false);
					RHP=motionProxy.getAngles("RHipPitch",false);
					RHR=motionProxy.getAngles("RHipRoll",false);
					RHYP=motionProxy.getAngles("RHipYawPitch",false);
					LHYP=motionProxy.getAngles("LHipYawPitch",false);
					LHR=motionProxy.getAngles("LHipRoll",false);
					LHP=motionProxy.getAngles("LHipPitch",false);
					LKP=motionProxy.getAngles("LKneePitch",false);
					LAP=motionProxy.getAngles("LAnklePitch",false);
					LAR=motionProxy.getAngles("LAnkleRoll",false);

					c1=cos(RAR[0]);s1=sin(RAR[0]);
					c2=cos(RAP[0]);s2=sin(RAP[0]);
					c3=cos(RKP[0]);s3=sin(RKP[0]);
					c4=cos(RHP[0]);s4=sin(RHP[0]);
					c5=cos(RHR[0]);s5=sin(RHR[0]);
					c6=cos(RHYP[0]);s6=sin(RHYP[0]);
					c7=cos(LHYP[0]);s7=sin(LHYP[0]);
					c8=cos(-LHR[0]);s8=sin(-LHR[0]);
					c9=cos(-LHP[0]);s9=sin(-LHP[0]);
					c10=cos(-LKP[0]);s10=sin(-LKP[0]);
					cp1=cos(PI/4);sp1=sin(PI/4);
					cp2=cos(-PI/4);sp2=sin(-PI/4);

					R1_1=0;
					R1_2=-s1;
					R1_3=c1;

					R2_1=c2*R1_1 + s2*R1_3;
					R2_2=R1_2;
					R2_3=c2*R1_3 - s2*R1_1;

					R3_1=c3*R2_1 + s3*R2_3;
					R3_2=R2_2;
					R3_3=c3*R2_3 - s3*R2_1;

					R4_1=c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3);
					R4_2=-s1;
					R4_3=c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2);

					R5_1=R4_1;
					R5_2=R4_2*c5-R4_3*s5;
					R5_3=R4_3*c5+R4_2*s5;

					R56_1=R5_1;
					R56_2=(sqrt(float(2))*R5_3)/2;
					R56_3=(sqrt(float(2))*R5_2)/2 + (sqrt(float(2))*R5_3)/2;

					R6_1=c6*R5_1 - R5_2*cp1*s6 + R5_3*s6*sp1;
					R6_2= R5_1*cp2*s6 - R5_3*(cp1*sp2 + c6*cp2*sp1) - R5_2*(sp1*sp2 - c6*cp1*cp2);
					R6_3=R5_2*(cp2*sp1 + c6*cp1*sp2) + R5_3*(cp1*cp2 - c6*sp1*sp2) + R5_1*s6*sp2;

					R7_1=c7*R6_1 - cp2*R6_2*s7 + R6_3*s7*sp2;
					R7_2=cp1*R6_1*s7 - R6_3*(cp2*sp1 + c7*cp1*sp2) - R6_2*(sp1*sp2 - c7*cp1*cp2);
					R7_3=R6_2*(cp1*sp2 + c7*cp2*sp1) + R6_3*(cp1*cp2 - c7*sp1*sp2) + R6_1*s7*sp1;

					x31=c10*(R7_1*c9 + s9*(R7_3*c8 + R7_2*s8)) - s10*(R7_1*s9 - c9*(R7_3*c8 + R7_2*s8));
					x32=R7_2*c8 - R7_3*s8;
					x33=-c10*(R7_1*s9 - c9*(R7_3*c8 + R7_2*s8)) - s10*(R7_1*c9 + s9*(R7_3*c8 + R7_2*s8));

					R8_1=R7_1;
					R8_2=c8*R7_2-s8*R7_3;
					R8_3=c8*R7_3+s8*R7_2;

					R9_1=c9*R8_1 + s9*R8_3;
					R9_2=R8_2;
					R9_3=c9*R8_3 - s9*R8_1;

					R10_1=c10*R9_1 + s10*R9_3;
					R10_2=R9_2;
					R10_3=c10*R9_3 - s10*R9_1;

					HR=R2_3*102.9+R3_3*100+R5_3*50;
					HL=R7_3*50+R9_3*100+R10_3*102.9;

					if(x33>0)
					{
						c11=x33*sqrt((1/(x31*x31 + x33*x33)));
						s11=-x31*sqrt((1/(x31*x31 + x33*x33)));
						c12=(x31*x31 + x33*x33)*sqrt(1/(x31*x31 + x32*x32 + x33*x33))*sqrt(1/(x31*x31 + x33*x33));
						s12=x32*sqrt(1/(x31*x31 + x32*x32 + x33*x33));
						LandingLAP=-asin(s11);
						LandingLAR=-asin(s12);

					}

					if(x33<=0)
					{
						c11=-x33*sqrt((1/(x31*x31 + x33*x33)));
						s11=x31*sqrt((1/(x31*x31 + x33*x33)));
						c12=(x31*x31 + x33*x33)*sqrt(1/(x31*x31 + x32*x32 + x33*x33))*sqrt(1/(x31*x31 + x33*x33));
						s12=-x32*sqrt(1/(x31*x31 + x32*x32 + x33*x33));
						LandingLAP=-asin(s11);
						LandingLAR=-asin(s12);
					}


					LLegLowerAngleLists[0]=RAP[0]-0.15;
					LLegLowerAngleLists[1]=RKP[0]+0.30;
					LLegLowerAngleLists[2]=RHP[0]-0.15;
					motionProxy.setAngles("LAnklePitch",LandingLAP,0.30f);
					motionProxy.setAngles("LAnkleRoll",LandingLAR,0.30f);
					if(HR-HL<=-5) motionProxy.angleInterpolationWithSpeed(LLegLowerNames,LLegLowerAngleLists,0.10f);
					else motionProxy.setAngles(LLegLowerNames,LLegLowerAngleLists,0.10f);	
				}
			}

			//right foot landing
			if(m2>0)
			{
				LHP=motionProxy.getAngles("LHipPitch",false);
				motionProxy.angleInterpolationWithSpeed("LHipPitch",LHP[0]-0.15,0.10f);	

				HL=10;
				HR=0;

				while(HL-HR>-5)
				{
					RAR=motionProxy.getAngles("RAnkleRoll",false);
					RAP=motionProxy.getAngles("RAnklePitch",false);
					RKP=motionProxy.getAngles("RKneePitch",false);
					RHP=motionProxy.getAngles("RHipPitch",false);
					RHR=motionProxy.getAngles("RHipRoll",false);
					RHYP=motionProxy.getAngles("RHipYawPitch",false);
					LHYP=motionProxy.getAngles("LHipYawPitch",false);
					LHR=motionProxy.getAngles("LHipRoll",false);
					LHP=motionProxy.getAngles("LHipPitch",false);
					LKP=motionProxy.getAngles("LKneePitch",false);
					LAP=motionProxy.getAngles("LAnklePitch",false);
					LAR=motionProxy.getAngles("LAnkleRoll",false);

					c1=cos(LAR[0]);s1=sin(LAR[0]);
					c2=cos(LAP[0]);s2=sin(LAP[0]);
					c3=cos(LKP[0]);s3=sin(LKP[0]);
					c4=cos(LHP[0]);s4=sin(LHP[0]);
					c5=cos(LHR[0]);s5=sin(LHR[0]);
					c6=cos(-LHYP[0]);s6=sin(-LHYP[0]);
					c7=cos(-RHYP[0]);s7=sin(-RHYP[0]);
					c8=cos(-RHR[0]);s8=sin(-RHR[0]);
					c9=cos(-RHP[0]);s9=sin(-RHP[0]);
					c10=cos(-RKP[0]);s10=sin(-RKP[0]);
					cp2=cos(PI/4);sp2=sin(PI/4);
					cp1=cos(-PI/4);sp1=sin(-PI/4);

					R1_1=0;
					R1_2=-s1;
					R1_3=c1;

					R2_1=c2*R1_1 + s2*R1_3;
					R2_2=R1_2;
					R2_3=c2*R1_3 - s2*R1_1;

					R3_1=c3*R2_1 + s3*R2_3;
					R3_2=R2_2;
					R3_3=c3*R2_3 - s3*R2_1;

					R4_1=c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3);
					R4_2=-s1;
					R4_3=c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2);

					R5_1=R4_1;
					R5_2=R4_2*c5-R4_3*s5;
					R5_3=R4_3*c5+R4_2*s5;

					R56_1=R5_1;
					R56_2=(sqrt(float(2))*R5_3)/2;
					R56_3=(sqrt(float(2))*R5_2)/2 + (sqrt(float(2))*R5_3)/2;

					R6_1=c6*R5_1 - R5_2*cp1*s6 + R5_3*s6*sp1;
					R6_2= R5_1*cp2*s6 - R5_3*(cp1*sp2 + c6*cp2*sp1) - R5_2*(sp1*sp2 - c6*cp1*cp2);
					R6_3=R5_2*(cp2*sp1 + c6*cp1*sp2) + R5_3*(cp1*cp2 - c6*sp1*sp2) + R5_1*s6*sp2;

					R7_1=c7*R6_1 - cp2*R6_2*s7 + R6_3*s7*sp2;
					R7_2=cp1*R6_1*s7 - R6_3*(cp2*sp1 + c7*cp1*sp2) - R6_2*(sp1*sp2 - c7*cp1*cp2);
					R7_3=R6_2*(cp1*sp2 + c7*cp2*sp1) + R6_3*(cp1*cp2 - c7*sp1*sp2) + R6_1*s7*sp1;

					x31=c10*(R7_1*c9 + s9*(R7_3*c8 + R7_2*s8)) - s10*(R7_1*s9 - c9*(R7_3*c8 + R7_2*s8));
					x32=R7_2*c8 - R7_3*s8;
					x33=-c10*(R7_1*s9 - c9*(R7_3*c8 + R7_2*s8)) - s10*(R7_1*c9 + s9*(R7_3*c8 + R7_2*s8));


					R8_1=R7_1;
					R8_2=c8*R7_2-s8*R7_3;
					R8_3=c8*R7_3+s8*R7_2;

					R9_1=c9*R8_1 + s9*R8_3;
					R9_2=R8_2;
					R9_3=c9*R8_3 - s9*R8_1;

					R10_1=c10*R9_1 + s10*R9_3;
					R10_2=R9_2;
					R10_3=c10*R9_3 - s10*R9_1;

					HL=R2_3*102.9+R3_3*100+R5_3*50;
					HR=R7_3*50+R9_3*100+R10_3*102.9;

					if(x33>0)
					{
						c11=x33*sqrt((1/(x31*x31 + x33*x33)));
						s11=-x31*sqrt((1/(x31*x31 + x33*x33)));
						c12=(x31*x31 + x33*x33)*sqrt(1/(x31*x31 + x32*x32 + x33*x33))*sqrt(1/(x31*x31 + x33*x33));
						s12=x32*sqrt(1/(x31*x31 + x32*x32 + x33*x33));
						LandingRAP=-asin(s11);
						LandingRAR=-asin(s12);
					}

					if(x33<=0)
					{
						c11=-x33*sqrt((1/(x31*x31 + x33*x33)));
						s11=x31*sqrt((1/(x31*x31 + x33*x33)));
						c12=(x31*x31 + x33*x33)*sqrt(1/(x31*x31 + x32*x32 + x33*x33))*sqrt(1/(x31*x31 + x33*x33));
						s12=-x32*sqrt(1/(x31*x31 + x32*x32 + x33*x33));
						LandingRAP=-asin(s11);
						LandingRAR=-asin(s12);
					}

					RLegLowerAngleLists[0]=LAP[0]-0.15;
					RLegLowerAngleLists[1]=LKP[0]+0.30;
					RLegLowerAngleLists[2]=LHP[0]-0.15;
					motionProxy.setAngles("RAnklePitch",LandingRAP,0.30f);
					motionProxy.setAngles("RAnkleRoll",LandingRAR,0.30f);
					if(HL-HR<=-5)motionProxy.angleInterpolationWithSpeed(RLegLowerNames,RLegLowerAngleLists,0.10f);
					else motionProxy.setAngles(RLegLowerNames,RLegLowerAngleLists,0.10f);
				}
			}
			m1=0;
			m2=0;

			//walking
			if(((Ld>0.05&&Rd>0.05)||varepsilon>0.50||varepsilon<-0.50)&&m>1&&n!=m)
			{
				DoubleSupportTimes=0;
				n=m;
			    motionProxy.setMoveArmsEnabled(false, false);
				motionProxy.moveTo(dist*sin(theta), dist*cos(theta), varepsilon);
			}
			motionProxy.wbEnable(true);
			motionProxy.wbFootState("Plane","Legs");
			motionProxy.wbEnableBalanceConstraint(true,"Legs");

			//double-foot model
			if((Ld1<0.003&&Rd1<0.003&&FootHeight>-0.03&&FootHeight<0.03&&t>0&&LLowerCount>5&&RLowerCount>5)||t==0)
			{
				DoubleSupportTimes++;
			   if(DoubleSupportTimes<=1) 
			   {
				   motionProxy.angleInterpolationWithSpeed(LegWithoutAnkleRollNames,DoubleLegsAngleLists,0.15f);
			   }
			   else 
			   {
				   motionProxy.angleInterpolationWithSpeed(LegWithoutAnklePitchAndRollNames,DoubleLegsAngleLists1,0.15f);
			   }
			}

			motionProxy.setAngles("LHipYawPitch",0,0.15f);
			motionProxy.setAngles("RHipYawPitch",0,0.15f);
			motionProxy.setAngles(ArmNames,ArmAngleLists,0.31f);
			motionProxy.setAngles(HeadNames,HeadAngleLists,0.10f);
			motionProxy.setAngles("LWristYaw",-0.72,0.10f);
			motionProxy.setAngles("RWristYaw",0.72,0.10f);
			motionProxy.wbEnable(false);
			m3=m3+1;
		}

		clockRecord=clock();
		time_total=clockRecord-clockInitial;

	}catch (const AL::ALError& e){
		cerr << "Caught exception: " << e.what() << endl;
		exit();
	}
}
void naoRobot::ctrHandState(_kinectData kinectData)
{
	if(kinectData.handState[LEFT_HAND])
	{
		if(!isLeftHandOpen)
		{
			motionProxy.openHand("LHand");
			isLeftHandOpen = true;
		}
	}
	else
	{
		if(isLeftHandOpen)//如果手已经打开
		{
			motionProxy.closeHand("LHand");
			isLeftHandOpen = false;
		}
	}
	if(kinectData.handState[RIGHT_HAND])
	{
		if(!isRightHandOpen)
		{
			motionProxy.openHand("RHand");
			isRightHandOpen = true;
		}
	}
	else
	{
		if(isRightHandOpen)//如果手已经打开
		{
			motionProxy.closeHand("RHand");
			isRightHandOpen = false;
		}
	}

}
void naoRobot::getBodyAngles(_kinectData kinectData)
{
	//Head Joint Angles 
	naoJointAngles[NAO_HEAD_PITCH] =  kinectData.headRotationAngle.pitch;
	naoJointAngles[NAO_HEAD_YAW] = kinectData.headRotationAngle.yaw;
	//Preparation angles
	naoJointAngles[NAO_LEFT_ALPHA] = acos(getCos(jointVector[LEFT_SHOULDER_TO_ELBOW],jointVector[LEFT_UPPER_TORSO]));
	naoJointAngles[NAO_LEFT_BETA] = acos(getCos(jointVector[LEFT_SHOULDER_TO_ELBOW],jointVector[LEFT_REFERENCE1]));
	naoJointAngles[NAO_LEFT_REFERENCE1] = acos(getCos(jointVector[LEFT_SHOULDER_TO_ELBOW],jointVector[SPINE_MID_TO_SHOULDER]));
	naoJointAngles[NAO_LEFT_REFERENCE2] = acos(getCos(jointVector[LEFT_ELBOW_TO_WRIST],jointVector[LEFT_NORMAL_BY_SHOULDER_UPPER_ARM]));
	if(naoJointAngles[NAO_LEFT_REFERENCE2]<=PI/2)naoJointAngles[NAO_LEFT_TWO_PLANES] = -acos(getCos(jointVector[LEFT_NORMAL_BY_SHOULDER_UPPER_ARM],jointVector[LEFT_NORMAL_BY_UPPER_LOWER_ARM]));
	else naoJointAngles[NAO_LEFT_TWO_PLANES] = acos(getCos(jointVector[LEFT_NORMAL_BY_SHOULDER_UPPER_ARM],jointVector[LEFT_NORMAL_BY_UPPER_LOWER_ARM]));
	naoJointAngles[NAO_LEFT_GAMMA] = acos(getCos(jointVector[LEFT_HIP_TO_KNEE],jointVector[LEFT_LOWER_TORSO]));
	naoJointAngles[NAO_LEFT_DELTA] = acos(getCos(jointVector[LEFT_HIP_TO_KNEE],jointVector[LEFT_REFERENCE2]));

	naoJointAngles[NAO_RIGHT_ALPHA] = acos(getCos(jointVector[RIGHT_SHOULDER_TO_ELBOW],jointVector[RIGHT_UPPER_TORSO]));
	naoJointAngles[NAO_RIGHT_BETA] = acos(getCos(jointVector[RIGHT_SHOULDER_TO_ELBOW],jointVector[RIGHT_REFERENCE1]));
	naoJointAngles[NAO_RIGHT_REFERENCE1] = acos(getCos(jointVector[RIGHT_SHOULDER_TO_ELBOW],jointVector[SPINE_MID_TO_SHOULDER]));
	naoJointAngles[NAO_RIGHT_REFERENCE2] = acos(getCos(jointVector[RIGHT_ELBOW_TO_WRIST],jointVector[RIGHT_NORMAL_BY_SHOULDER_UPPER_ARM]));
	if(naoJointAngles[NAO_RIGHT_REFERENCE2]<=PI/2)naoJointAngles[NAO_RIGHT_TWO_PLANES] = -acos(getCos(jointVector[RIGHT_NORMAL_BY_SHOULDER_UPPER_ARM],jointVector[RIGHT_NORMAL_BY_UPPER_LOWER_ARM]));
	else naoJointAngles[NAO_RIGHT_TWO_PLANES] = acos(getCos(jointVector[RIGHT_NORMAL_BY_SHOULDER_UPPER_ARM],jointVector[RIGHT_NORMAL_BY_UPPER_LOWER_ARM]));
	naoJointAngles[NAO_RIGHT_GAMMA] = acos(getCos(jointVector[RIGHT_HIP_TO_KNEE],jointVector[RIGHT_LOWER_TORSO]));
	naoJointAngles[NAO_RIGHT_DELTA] = acos(getCos(jointVector[RIGHT_HIP_TO_KNEE],jointVector[RIGHT_REFERENCE2]));

	naoJointAngles[VAREPSILON_REFERENCE] = acos(getCos(jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[SPINE_MID_TO_BASE]));
	//ElbowRoll
	naoJointAngles[NAO_LEFT_ELBOW_ROLL]=-acos(getCos(jointVector[LEFT_SHOULDER_TO_ELBOW],jointVector[LEFT_ELBOW_TO_WRIST]));
	naoJointAngles[NAO_RIGHT_ELBOW_ROLL]=acos(getCos(jointVector[RIGHT_SHOULDER_TO_ELBOW],jointVector[RIGHT_ELBOW_TO_WRIST]));
	//ShoulderRoll
	/*naoJointAngles[NAO_LEFT_SHOULDER_ROLL]=acos(getCos(jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[LEFT_SHOULDER_TO_ELBOW]))-PI/2;*/
	naoJointAngles[NAO_LEFT_SHOULDER_ROLL]=PI/2-acos(getCos(jointVector[LEFT_SHOULDER_TO_ELBOW],jointVector[LEFT_REFERENCE1]));
	/*naoJointAngles[NAO_RIGHT_SHOULDER_ROLL]=acos(getCos(jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[RIGHT_SHOULDER_TO_ELBOW]));*/
	naoJointAngles[NAO_RIGHT_SHOULDER_ROLL]=-PI/2+acos(getCos(jointVector[RIGHT_SHOULDER_TO_ELBOW],jointVector[RIGHT_REFERENCE1]));
	//ShoulderPitch
	/*if(jointVector[LEFT_SHOULDER_TO_ELBOW].z<0)
		naoJointAngles[NAO_LEFT_SHOULDER_PITCH]=atan(jointVector[LEFT_SHOULDER_TO_ELBOW].y/jointVector[LEFT_SHOULDER_TO_ELBOW].z);*/
	/*else
	    naoJointAngles[NAO_LEFT_SHOULDER_PITCH]=jointVector[LEFT_SHOULDER_TO_ELBOW].y<0?-atan(jointVector[LEFT_SHOULDER_TO_ELBOW].z/jointVector[LEFT_SHOULDER_TO_ELBOW].y)+PI/2:-atan(jointVector[LEFT_SHOULDER_TO_ELBOW].z/jointVector[LEFT_SHOULDER_TO_ELBOW].y)-PI/2;*/
	if(naoJointAngles[NAO_LEFT_REFERENCE1]<=PI/2)
		naoJointAngles[NAO_LEFT_SHOULDER_PITCH]=-acos(cos(naoJointAngles[NAO_LEFT_ALPHA])/cos(PI/2-naoJointAngles[NAO_LEFT_BETA]));
	else
		naoJointAngles[NAO_LEFT_SHOULDER_PITCH]=acos(cos(naoJointAngles[NAO_LEFT_ALPHA])/cos(PI/2-naoJointAngles[NAO_LEFT_BETA]));

	/*if(jointVector[RIGHT_SHOULDER_TO_ELBOW].z<0)
		naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]=atan(jointVector[RIGHT_SHOULDER_TO_ELBOW].y/jointVector[RIGHT_SHOULDER_TO_ELBOW].z);
	else
		naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]=jointVector[RIGHT_SHOULDER_TO_ELBOW].y<0?-atan(jointVector[RIGHT_SHOULDER_TO_ELBOW].z/jointVector[RIGHT_SHOULDER_TO_ELBOW].y)+PI/2:-atan(jointVector[RIGHT_SHOULDER_TO_ELBOW].z/jointVector[RIGHT_SHOULDER_TO_ELBOW].y)-PI/2;*/
	if(naoJointAngles[NAO_RIGHT_REFERENCE1]<=PI/2)
		naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]=-acos(cos(naoJointAngles[NAO_RIGHT_ALPHA])/cos(PI/2-naoJointAngles[NAO_RIGHT_BETA]));
	else
		naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]=acos(cos(naoJointAngles[NAO_RIGHT_ALPHA])/cos(PI/2-naoJointAngles[NAO_RIGHT_BETA]));
	//ElbowYaw
	/*naoJointAngles[NAO_LEFT_ELBOW_YAW]=jointVector[LEFT_NORMAL_BY_UPPER_LOWER_ARM].x<0?-acos(getCos(jointVector[LEFT_NORMAL_BY_UPPER_LOWER_ARM],jointVector[LEFT_NORMAL_BY_SHOULDER_UPPER_ARM])):acos(getCos(jointVector[LEFT_NORMAL_BY_UPPER_LOWER_ARM],jointVector[LEFT_NORMAL_BY_SHOULDER_UPPER_ARM]));*/
	if(naoJointAngles[NAO_LEFT_TWO_PLANES]<PI/2&&naoJointAngles[NAO_LEFT_TWO_PLANES]>-119.5*PI/180)naoJointAngles[NAO_LEFT_ELBOW_YAW]=naoJointAngles[NAO_LEFT_TWO_PLANES];
	else naoJointAngles[NAO_LEFT_ELBOW_YAW]=-119.5*PI/180;
	/*naoJointAngles[NAO_RIGHT_ELBOW_YAW]=jointVector[RIGHT_NORMAL_BY_UPPER_LOWER_ARM].x<0?-acos(getCos(jointVector[RIGHT_NORMAL_BY_UPPER_LOWER_ARM],jointVector[RIGHT_NORMAL_BY_SHOULDER_UPPER_ARM])):acos(getCos(jointVector[RIGHT_NORMAL_BY_UPPER_LOWER_ARM],jointVector[RIGHT_NORMAL_BY_SHOULDER_UPPER_ARM]));*/
	if(naoJointAngles[NAO_RIGHT_TWO_PLANES]>-PI/2&&naoJointAngles[NAO_RIGHT_TWO_PLANES]<119.5*PI/180)naoJointAngles[NAO_RIGHT_ELBOW_YAW]=-naoJointAngles[NAO_RIGHT_TWO_PLANES];
	else naoJointAngles[NAO_RIGHT_ELBOW_YAW]=119.5*PI/180;
	//HipRoll
	/*naoJointAngles[NAO_LEFT_HIP_ROLL]=acos(getCos(jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[LEFT_HIP_TO_KNEE]))-PI/2;*/
	naoJointAngles[NAO_LEFT_HIP_ROLL]=PI/2-acos(cos(naoJointAngles[NAO_LEFT_DELTA])/cos(PI/2-naoJointAngles[NAO_LEFT_GAMMA]));
	/*naoJointAngles[NAO_RIGHT_HIP_ROLL]=acos(getCos(jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[RIGHT_HIP_TO_KNEE]))-PI/2;*/
	naoJointAngles[NAO_RIGHT_HIP_ROLL]=-PI/2+acos(cos(naoJointAngles[NAO_RIGHT_DELTA])/cos(PI/2-naoJointAngles[NAO_RIGHT_GAMMA]));
	
	//HipPitch
	/*naoJointAngles[NAO_LEFT_HIP_PITCH]=acos(getCos(jointVector[LEFT_HIP_TO_KNEE],jointVector[NORMAL_BY_HIP_SPINE]))-PI/2;*/
	naoJointAngles[NAO_LEFT_HIP_PITCH]=-PI/2+acos(getCos(jointVector[LEFT_HIP_TO_KNEE],jointVector[LEFT_LOWER_TORSO]));
	/*naoJointAngles[NAO_RIGHT_HIP_PITCH]=acos(getCos(jointVector[RIGHT_HIP_TO_KNEE],jointVector[NORMAL_BY_HIP_SPINE]))-PI/2;*/
	naoJointAngles[NAO_RIGHT_HIP_PITCH]=-PI/2+acos(getCos(jointVector[RIGHT_HIP_TO_KNEE],jointVector[RIGHT_LOWER_TORSO]));
	//KneePitch
	naoJointAngles[NAO_LEFT_KNEE_PITCH]=acos(getCos(jointVector[LEFT_HIP_TO_KNEE],jointVector[LEFT_KNEE_TO_ANKLE]));
	naoJointAngles[NAO_RIGHT_KNEE_PITCH]=acos(getCos(jointVector[RIGHT_HIP_TO_KNEE],jointVector[RIGHT_KNEE_TO_ANKLE]));
	//AnklePitch
	naoJointAngles[NAO_LEFT_ANKLE_PITCH]=-acos(getCos(jointVector[LEFT_KNEE_TO_ANKLE],jointVector[LEFT_ANKLE_TO_FOOT]))+PI/2-1.40;
	naoJointAngles[NAO_RIGHT_ANKLE_PITCH]=-acos(getCos(jointVector[RIGHT_KNEE_TO_ANKLE],jointVector[RIGHT_ANKLE_TO_FOOT]))+PI/2-0.90;
	//joint limitation
	//LArm
	if(naoJointAngles[NAO_LEFT_SHOULDER_PITCH]<-2.0857 )naoJointAngles[NAO_LEFT_SHOULDER_PITCH]=-2.0857; 
	if(naoJointAngles[NAO_LEFT_SHOULDER_PITCH]>2.0857 )naoJointAngles[NAO_LEFT_SHOULDER_PITCH]=2.0857;

	if(naoJointAngles[NAO_LEFT_SHOULDER_ROLL]<-0.3142 )naoJointAngles[NAO_LEFT_SHOULDER_ROLL]=-0.3142; 
	if(naoJointAngles[NAO_LEFT_SHOULDER_ROLL]>1.3265 )naoJointAngles[NAO_LEFT_SHOULDER_ROLL]=1.3265;

	if(naoJointAngles[NAO_LEFT_ELBOW_YAW]<-2.0857)naoJointAngles[NAO_LEFT_ELBOW_YAW]=-2.0857; 
	if(naoJointAngles[NAO_LEFT_ELBOW_YAW]>2.0857 )naoJointAngles[NAO_LEFT_ELBOW_YAW]=2.0857;

	if(naoJointAngles[NAO_LEFT_ELBOW_ROLL]<-1.5446)naoJointAngles[NAO_LEFT_ELBOW_ROLL]=-1.5446; 
	if(naoJointAngles[NAO_LEFT_ELBOW_ROLL]>-0.0349 )naoJointAngles[NAO_LEFT_ELBOW_ROLL]=-0.0349;
	//RArm
	if(naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]<-2.0857 )naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]=-2.0857; 
	if(naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]>2.0857 )naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]=2.0857;

	if(naoJointAngles[NAO_RIGHT_SHOULDER_ROLL]<-1.3265 )naoJointAngles[NAO_RIGHT_SHOULDER_ROLL]=-1.3265; 
	if(naoJointAngles[NAO_RIGHT_SHOULDER_ROLL]>0.3142 )naoJointAngles[NAO_RIGHT_SHOULDER_ROLL]=0.3142;

	if(naoJointAngles[NAO_RIGHT_ELBOW_YAW]<-2.0857)naoJointAngles[NAO_RIGHT_ELBOW_YAW]=-2.0857; 
	if(naoJointAngles[NAO_RIGHT_ELBOW_YAW]>2.0857 )naoJointAngles[NAO_RIGHT_ELBOW_YAW]=2.0857;

	if(naoJointAngles[NAO_RIGHT_ELBOW_ROLL]<0.0349)naoJointAngles[NAO_RIGHT_ELBOW_ROLL]=0.0349; 
	if(naoJointAngles[NAO_RIGHT_ELBOW_ROLL]>1.5446)naoJointAngles[NAO_RIGHT_ELBOW_ROLL]=1.5446;
	//LLeg
	if(naoJointAngles[NAO_LEFT_HIP_ROLL]<-0.379472)naoJointAngles[NAO_LEFT_HIP_ROLL]=-0.379472;
	if(naoJointAngles[NAO_LEFT_HIP_ROLL]>0.790477)naoJointAngles[NAO_LEFT_HIP_ROLL]=0.790477;

	if(naoJointAngles[NAO_LEFT_HIP_PITCH]<-1.535889)naoJointAngles[NAO_LEFT_HIP_PITCH]=-1.535889;
	if(naoJointAngles[NAO_LEFT_HIP_PITCH]>0.484090)naoJointAngles[NAO_LEFT_HIP_PITCH]=0.484090;

	if(naoJointAngles[NAO_LEFT_KNEE_PITCH]<-0.092346)naoJointAngles[NAO_LEFT_KNEE_PITCH]=-0.092346;
	if(naoJointAngles[NAO_LEFT_KNEE_PITCH]>02.112528)naoJointAngles[NAO_LEFT_KNEE_PITCH]=2.112528;

	if(naoJointAngles[NAO_LEFT_ANKLE_PITCH]<-1.189516)naoJointAngles[NAO_LEFT_ANKLE_PITCH]=-1.189516;
	if(naoJointAngles[NAO_LEFT_ANKLE_PITCH]>0.922747)naoJointAngles[NAO_LEFT_ANKLE_PITCH]=0.922747;

	if(naoJointAngles[NAO_LEFT_ANKLE_ROLL]<-0.397880)naoJointAngles[NAO_LEFT_ANKLE_ROLL]=-0.397880;
	if(naoJointAngles[NAO_LEFT_ANKLE_ROLL]>0.769001)naoJointAngles[NAO_LEFT_ANKLE_ROLL]=0.769001;
	//RLeg
	if(naoJointAngles[NAO_RIGHT_HIP_ROLL]<-0.790477)naoJointAngles[NAO_RIGHT_HIP_ROLL]=-0.790477;
	if(naoJointAngles[NAO_RIGHT_HIP_ROLL]>0.379472)naoJointAngles[NAO_RIGHT_HIP_ROLL]=0.379472;

	if(naoJointAngles[NAO_RIGHT_HIP_PITCH]<-1.535889)naoJointAngles[NAO_RIGHT_HIP_PITCH]=-1.535889;
	if(naoJointAngles[NAO_RIGHT_HIP_PITCH]>0.484090)naoJointAngles[NAO_RIGHT_HIP_PITCH]=0.484090;

	if(naoJointAngles[NAO_RIGHT_KNEE_PITCH]<-0.103083)naoJointAngles[NAO_RIGHT_KNEE_PITCH]=-0.103083;
	if(naoJointAngles[NAO_RIGHT_KNEE_PITCH]>2.112528)naoJointAngles[NAO_RIGHT_KNEE_PITCH]=2.112528;

	if(naoJointAngles[NAO_RIGHT_ANKLE_PITCH]<-1.186448)naoJointAngles[NAO_RIGHT_ANKLE_PITCH]=-1.186448;
	if(naoJointAngles[NAO_RIGHT_ANKLE_PITCH]>0.932056)naoJointAngles[NAO_RIGHT_ANKLE_PITCH]=0.932056;

	if(naoJointAngles[NAO_RIGHT_ANKLE_ROLL]<-0.768992)naoJointAngles[NAO_RIGHT_ANKLE_ROLL]=-0.768992;
	if(naoJointAngles[NAO_RIGHT_ANKLE_ROLL]>0.397935)naoJointAngles[NAO_RIGHT_ANKLE_ROLL]=0.397935;

	//Calculation cost
	QueryPerformanceCounter(&large_interger);  
	time_angle2 = large_interger.QuadPart;  
	time_angle=(time_angle2 - time_angle1) * 1000 / dff;

	naoJointAngles[SHOULDER_SPINE1]=acos(getCos(jointVector[LEFT_SHOULDER_TO_SPINE_SHOULDER],jointVector[SPINE_MID_TO_SHOULDER]));
	naoJointAngles[SHOULDER_SPINE2]=acos(getCos(jointVector[RIGHT_SHOULDER_TO_SPINE_SHOULDER],jointVector[SPINE_MID_TO_SHOULDER]));

	cout<<motionProxy.getCollisionProtectionEnabled("LArm")<<","<<motionProxy.getCollisionProtectionEnabled("RArm")<<endl;


	//FootHeight
	FootHeight=kinectData.bodyJointsArray[JointType_FootLeft].y-kinectData.bodyJointsArray[JointType_FootRight].y;

	//Displacement
	if(t==0)
	{
		Lprex1=0;Lprey1=0;Lcurx1=0;Lcury1=0;Ldisx1=0;Rdisx1=0;disx1=0;Rprex1=0;Rprey1=0;Rcurx1=0;Rcury1=0;Ldisy1=0;Rdisy1=0;disy1=0;prea1=0;cura1=0;Ld1=0;Rd1=0;
		jointVector[HIP_REFERENCE].x=1;
		jointVector[HIP_REFERENCE].y=0;
		jointVector[HIP_REFERENCE].z=0;
		jointVector[HIP_REFERENCE_INIT].x=jointVector[LEFT_HIP_TO_RIGHT_HIP].x;
		jointVector[HIP_REFERENCE_INIT].y=jointVector[LEFT_HIP_TO_RIGHT_HIP].y;
		jointVector[HIP_REFERENCE_INIT].z=jointVector[LEFT_HIP_TO_RIGHT_HIP].z;
		jointVector[PRE_LEFT_HIP_TO_RIGHT_HIP]=jointVector[LEFT_HIP_TO_RIGHT_HIP];
		LLCount=0;
		RLCount=0;
		LLowerCount=0;
		RLowerCount=0;
	}
	t=t+1;

	if(FootHeight>0.05)LLCount++;
	else LLCount=0;
	if(FootHeight<-0.05)RLCount++;
	else RLCount=0;

	if(FootHeight<0.05&&FootHeight>-0.05)
	{
	  LLowerCount++;
	  RLowerCount++;
	}
	else 
	{
	  LLowerCount=0;
	  RLowerCount=0;
	}

	Lcurx1=-kinectData.bodyJointsArray[JointType_FootLeft].z;
	Lcury1=-kinectData.bodyJointsArray[JointType_FootLeft].x;
	Rcurx1=-kinectData.bodyJointsArray[JointType_FootRight].z;
	Rcury1=-kinectData.bodyJointsArray[JointType_FootRight].x;

	Ldisx1=Lcurx1-Lprex1;
	Ldisy1=Lcury1-Lprey1;
	Rdisx1=Rcurx1-Rprex1;
	Rdisy1=Rcury1-Rprey1;
	disx1=(Ldisx1+Rdisx1)/2;
	disy1=(Ldisy1+Rdisy1)/2;
	Ld1=sqrt(Ldisx1*Ldisx1+Ldisy1*Ldisy1);
	Rd1=sqrt(Rdisx1*Rdisx1+Rdisy1*Rdisy1);

	Lprex1=-kinectData.bodyJointsArray[JointType_FootLeft].z;
	Lprey1=-kinectData.bodyJointsArray[JointType_FootLeft].x;
	Rprex1=-kinectData.bodyJointsArray[JointType_FootRight].z;
	Rprey1=-kinectData.bodyJointsArray[JointType_FootRight].x;

	if(p-r>30000)
	{
		if(m==0)
		{
			px=kinectData.bodyJointsArray[JointType_SpineBase].x;
			py=kinectData.bodyJointsArray[JointType_SpineBase].y;
			pz=kinectData.bodyJointsArray[JointType_SpineBase].z;
		}

		jointVector[SPINE_BASE_CUR_TO_PRE].x=px-kinectData.bodyJointsArray[JointType_SpineBase].x;
		jointVector[SPINE_BASE_CUR_TO_PRE].y=py-kinectData.bodyJointsArray[JointType_SpineBase].y;
		jointVector[SPINE_BASE_CUR_TO_PRE].z=pz-kinectData.bodyJointsArray[JointType_SpineBase].z;

		px=kinectData.bodyJointsArray[JointType_SpineBase].x;
		py=kinectData.bodyJointsArray[JointType_SpineBase].y;
		pz=kinectData.bodyJointsArray[JointType_SpineBase].z;
		
		if(m==0)
		{
			Lprex=0;Lprey=0;Lcurx=0;Lcury=0;Ldisx=0;Rdisx=0;disx=0;Rprex=0;Rprey=0;Rcurx=0;Rcury=0;Ldisy=0;Rdisy=0;disy=0;Ld=0;Rd=0;dist=0;
			prea=0;cura=0;theta=0;b=0;c=0;curx=0;cury=0;prex=0;prey=0;varepsilon=0;theta_VR=0;theta_TR=0;
		}
		

		Lcurx=-kinectData.bodyJointsArray[JointType_FootLeft].z;
		Lcury=-kinectData.bodyJointsArray[JointType_FootLeft].x;
		Rcurx=-kinectData.bodyJointsArray[JointType_FootRight].z;
		Rcury=-kinectData.bodyJointsArray[JointType_FootRight].x;

		curx=-kinectData.bodyJointsArray[JointType_SpineBase].z;
		cury=-kinectData.bodyJointsArray[JointType_SpineBase].x;

		if(m==0)jointVector[PRE_LOWER_TORSO]=jointVector[LOWER_TORSO];

		m=m+1;

		//varepsilon reference
		naoJointAngles[VAREPSILON_REFERENCE]=acos(getCos(jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[PRE_LOWER_TORSO]));

		//theta reference
		theta_TR=acos(getCos(jointVector[SPINE_BASE_CUR_TO_PRE],jointVector[PRE_LOWER_TORSO]));

		//theta
		if(theta_TR>PI/2)theta=acos(getCos(jointVector[SPINE_BASE_CUR_TO_PRE],jointVector[LEFT_HIP_TO_RIGHT_HIP]));
		else theta=-acos(getCos(jointVector[SPINE_BASE_CUR_TO_PRE],jointVector[LEFT_HIP_TO_RIGHT_HIP]));

		if(jointVector[HIP_REFERENCE_INIT].z<jointVector[LEFT_HIP_TO_RIGHT_HIP].z)cura=acos(getCos(jointVector[HIP_REFERENCE_INIT],jointVector[LEFT_HIP_TO_RIGHT_HIP]));
		else cura=-acos(getCos(jointVector[HIP_REFERENCE_INIT],jointVector[LEFT_HIP_TO_RIGHT_HIP]));

		Ldisx=Lcurx-Lprex;
		Ldisy=Lcury-Lprey;
		Rdisx=Rcurx-Rprex;
		Rdisy=Rcury-Rprey;
		/*disx=(Ldisx+Rdisx)/2;
		disy=(Ldisy+Rdisy)/2;*/
		disx=curx-prex;
		disy=cury-prey;
		dist=sqrt(disx*disx+disy*disy);
		if(jointVector[LEFT_HIP_TO_RIGHT_HIP].z<0)b=acos(getCos(jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[HIP_REFERENCE]));
		else b=-acos(getCos(jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[HIP_REFERENCE]));
		if(dist<0.01)c=0;
		else
		{
			if(disx>0)c=acos(-disy/dist);
			else c=-acos(-disy/dist);
		}
		Ld=sqrt(Ldisx*Ldisx+Ldisy*Ldisy);
		Rd=sqrt(Rdisx*Rdisx+Rdisy*Rdisy);
		theta_VR=naoJointAngles[VAREPSILON_REFERENCE];
		if(theta_VR<=PI/2)varepsilon=acos(getCos(jointVector[PRE_LEFT_HIP_TO_RIGHT_HIP],jointVector[LEFT_HIP_TO_RIGHT_HIP]));
		else varepsilon=-acos(getCos(jointVector[PRE_LEFT_HIP_TO_RIGHT_HIP],jointVector[LEFT_HIP_TO_RIGHT_HIP]));

		Lprex=-kinectData.bodyJointsArray[JointType_FootLeft].z;
		Lprey=-kinectData.bodyJointsArray[JointType_FootLeft].x;
		Rprex=-kinectData.bodyJointsArray[JointType_FootRight].z;
		Rprey=-kinectData.bodyJointsArray[JointType_FootRight].x;

		prex=-kinectData.bodyJointsArray[JointType_SpineBase].z;
		prey=-kinectData.bodyJointsArray[JointType_SpineBase].x;
		if(jointVector[HIP_REFERENCE_INIT].z<jointVector[LEFT_HIP_TO_RIGHT_HIP].z)prea=acos(getCos(jointVector[HIP_REFERENCE_INIT],jointVector[LEFT_HIP_TO_RIGHT_HIP]));
		else prea=-acos(getCos(jointVector[HIP_REFERENCE_INIT],jointVector[LEFT_HIP_TO_RIGHT_HIP]));

		jointVector[PRE_LEFT_HIP_TO_RIGHT_HIP]=jointVector[LEFT_HIP_TO_RIGHT_HIP];

		jointVector[PRE_LOWER_TORSO]=jointVector[LOWER_TORSO];

		
		r=p;
	}

}
void naoRobot::getVector(_kinectData kinectData)
{
	getVectorDiff(jointVector[LEFT_ANKLE_TO_KNEE],kinectData,JointType_KneeLeft,JointType_AnkleLeft);
	getVectorDiff(jointVector[LEFT_KNEE_TO_HIP],kinectData,JointType_HipLeft,JointType_KneeLeft);
	getVectorDiff(jointVector[RIGHT_KNEE_TO_HIP],kinectData,JointType_HipRight,JointType_KneeRight);
	getVectorDiff(jointVector[RIGHT_ANKLE_TO_KNEE],kinectData,JointType_KneeRight,JointType_AnkleRight);
	getVectorDiff(jointVector[LEFT_ELBOW_TO_SHOULDER],kinectData,JointType_ShoulderLeft,JointType_ElbowLeft);
	getVectorDiff(jointVector[LEFT_WRIST_TO_ELBOW],kinectData,JointType_ElbowLeft,JointType_WristLeft);
	getVectorDiff(jointVector[RIGHT_ELBOW_TO_SHOULDER],kinectData,JointType_ShoulderRight,JointType_ElbowRight);
	getVectorDiff(jointVector[RIGHT_WRIST_TO_ELBOW],kinectData,JointType_ElbowRight,JointType_WristRight);
	getVectorDiff(jointVector[SPINE_SHOULDER_TO_HEAD],kinectData,JointType_Head,JointType_SpineShoulder);

	   
	QueryPerformanceFrequency(&large_interger);  
	dff = large_interger.QuadPart;  
	QueryPerformanceCounter(&large_interger);  
	time_angle1 = large_interger.QuadPart;  

	getVectorDiff(jointVector[LEFT_SHOULDER_TO_ELBOW],kinectData,JointType_ElbowLeft,JointType_ShoulderLeft);
	getVectorDiff(jointVector[LEFT_ELBOW_TO_WRIST],kinectData,JointType_WristLeft,JointType_ElbowLeft);
	getVectorDiff(jointVector[RIGHT_SHOULDER_TO_ELBOW],kinectData,JointType_ElbowRight,JointType_ShoulderRight);
	getVectorDiff(jointVector[RIGHT_ELBOW_TO_WRIST],kinectData,JointType_WristRight,JointType_ElbowRight);
	getVectorDiff(jointVector[LEFT_HIP_TO_RIGHT_HIP],kinectData,JointType_HipRight,JointType_HipLeft);
	getVectorDiff(jointVector[LEFT_SHOULDER_TO_RIGHT_SHOULDER],kinectData,JointType_ShoulderRight,JointType_ShoulderLeft);
	getVectorDiff(jointVector[LEFT_HIP_TO_KNEE],kinectData,JointType_KneeLeft,JointType_HipLeft);
	getVectorDiff(jointVector[LEFT_KNEE_TO_ANKLE],kinectData,JointType_AnkleLeft,JointType_KneeLeft);
	getVectorDiff(jointVector[LEFT_ANKLE_TO_FOOT],kinectData,JointType_ElbowLeft,JointType_ShoulderLeft);
	getVectorDiff(jointVector[RIGHT_HIP_TO_KNEE],kinectData,JointType_KneeRight,JointType_HipRight);
	getVectorDiff(jointVector[RIGHT_KNEE_TO_ANKLE],kinectData,JointType_AnkleRight,JointType_KneeRight);
	getVectorDiff(jointVector[RIGHT_ANKLE_TO_FOOT],kinectData,JointType_FootRight,JointType_AnkleRight);
	getVectorDiff(jointVector[SPINE_BASE_TO_MID],kinectData,JointType_SpineMid,JointType_SpineBase);
	getVectorDiff(jointVector[SPINE_MID_TO_SHOULDER],kinectData,JointType_SpineShoulder,JointType_SpineMid);
	getVectorDiff(jointVector[LEFT_SHOULDER_TO_SPINE_SHOULDER],kinectData,JointType_SpineShoulder,JointType_ShoulderLeft);
	getVectorDiff(jointVector[LEFT_HIP_TO_SPINE_BASE],kinectData,JointType_SpineBase,JointType_HipLeft);
	getVectorDiff(jointVector[SPINE_MID_TO_BASE],kinectData,JointType_SpineBase,JointType_SpineMid);
	getVectorDiff(jointVector[RIGHT_SHOULDER_TO_SPINE_SHOULDER],kinectData,JointType_SpineShoulder,JointType_ShoulderRight);
	getVectorDiff(jointVector[RIGHT_HIP_TO_SPINE_BASE],kinectData,JointType_SpineBase,JointType_HipRight);
	getVectorDiff(jointVector[SPINE_BASE_TO_SHOULDER],kinectData,JointType_SpineShoulder,JointType_SpineBase);
	
	getCrossVector(jointVector[LEFT_ELBOW_TO_WRIST],jointVector[LEFT_SHOULDER_TO_ELBOW],jointVector[LEFT_NORMAL_BY_UPPER_LOWER_ARM]);
	getCrossVector(jointVector[LEFT_SHOULDER_TO_SPINE_SHOULDER],jointVector[LEFT_SHOULDER_TO_ELBOW],jointVector[LEFT_NORMAL_BY_SHOULDER_UPPER_ARM]);
	getCrossVector(jointVector[RIGHT_SHOULDER_TO_ELBOW],jointVector[RIGHT_ELBOW_TO_WRIST],jointVector[RIGHT_NORMAL_BY_UPPER_LOWER_ARM]);
	getCrossVector(jointVector[RIGHT_SHOULDER_TO_ELBOW],jointVector[RIGHT_SHOULDER_TO_SPINE_SHOULDER],jointVector[RIGHT_NORMAL_BY_SHOULDER_UPPER_ARM]);
	getCrossVector(jointVector[SPINE_BASE_TO_MID],jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[NORMAL_BY_HIP_SPINE]);
	getCrossVector(jointVector[SPINE_MID_TO_SHOULDER],jointVector[LEFT_SHOULDER_TO_SPINE_SHOULDER],jointVector[LEFT_UPPER_TORSO]);
	getCrossVector(jointVector[SPINE_MID_TO_SHOULDER],jointVector[LEFT_UPPER_TORSO],jointVector[LEFT_REFERENCE1]);
	getCrossVector(jointVector[LEFT_HIP_TO_SPINE_BASE],jointVector[SPINE_MID_TO_BASE],jointVector[LEFT_LOWER_TORSO]);
	getCrossVector(jointVector[LEFT_LOWER_TORSO],jointVector[SPINE_MID_TO_BASE],jointVector[LEFT_REFERENCE2]);
	getCrossVector(jointVector[RIGHT_SHOULDER_TO_SPINE_SHOULDER],jointVector[SPINE_MID_TO_SHOULDER],jointVector[RIGHT_UPPER_TORSO]);
	getCrossVector(jointVector[RIGHT_UPPER_TORSO],jointVector[SPINE_MID_TO_SHOULDER],jointVector[RIGHT_REFERENCE1]);
	getCrossVector(jointVector[SPINE_MID_TO_BASE],jointVector[RIGHT_HIP_TO_SPINE_BASE],jointVector[RIGHT_LOWER_TORSO]);
	getCrossVector(jointVector[SPINE_MID_TO_BASE],jointVector[RIGHT_LOWER_TORSO],jointVector[RIGHT_REFERENCE2]);
	getCrossVector(jointVector[LEFT_HIP_TO_RIGHT_HIP],jointVector[SPINE_MID_TO_BASE],jointVector[LOWER_TORSO]);

}
void naoRobot::getVectorDiff(_jointVector& jointVector, _kinectData kinectData, int joint1, int joint2)
{
	jointVector.x = kinectData.bodyJointsArray[joint1].x-kinectData.bodyJointsArray[joint2].x;
	jointVector.y = kinectData.bodyJointsArray[joint1].y-kinectData.bodyJointsArray[joint2].y;
	jointVector.z = kinectData.bodyJointsArray[joint1].z-kinectData.bodyJointsArray[joint2].z;
}
void naoRobot::getCrossVector(_jointVector jointVector1,_jointVector jointVector2,_jointVector& jointVector)
{
	float x1=jointVector1.x,
		y1=jointVector1.y,
		z1=jointVector1.z;
	float x2=jointVector2.x,
		y2=jointVector2.y,
		z2=jointVector2.z;	
	jointVector.x=y1*z2-z1*y2;
	jointVector.y=z1*x2-x1*z2;
	jointVector.z=x1*y2-y1*x2;
	float temp=sqrt(pow(jointVector.x,2)+pow(jointVector.y,2)+pow(jointVector.z,2));
	if(temp!=0)
	{
		jointVector.x=jointVector.x/temp;
		jointVector.y=jointVector.y/temp;
		jointVector.z=jointVector.z/temp;
	}
}
float naoRobot::getCos(_jointVector jointVector1,_jointVector jointVector2)
{
	float result;
	result=(jointVector1.x*jointVector2.x+jointVector1.y*jointVector2.y+jointVector1.z*jointVector2.z)/(sqrt(pow(jointVector1.x,2)+pow(jointVector1.y,2)+pow(jointVector1.z,2))*sqrt(pow(jointVector2.x,2)+pow(jointVector2.y,2)+pow(jointVector2.z,2)));
	return result;
}

float naoRobot::getCos1(_skeletonVector skeletonVector1,_skeletonVector skeletonVector2)
{
	float result1;
	result1=(skeletonVector1.x*skeletonVector2.x+skeletonVector1.y*skeletonVector2.y+skeletonVector1.z*skeletonVector2.z)/(sqrt(pow(skeletonVector1.x,2)+pow(skeletonVector1.y,2)+pow(skeletonVector1.z,2))*sqrt(pow(skeletonVector2.x,2)+pow(skeletonVector2.y,2)+pow(skeletonVector2.z,2)));
	return result1;
}
void naoRobot::evaluation()
{
	//*********************************************Evaluation1*******************************************************//
	//Absolute Frame Reference
	skeletonVector[AbsoluteFrameReference].x=1;skeletonVector[AbsoluteFrameReference].y=0;skeletonVector[AbsoluteFrameReference].z=0;
	//Human Absolute Frame
	skeletonVector[HumanDirection].x=-jointVector[NORMAL_BY_HIP_SPINE].z;skeletonVector[HumanDirection].y=-jointVector[NORMAL_BY_HIP_SPINE].x;skeletonVector[HumanDirection].z=0;
	//Rotation Angle
	if(skeletonVector[HumanDirection].y<0) RotationAngle=acos(getCos1(skeletonVector[AbsoluteFrameReference],skeletonVector[HumanDirection]));
	else RotationAngle=-acos(getCos1(skeletonVector[AbsoluteFrameReference],skeletonVector[HumanDirection]));
	//Human Skeleton Vector in Absolute Frame
	humanAbsoluteFrame(skeletonVector[HLeftTibia],jointVector[LEFT_ANKLE_TO_KNEE],RotationAngle);
	humanAbsoluteFrame(skeletonVector[HLeftThigh],jointVector[LEFT_KNEE_TO_HIP],RotationAngle);
	humanAbsoluteFrame(skeletonVector[HRightTibia],jointVector[RIGHT_ANKLE_TO_KNEE],RotationAngle);
	humanAbsoluteFrame(skeletonVector[HRightThigh],jointVector[RIGHT_KNEE_TO_HIP],RotationAngle);
	humanAbsoluteFrame(skeletonVector[HLeftUpperArm],jointVector[LEFT_ELBOW_TO_SHOULDER],RotationAngle);
	humanAbsoluteFrame(skeletonVector[HLeftLowerArm],jointVector[LEFT_WRIST_TO_ELBOW],RotationAngle);
	humanAbsoluteFrame(skeletonVector[HRightUpperArm],jointVector[RIGHT_ELBOW_TO_SHOULDER],RotationAngle);
	humanAbsoluteFrame(skeletonVector[HRightLowerArm],jointVector[RIGHT_WRIST_TO_ELBOW],RotationAngle);
	humanAbsoluteFrame(skeletonVector[HTorso],jointVector[SPINE_BASE_TO_SHOULDER],RotationAngle);
	humanAbsoluteFrame(skeletonVector[HHead],jointVector[SPINE_SHOULDER_TO_HEAD],RotationAngle);

	//Robot Skeleton Vector in Absolute Frame
	RAR=motionProxy.getAngles("RAnkleRoll",false);
	RAP=motionProxy.getAngles("RAnklePitch",false);
	RKP=motionProxy.getAngles("RKneePitch",false);
	RHP=motionProxy.getAngles("RHipPitch",false);
	RHR=motionProxy.getAngles("RHipRoll",false);
	RHYP=motionProxy.getAngles("RHipYawPitch",false);
	LHYP=motionProxy.getAngles("LHipYawPitch",false);
	LHR=motionProxy.getAngles("LHipRoll",false);
	LHP=motionProxy.getAngles("LHipPitch",false);
	LKP=motionProxy.getAngles("LKneePitch",false);
	LAP=motionProxy.getAngles("LAnklePitch",false);
	LAR=motionProxy.getAngles("LAnkleRoll",false);
	RSR=motionProxy.getAngles("RShoulderRoll",false);
	RSP=motionProxy.getAngles("RShoulderPitch",false);
	RER=motionProxy.getAngles("RElbowRoll",false);
	REY=motionProxy.getAngles("RElbowYaw",false);
	LSR=motionProxy.getAngles("LShoulderRoll",false);
	LSP=motionProxy.getAngles("LShoulderPitch",false);
	LER=motionProxy.getAngles("LElbowRoll",false);
	LEY=motionProxy.getAngles("LElbowYaw",false);
	HY=motionProxy.getAngles("HeadYaw",false);
	HP=motionProxy.getAngles("HeadPitch",false);

	if(LLCount>2)
	{
		c1=cos(-RAR[0]);s1=sin(-RAR[0]);
		c2=cos(-RAP[0]);s2=sin(-RAP[0]);
		c3=cos(-RKP[0]);s3=sin(-RKP[0]);
		c4=cos(-RHP[0]);s4=sin(-RHP[0]);
		c5=cos(-RHR[0]);s5=sin(-RHR[0]);
		c6=cos(-RHYP[0]);s6=sin(-RHYP[0]);
		c7_1=cos(RSP[0]-PI/2);s7_1=sin(RSP[0]-PI/2);
		c7_2=cos(LSP[0]-PI/2);s7_2=sin(LSP[0]-PI/2);
		c8_1=cos(RSR[0]);s8_1=sin(RSR[0]);
		c8_2=cos(LSR[0]);s8_2=sin(LSR[0]);
		c9_1=cos(-REY[0]);s9_1=sin(-REY[0]);
		c9_2=cos(-LEY[0]);s9_2=sin(-LEY[0]);
		c10_1=cos(RER[0]);s10_1=sin(RER[0]);
		c10_2=cos(LER[0]);s10_2=sin(LER[0]);
		c7=cos(-LHYP[0]);s7=sin(-LHYP[0]);
		c8=cos(LHR[0]);s8=sin(LHR[0]);
		c9=cos(LHP[0]);s9=sin(LHP[0]);
		c10=cos(LKP[0]);s10=sin(LKP[0]);
		c11=cos(HY[0]);s11=sin(HY[0]);
		c12=cos(HP[0]);s12=sin(HP[0]);
		cp1=cos(PI/4);sp1=sin(PI/4);
		cp2=cos(-PI/4);sp2=sin(-PI/4);
	}

	if(RLCount>2)
	{
		c1=cos(-LAR[0]);s1=sin(-LAR[0]);
		c2=cos(-LAP[0]);s2=sin(-LAP[0]);
		c3=cos(-LKP[0]);s3=sin(-LKP[0]);
		c4=cos(-LHP[0]);s4=sin(-LHP[0]);
		c5=cos(-LHR[0]);s5=sin(-LHR[0]);
		c6=cos(LHYP[0]);s6=sin(LHYP[0]);
		c7_1=cos(LSP[0]-PI/2);s7_1=sin(LSP[0]-PI/2);
		c7_2=cos(RSP[0]-PI/2);s7_2=sin(RSP[0]-PI/2);
		c8_1=cos(LSR[0]);s8_1=sin(LSR[0]);
		c8_2=cos(RSR[0]);s8_2=sin(RSR[0]);
		c9_1=cos(-LEY[0]);s9_1=sin(-LEY[0]);
		c9_2=cos(-REY[0]);s9_2=sin(-REY[0]);
		c10_1=cos(LER[0]);s10_1=sin(LER[0]);
		c10_2=cos(RER[0]);s10_2=sin(RER[0]);
		c7=cos(RHYP[0]);s7=sin(RHYP[0]);
		c8=cos(RHR[0]);s8=sin(RHR[0]);
		c9=cos(RHP[0]);s9=sin(RHP[0]);
		c10=cos(RKP[0]);s10=sin(RKP[0]);
		c11=cos(HY[0]);s11=sin(HY[0]);
		c12=cos(HP[0]);s12=sin(HP[0]);
		cp2=cos(PI/4);sp2=sin(PI/4);
		cp1=cos(-PI/4);sp1=sin(-PI/4);
	}

	if(LLCount<=2&&RLCount<=2)
	{
		c1=cos(-RAR[0]);s1=sin(-RAR[0]);
		c2=cos(-RAP[0]);s2=sin(-RAP[0]);
		c3=cos(-RKP[0]);s3=sin(-RKP[0]);
		c4=cos(-RHP[0]);s4=sin(-RHP[0]);
		c5=cos(-RHR[0]);s5=sin(-RHR[0]);
		c6=cos(-RHYP[0]);s6=sin(-RHYP[0]);
		c7_1=cos(RSP[0]-PI/2);s7_1=sin(RSP[0]-PI/2);
		c7_2=cos(LSP[0]-PI/2);s7_2=sin(LSP[0]-PI/2);
		c8_1=cos(RSR[0]);s8_1=sin(RSR[0]);
		c8_2=cos(LSR[0]);s8_2=sin(LSR[0]);
		c9_1=cos(-REY[0]);s9_1=sin(-REY[0]);
		c9_2=cos(-LEY[0]);s9_2=sin(-LEY[0]);
		c10_1=cos(RER[0]);s10_1=sin(RER[0]);
		c10_2=cos(LER[0]);s10_2=sin(LER[0]);
		c7=cos(-LHYP[0]);s7=sin(-LHYP[0]);
		c8=cos(LHR[0]);s8=sin(LHR[0]);
		c9=cos(LHP[0]);s9=sin(LHP[0]);
		c10=cos(LKP[0]);s10=sin(LKP[0]);
		c11=cos(HY[0]);s11=sin(HY[0]);
		c12=cos(HP[0]);s12=sin(HP[0]);
		cp1=cos(PI/4);sp1=sin(PI/4);
		cp2=cos(-PI/4);sp2=sin(-PI/4);
	}

	R1_1=0;
	R1_2=-s1;
	R1_3=c1;

	R2_1=s2;
	R2_2=-c2*s1;
	R2_3=c1*c2;

	R3_1=c2*s3 + c3*s2;
	R3_2=-s1*(c2*c3 - s2*s3);
	R3_3=c1*(c2*c3 - s2*s3);

	R4_1=c2*(c3*s4 + c4*s3) + s2*(c3*c4 - s3*s4);
	R4_2=-s1*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3));
	R4_3=c1*(c2*(c3*c4 - s3*s4) - s2*(c3*s4 + c4*s3));

	R5_1=c2*(c3*c5*s4 + c4*c5*s3) + s2*(c3*c4*c5 - c5*s3*s4);
	R5_2=- c1*s5 - s1*(c2*(c3*c4*c5 - c5*s3*s4) - s2*(c3*c5*s4 + c4*c5*s3));
	R5_3=c1*(c2*(c3*c4*c5 - c5*s3*s4) - s2*(c3*c5*s4 + c4*c5*s3)) - s1*s5;

	R611=c2*(c3*(c4*c6 + s4*(c5*s6*sp2 + cp2*s5*s6)) - s3*(c6*s4 - c4*(c5*s6*sp2 + cp2*s5*s6))) - s2*(c3*(c6*s4 - c4*(c5*s6*sp2 + cp2*s5*s6)) + s3*(c4*c6 + s4*(c5*s6*sp2 + cp2*s5*s6)));
	R612=c1*(c5*cp2*s6 - s5*s6*sp2) + s1*(c2*(c3*(c6*s4 - c4*(c5*s6*sp2 + cp2*s5*s6)) + s3*(c4*c6 + s4*(c5*s6*sp2 + cp2*s5*s6))) + s2*(c3*(c4*c6 + s4*(c5*s6*sp2 + cp2*s5*s6)) - s3*(c6*s4 - c4*(c5*s6*sp2 + cp2*s5*s6))));
	R613=s1*(c5*cp2*s6 - s5*s6*sp2) - c1*(c2*(c3*(c6*s4 - c4*(c5*s6*sp2 + cp2*s5*s6)) + s3*(c4*c6 + s4*(c5*s6*sp2 + cp2*s5*s6))) + s2*(c3*(c4*c6 + s4*(c5*s6*sp2 + cp2*s5*s6)) - s3*(c6*s4 - c4*(c5*s6*sp2 + cp2*s5*s6))));
	R621=- s2*(c3*(c4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) - cp1*s4*s6) - s3*(s4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) + c4*cp1*s6)) - c2*(c3*(s4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) + c4*cp1*s6) + s3*(c4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) - cp1*s4*s6));
	R622=- c1*(s5*(cp2*sp1 + c6*cp1*sp2) + c5*(sp1*sp2 - c6*cp1*cp2)) - s1*(s2*(c3*(s4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) + c4*cp1*s6) + s3*(c4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) - cp1*s4*s6)) - c2*(c3*(c4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) - cp1*s4*s6) - s3*(s4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) + c4*cp1*s6)));
	R623=c1*(s2*(c3*(s4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) + c4*cp1*s6) + s3*(c4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) - cp1*s4*s6)) - c2*(c3*(c4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) - cp1*s4*s6) - s3*(s4*(s5*(sp1*sp2 - c6*cp1*cp2) - c5*(cp2*sp1 + c6*cp1*sp2)) + c4*cp1*s6))) - s1*(s5*(cp2*sp1 + c6*cp1*sp2) + c5*(sp1*sp2 - c6*cp1*cp2));
	R631=- c2*(c3*(s4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) - c4*s6*sp1) + s3*(c4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) + s4*s6*sp1)) - s2*(c3*(c4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) + s4*s6*sp1) - s3*(s4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) - c4*s6*sp1));
	R632=s1*(c2*(c3*(c4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) + s4*s6*sp1) - s3*(s4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) - c4*s6*sp1)) - s2*(c3*(s4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) - c4*s6*sp1) + s3*(c4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) + s4*s6*sp1))) - c1*(s5*(cp1*cp2 - c6*sp1*sp2) + c5*(cp1*sp2 + c6*cp2*sp1));
	R633=- s1*(s5*(cp1*cp2 - c6*sp1*sp2) + c5*(cp1*sp2 + c6*cp2*sp1)) - c1*(c2*(c3*(c4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) + s4*s6*sp1) - s3*(s4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) - c4*s6*sp1)) - s2*(c3*(s4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) - c4*s6*sp1) + s3*(c4*(s5*(cp1*sp2 + c6*cp2*sp1) - c5*(cp1*cp2 - c6*sp1*sp2)) + s4*s6*sp1)));

	R6_1=R631;
	R6_2=R632;
	R6_3=R633;

	R7_1_1=R631*c7_1 + R611*s7_1;
	R7_1_2=R632*c7_1 + R612*s7_1;
	R7_1_3=R633*c7_1 + R613*s7_1;

	R7_2_1=R631*c7_2 + R611*s7_2;
	R7_2_2=R632*c7_2 + R612*s7_2;
	R7_2_3=R633*c7_2 + R613*s7_2;

	R8_1_1=c8_1*(R631*c7_1 + R611*s7_1) - R621*s8_1;
	R8_1_2=c8_1*(R632*c7_1 + R612*s7_1) - R622*s8_1;
	R8_1_3=c8_1*(R633*c7_1 + R613*s7_1) - R623*s8_1;

	R8_2_1=c8_2*(R631*c7_2 + R611*s7_2) - R621*s8_2;
	R8_2_2=c8_2*(R632*c7_2 + R612*s7_2) - R622*s8_2;
	R8_2_3=c8_2*(R633*c7_2 + R613*s7_2) - R623*s8_2;

	R9_1_1=c8_1*(R631*c7_1 + R611*s7_1) - R621*s8_1;
	R9_1_2=c8_1*(R632*c7_1 + R612*s7_1) - R622*s8_1;
	R9_1_3=c8_1*(R633*c7_1 + R613*s7_1) - R623*s8_1;

	R9_2_1=c8_2*(R631*c7_2 + R611*s7_2) - R621*s8_2;
	R9_2_2=c8_2*(R632*c7_2 + R612*s7_2) - R622*s8_2;
	R9_2_3=c8_2*(R633*c7_2 + R613*s7_2) - R623*s8_2;

	R10_1_1=s10_1*(s9_1*(R611*c7_1 - R631*s7_1) - c9_1*(R621*c8_1 + s8_1*(R631*c7_1 + R611*s7_1))) - c10_1*(R621*s8_1 - c8_1*(R631*c7_1 + R611*s7_1));
	R10_1_2=s10_1*(s9_1*(R612*c7_1 - R632*s7_1) - c9_1*(R622*c8_1 + s8_1*(R632*c7_1 + R612*s7_1))) - c10_1*(R622*s8_1 - c8_1*(R632*c7_1 + R612*s7_1));
	R10_1_3=s10_1*(s9_1*(R613*c7_1 - R633*s7_1) - c9_1*(R623*c8_1 + s8_1*(R633*c7_1 + R613*s7_1))) - c10_1*(R623*s8_1 - c8_1*(R633*c7_1 + R613*s7_1));

	R10_2_1=s10_2*(s9_2*(R611*c7_2 - R631*s7_2) - c9_2*(R621*c8_2 + s8_2*(R631*c7_2 + R611*s7_2))) - c10_2*(R621*s8_2 - c8_2*(R631*c7_2 + R611*s7_2));
	R10_2_2=s10_2*(s9_2*(R612*c7_2 - R632*s7_2) - c9_2*(R622*c8_2 + s8_2*(R632*c7_2 + R612*s7_2))) - c10_2*(R622*s8_2 - c8_2*(R632*c7_2 + R612*s7_2));
	R10_2_3=s10_2*(s9_2*(R613*c7_2 - R633*s7_2) - c9_2*(R623*c8_2 + s8_2*(R633*c7_2 + R613*s7_2))) - c10_2*(R623*s8_2 - c8_2*(R633*c7_2 + R613*s7_2));

	R7_1= R631*(cp1*cp2 - c7*sp1*sp2) - R621*(cp2*sp1 + c7*cp1*sp2) + R611*s7*sp2;
	R7_2=R632*(cp1*cp2 - c7*sp1*sp2) - R622*(cp2*sp1 + c7*cp1*sp2) + R612*s7*sp2;
	R7_3=R633*(cp1*cp2 - c7*sp1*sp2) - R623*(cp2*sp1 + c7*cp1*sp2) + R613*s7*sp2;

	R8_1=R621*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R631*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R611*(c8*s7*sp2 + cp2*s7*s8);
	R8_2=R622*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R632*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R612*(c8*s7*sp2 + cp2*s7*s8);
	R8_3=R623*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R633*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R613*(c8*s7*sp2 + cp2*s7*s8);

	R9_1=s9*(R611*c7 + R621*cp1*s7 + R631*s7*sp1) + c9*(R621*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R631*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R611*(c8*s7*sp2 + cp2*s7*s8));
	R9_2=s9*(R612*c7 + R622*cp1*s7 + R632*s7*sp1) + c9*(R622*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R632*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R612*(c8*s7*sp2 + cp2*s7*s8));
	R9_3=s9*(R613*c7 + R623*cp1*s7 + R633*s7*sp1) + c9*(R623*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R633*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R613*(c8*s7*sp2 + cp2*s7*s8));

	R10_1=c10*(s9*(R611*c7 + R621*cp1*s7 + R631*s7*sp1) + c9*(R621*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R631*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R611*(c8*s7*sp2 + cp2*s7*s8))) + s10*(c9*(R611*c7 + R621*cp1*s7 + R631*s7*sp1) - s9*(R621*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R631*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R611*(c8*s7*sp2 + cp2*s7*s8)));
	R10_2=c10*(s9*(R612*c7 + R622*cp1*s7 + R632*s7*sp1) + c9*(R622*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R632*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R612*(c8*s7*sp2 + cp2*s7*s8))) + s10*(c9*(R612*c7 + R622*cp1*s7 + R632*s7*sp1) - s9*(R622*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R632*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R612*(c8*s7*sp2 + cp2*s7*s8)));
	R10_3=c10*(s9*(R613*c7 + R623*cp1*s7 + R633*s7*sp1) + c9*(R623*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R633*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R613*(c8*s7*sp2 + cp2*s7*s8))) + s10*(c9*(R613*c7 + R623*cp1*s7 + R633*s7*sp1) - s9*(R623*(s8*(sp1*sp2 - c7*cp1*cp2) - c8*(cp2*sp1 + c7*cp1*sp2)) - R633*(s8*(cp1*sp2 + c7*cp2*sp1) - c8*(cp1*cp2 - c7*sp1*sp2)) + R613*(c8*s7*sp2 + cp2*s7*s8)));

	R11_1=R631;
	R11_2=R632;
	R11_3=R633;

	R12_1=R631*c12 + R611*c11*s12 + R621*s11*s12;
	R12_2=R632*c12 + R612*c11*s12 + R622*s11*s12;
	R12_3=R633*c12 + R613*c11*s12 + R623*s11*s12;

	if(LLCount>2)
	{
		skeletonVector[RRightTibia].x=R2_1;skeletonVector[RRightTibia].y=R2_2;skeletonVector[RRightTibia].z=R2_3;
		skeletonVector[RRightThigh].x=R3_1;skeletonVector[RRightThigh].y=R3_2;skeletonVector[RRightThigh].z=R3_3;
		skeletonVector[RTorso].x=R6_1;skeletonVector[RTorso].y=R6_2;skeletonVector[RTorso].z=R6_3;
		skeletonVector[RLeftThigh].x=R9_1;skeletonVector[RLeftThigh].y=R9_2;skeletonVector[RLeftThigh].z=R9_3;
		skeletonVector[RLeftTibia].x=R10_1;skeletonVector[RLeftTibia].y=R10_2;skeletonVector[RLeftTibia].z=R10_3;
		skeletonVector[RRightUpperArm].x=R8_1_1;skeletonVector[RRightUpperArm].y=R8_1_2;skeletonVector[RRightUpperArm].z=R8_1_3;
		skeletonVector[RRightLowerArm].x=R10_1_1;skeletonVector[RRightLowerArm].y=R10_1_2;skeletonVector[RRightLowerArm].z=R10_1_3;
		skeletonVector[RLeftUpperArm].x=R8_2_1;skeletonVector[RLeftUpperArm].y=R8_2_2;skeletonVector[RLeftUpperArm].z=R8_2_3;
		skeletonVector[RLeftLowerArm].x=R10_2_1;skeletonVector[RLeftLowerArm].y=R10_2_2;skeletonVector[RLeftLowerArm].z=R10_2_3;
		skeletonVector[RHead].x=R12_1;skeletonVector[RHead].y=R12_2;skeletonVector[RHead].z=R12_3;
	}

	if(RLCount>2)
	{
		skeletonVector[RLeftTibia].x=R2_1;skeletonVector[RLeftTibia].y=R2_2;skeletonVector[RLeftTibia].z=R2_3;
		skeletonVector[RLeftThigh].x=R3_1;skeletonVector[RLeftThigh].y=R3_2;skeletonVector[RLeftThigh].z=R3_3;
		skeletonVector[RTorso].x=R6_1;skeletonVector[RTorso].y=R6_2;skeletonVector[RTorso].z=R6_3;
		skeletonVector[RRightThigh].x=R9_1;skeletonVector[RRightThigh].y=R9_2;skeletonVector[RRightThigh].z=R9_3;
		skeletonVector[RRightTibia].x=R10_1;skeletonVector[RRightTibia].y=R10_2;skeletonVector[RRightTibia].z=R10_3;
		skeletonVector[RLeftUpperArm].x=R8_1_1;skeletonVector[RLeftUpperArm].y=R8_1_2;skeletonVector[RLeftUpperArm].z=R8_1_3;
		skeletonVector[RLeftLowerArm].x=R10_1_1;skeletonVector[RLeftLowerArm].y=R10_1_2;skeletonVector[RLeftLowerArm].z=R10_1_3;
		skeletonVector[RRightUpperArm].x=R8_2_1;skeletonVector[RRightUpperArm].y=R8_2_2;skeletonVector[RRightUpperArm].z=R8_2_3;
		skeletonVector[RRightLowerArm].x=R10_2_1;skeletonVector[RRightLowerArm].y=R10_2_2;skeletonVector[RRightLowerArm].z=R10_2_3;
		skeletonVector[RHead].x=R12_1;skeletonVector[RHead].y=R12_2;skeletonVector[RHead].z=R12_3;
	}

	if(LLCount<=2&&RLCount<=2)
	{
		skeletonVector[RRightTibia].x=R2_1;skeletonVector[RRightTibia].y=R2_2;skeletonVector[RRightTibia].z=R2_3;
		skeletonVector[RRightThigh].x=R3_1;skeletonVector[RRightThigh].y=R3_2;skeletonVector[RRightThigh].z=R3_3;
		skeletonVector[RTorso].x=R6_1;skeletonVector[RTorso].y=R6_2;skeletonVector[RTorso].z=R6_3;
		skeletonVector[RLeftThigh].x=R9_1;skeletonVector[RLeftThigh].y=R9_2;skeletonVector[RLeftThigh].z=R9_3;
		skeletonVector[RLeftTibia].x=R10_1;skeletonVector[RLeftTibia].y=R10_2;skeletonVector[RLeftTibia].z=R10_3;
		skeletonVector[RRightUpperArm].x=R8_1_1;skeletonVector[RRightUpperArm].y=R8_1_2;skeletonVector[RRightUpperArm].z=R8_1_3;
		skeletonVector[RRightLowerArm].x=R10_1_1;skeletonVector[RRightLowerArm].y=R10_1_2;skeletonVector[RRightLowerArm].z=R10_1_3;
		skeletonVector[RLeftUpperArm].x=R8_2_1;skeletonVector[RLeftUpperArm].y=R8_2_2;skeletonVector[RLeftUpperArm].z=R8_2_3;
		skeletonVector[RLeftLowerArm].x=R10_2_1;skeletonVector[RLeftLowerArm].y=R10_2_2;skeletonVector[RLeftLowerArm].z=R10_2_3;
		skeletonVector[RHead].x=R12_1;skeletonVector[RHead].y=R12_2;skeletonVector[RHead].z=R12_3;
	}

	//Evaluation1
	r1=getCos1(skeletonVector[HLeftTibia],skeletonVector[RLeftTibia]);
	r2=getCos1(skeletonVector[HLeftThigh],skeletonVector[RLeftThigh]);
	r3=getCos1(skeletonVector[HTorso],skeletonVector[RTorso]);
	r4=getCos1(skeletonVector[HRightThigh],skeletonVector[RRightThigh]);
	r5=getCos1(skeletonVector[HRightTibia],skeletonVector[RRightTibia]);
	r6=getCos1(skeletonVector[HLeftUpperArm],skeletonVector[RLeftUpperArm]);
	r7=getCos1(skeletonVector[HLeftLowerArm],skeletonVector[RLeftLowerArm]);
	r8=getCos1(skeletonVector[HRightUpperArm],skeletonVector[RRightUpperArm]);
	r9=getCos1(skeletonVector[HRightLowerArm],skeletonVector[RRightLowerArm]);
	r10=getCos1(skeletonVector[HHead],skeletonVector[RHead]);
	evaluation1=(r1+r2+r3+r4+r5+r6+r7+r8+r9+r10)/10;
	//cout<<skeletonVector[HLeftLowerArm].x<<","<<skeletonVector[HLeftLowerArm].y<<","<<skeletonVector[HLeftLowerArm].z<<","<<skeletonVector[RLeftLowerArm].x<<","<<skeletonVector[RLeftLowerArm].y<<","<<skeletonVector[RLeftLowerArm].z<<endl;
	//cout<<r1<<","<<r2<<","<<r3<<","<<r4<<","<<r5<<","<<r6<<","<<r7<<","<<r8<<","<<r9<<","<<r10<<","<<evaluation1<<endl;

	//***************************************************Evaluation2******************************************************//

	//Torso Reference
	skeletonVector[_HTorso].x=skeletonVector[HTorso].x;skeletonVector[_HTorso].y=skeletonVector[HTorso].y;skeletonVector[_HTorso].z=skeletonVector[HTorso].z;
	skeletonVector[_RTorso].x=skeletonVector[RTorso].x;skeletonVector[_RTorso].y=skeletonVector[RTorso].y;skeletonVector[_RTorso].z=skeletonVector[RTorso].z;
	//Human Relative Vector
	c1_1_1=cos(naoJointAngles[NAO_RIGHT_HIP_ROLL]);c1_1_2=cos(naoJointAngles[NAO_RIGHT_HIP_PITCH]);
	c1_2_1=cos(naoJointAngles[NAO_RIGHT_KNEE_PITCH]);
	c2_1_1=cos(naoJointAngles[NAO_LEFT_HIP_ROLL]);c2_1_2=cos(naoJointAngles[NAO_LEFT_HIP_PITCH]);
	c2_2_1=cos(naoJointAngles[NAO_LEFT_KNEE_PITCH]);
	c3_1_1=cos(naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]-PI/2);c3_1_2=cos(naoJointAngles[NAO_RIGHT_SHOULDER_ROLL]);
	c3_2_1=cos(-naoJointAngles[NAO_RIGHT_ELBOW_YAW]);c3_2_2=cos(naoJointAngles[NAO_RIGHT_ELBOW_ROLL]);
	c4_1_1=cos(naoJointAngles[NAO_LEFT_SHOULDER_PITCH]-PI/2);c4_1_2=cos(naoJointAngles[NAO_LEFT_SHOULDER_ROLL]);
	c4_2_1=cos(-naoJointAngles[NAO_LEFT_ELBOW_YAW]);c4_2_2=cos(naoJointAngles[NAO_LEFT_ELBOW_ROLL]);
	c5_1=cos(naoJointAngles[NAO_HEAD_YAW]);c5_2=cos(naoJointAngles[NAO_HEAD_PITCH]);
	s1_1_1=sin(naoJointAngles[NAO_RIGHT_HIP_ROLL]);s1_1_2=sin(naoJointAngles[NAO_RIGHT_HIP_PITCH]);
	s1_2_1=sin(naoJointAngles[NAO_RIGHT_KNEE_PITCH]);
	s2_1_1=sin(naoJointAngles[NAO_LEFT_HIP_ROLL]);s2_1_2=sin(naoJointAngles[NAO_LEFT_HIP_PITCH]);
	s2_2_1=sin(naoJointAngles[NAO_LEFT_KNEE_PITCH]);
	s3_1_1=sin(naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]-PI/2);s3_1_2=sin(naoJointAngles[NAO_RIGHT_SHOULDER_ROLL]);
	s3_2_1=sin(-naoJointAngles[NAO_RIGHT_ELBOW_YAW]);s3_2_2=sin(naoJointAngles[NAO_RIGHT_ELBOW_ROLL]);
	s4_1_1=sin(naoJointAngles[NAO_LEFT_SHOULDER_PITCH]-PI/2);s4_1_2=sin(naoJointAngles[NAO_LEFT_SHOULDER_ROLL]);
	s4_2_1=sin(-naoJointAngles[NAO_LEFT_ELBOW_YAW]);s4_2_2=sin(naoJointAngles[NAO_LEFT_ELBOW_ROLL]);
	s5_1=sin(naoJointAngles[NAO_HEAD_YAW]);s5_2=sin(naoJointAngles[NAO_HEAD_PITCH]);

	V2_1=0;V2_2=-s1_1_1;V2_3=c1_1_1;
	V3_1=s1_1_2;V3_2=-c1_1_2*s1_1_1;V3_3=c1_1_1*c1_1_2;
	V4_1=s1_2_1;V4_2=0;V4_3=c1_2_1;

	V6_1=0;V6_2=-s2_1_1;V6_3=c2_1_1;
	V7_1=s2_1_2;V7_2=-c2_1_2*s2_1_1;V7_3=c2_1_1*c2_1_2;
	V8_1=s2_2_1;V8_2=0;V8_3=c2_2_1;

	V9_1=s3_1_1;V9_2=0;V9_3=c3_1_1;
	V10_1=c3_1_2*s3_1_1;V10_2=-s3_1_2;V10_3=c3_1_1*c3_1_2;

	V11_1=0;V11_2=0;V11_3=1;
	V12_1=s3_2_1*s3_2_2;V12_2=-c3_2_1*s3_2_2;V12_3=c3_2_2;

	V13_1=s4_1_1;V13_2=0;V13_3=c4_1_1;
	V14_1=c4_1_2*s4_1_1;V14_2=-s4_1_2;V14_3=c4_1_1*c4_1_2;

	V15_1=0;V15_2=0;V15_3=1;
	V16_1=s4_2_1*s4_2_2;V16_2=-c4_2_1*s4_2_2;V16_3=c4_2_2;

	V17_1=0;V17_2=0;V17_3=0;
	V18_1=c5_1*s5_2;V18_2=s5_1*s5_2;V18_3=c5_2;

	skeletonVector[_HRightThigh].x=V3_1;skeletonVector[_HRightThigh].y=V3_2;skeletonVector[_HRightThigh].z=V3_3;
	skeletonVector[_HRightTibia].x=V4_1;skeletonVector[_HRightTibia].y=V4_2;skeletonVector[_HRightTibia].z=V4_3;
	skeletonVector[_HLeftThigh].x=V7_1;skeletonVector[_HLeftThigh].y=V7_2;skeletonVector[_HLeftThigh].z=V7_3;
	skeletonVector[_HLeftTibia].x=V8_1;skeletonVector[_HLeftTibia].y=V8_2;skeletonVector[_HLeftTibia].z=V8_3;
	skeletonVector[_HRightUpperArm].x=V10_1;skeletonVector[_HRightUpperArm].y=V10_2;skeletonVector[_HRightUpperArm].z=V10_3;
	skeletonVector[_HRightLowerArm].x=V12_1;skeletonVector[_HRightLowerArm].y=V12_2;skeletonVector[_HRightLowerArm].z=V12_3;
	skeletonVector[_HLeftUpperArm].x=V14_1;skeletonVector[_HLeftUpperArm].y=V14_2;skeletonVector[_HLeftUpperArm].z=V14_3;
	skeletonVector[_HLeftLowerArm].x=V16_1;skeletonVector[_HLeftLowerArm].y=V16_2;skeletonVector[_HLeftLowerArm].z=V16_3;
	skeletonVector[_HHead].x=V18_1;skeletonVector[_HHead].y=V18_2;skeletonVector[_HHead].z=V18_3;

	//Robot Relative Vector
	c1_1_1=cos(RHYP[0]);c1_1_2=cos(RHR[0]);c1_1_3=cos(RHP[0]);
	c1_2_1=cos(RKP[0]);
	c2_1_1=cos(-LHYP[0]);c2_1_2=cos(LHR[0]);c2_1_3=cos(LHP[0]);
	c2_2_1=cos(LKP[0]);
	c3_1_1=cos(RSP[0]-PI/2);c3_1_2=cos(RSR[0]);
	c3_2_1=cos(-REY[0]);c3_2_2=cos(RER[0]);
	c4_1_1=cos(LSP[0]-PI/2);c4_1_2=cos(LSR[0]);
	c4_2_1=cos(-LEY[0]);c4_2_2=cos(LER[0]);
	c5_1=cos(HY[0]);c5_2=cos(HP[0]);

	s1_1_1=sin(RHYP[0]);s1_1_2=sin(RHR[0]);s1_1_3=sin(RHP[0]);
	s1_2_1=sin(RKP[0]);
	s2_1_1=sin(-LHYP[0]);s2_1_2=sin(LHR[0]);s2_1_3=sin(LHP[0]);
	s2_2_1=sin(LKP[0]);
	s3_1_1=sin(RSP[0]-PI/2);s3_1_2=sin(RSR[0]);
	s3_2_1=sin(-REY[0]);s3_2_2=sin(RER[0]);
	s4_1_1=sin(LSP[0]-PI/2);s4_1_2=sin(LSR[0]);
	s4_2_1=sin(-LEY[0]);s4_2_2=sin(LER[0]);
	s5_1=sin(HY[0]);s5_2=sin(HP[0]);

	cp1=cos(PI/4);sp1=sin(PI/4);
	cp2=cos(-PI/4);sp2=sin(-PI/4);

	V3_1=c1_1_1*s1_1_3 + c1_1_2*c1_1_3*s1_1_1*sp1 + c1_1_3*cp1*s1_1_1*s1_1_2;
	V3_2=c1_1_3*s1_1_2*(sp1*sp2 - c1_1_1*cp1*cp2) - c1_1_2*c1_1_3*(cp1*sp2 + c1_1_1*cp2*sp1) + cp2*s1_1_1*s1_1_3;
	V3_3=c1_1_2*c1_1_3*(cp1*cp2 - c1_1_1*sp1*sp2) - c1_1_3*s1_1_2*(cp2*sp1 + c1_1_1*cp1*sp2) + s1_1_1*s1_1_3*sp2;
	V4_1=s1_2_1;V4_2=0;V4_3=c1_2_1;

	V7_1=c2_1_1*s2_1_3 + c2_1_2*c2_1_3*s2_1_1*sp2 + c2_1_3*cp2*s2_1_1*s2_1_2;
	V7_2=c2_1_3*s2_1_2*(sp1*sp2 - c2_1_1*cp1*cp2) - c2_1_2*c2_1_3*(cp2*sp1 + c2_1_1*cp1*sp2) + cp1*s2_1_1*s2_1_3;
	V7_3=c2_1_2*c2_1_3*(cp1*cp2 - c2_1_1*sp1*sp2) - c2_1_3*s2_1_2*(cp1*sp2 + c2_1_1*cp2*sp1) + s2_1_1*s2_1_3*sp1;
	V8_1=s2_2_1;V8_2=0;V8_3=c2_2_1;

	V9_1=s3_1_1;V9_2=0;V9_3=c3_1_1;
	V10_1=c3_1_2*s3_1_1;V10_2=-s3_1_2;V10_3=c3_1_1*c3_1_2;

	V11_1=0;V11_2=0;V11_3=1;
	V12_1=s3_2_1*s3_2_2;V12_2=-c3_2_1*s3_2_2;V12_3=c3_2_2;

	V13_1=s4_1_1;V13_2=0;V13_3=c4_1_1;
	V14_1=c4_1_2*s4_1_1;V14_2=-s4_1_2;V14_3=c4_1_1*c4_1_2;

	V15_1=0;V15_2=0;V15_3=1;
	V16_1=s4_2_1*s4_2_2;V16_2=-c4_2_1*s4_2_2;V16_3=c4_2_2;

	V17_1=0;V17_2=0;V17_3=0;
	V18_1=c5_1*s5_2;V18_2=s5_1*s5_2;V18_3=c5_2;

	skeletonVector[_RRightThigh].x=V3_1;skeletonVector[_RRightThigh].y=V3_2;skeletonVector[_RRightThigh].z=V3_3;
	skeletonVector[_RRightTibia].x=V4_1;skeletonVector[_RRightTibia].y=V4_2;skeletonVector[_RRightTibia].z=V4_3;
	skeletonVector[_RLeftThigh].x=V7_1;skeletonVector[_RLeftThigh].y=V7_2;skeletonVector[_RLeftThigh].z=V7_3;
	skeletonVector[_RLeftTibia].x=V8_1;skeletonVector[_RLeftTibia].y=V8_2;skeletonVector[_RLeftTibia].z=V8_3;
	skeletonVector[_RRightUpperArm].x=V10_1;skeletonVector[_RRightUpperArm].y=V10_2;skeletonVector[_RRightUpperArm].z=V10_3;
	skeletonVector[_RRightLowerArm].x=V12_1;skeletonVector[_RRightLowerArm].y=V12_2;skeletonVector[_RRightLowerArm].z=V12_3;
	skeletonVector[_RLeftUpperArm].x=V14_1;skeletonVector[_RLeftUpperArm].y=V14_2;skeletonVector[_RLeftUpperArm].z=V14_3;
	skeletonVector[_RLeftLowerArm].x=V16_1;skeletonVector[_RLeftLowerArm].y=V16_2;skeletonVector[_RLeftLowerArm].z=V16_3;
	skeletonVector[_RHead].x=V18_1;skeletonVector[_RHead].y=V18_2;skeletonVector[_RHead].z=V18_3;

	//Evaluation2
	_r1=getCos1(skeletonVector[_HLeftTibia],skeletonVector[_RLeftTibia]);
	_r2=getCos1(skeletonVector[_HLeftThigh],skeletonVector[_RLeftThigh]);
	_r3=getCos1(skeletonVector[_HTorso],skeletonVector[RTorso]);
	_r4=getCos1(skeletonVector[_HRightThigh],skeletonVector[_RRightThigh]);
	_r5=getCos1(skeletonVector[_HRightTibia],skeletonVector[_RRightTibia]);
	_r6=getCos1(skeletonVector[_HLeftUpperArm],skeletonVector[_RLeftUpperArm]);
	_r7=getCos1(skeletonVector[_HLeftLowerArm],skeletonVector[_RLeftLowerArm]);
	_r8=getCos1(skeletonVector[_HRightUpperArm],skeletonVector[_RRightUpperArm]);
	_r9=getCos1(skeletonVector[_HRightLowerArm],skeletonVector[_RRightLowerArm]);
	_r10=getCos1(skeletonVector[_HHead],skeletonVector[_RHead]);
	_evaluation2=_r3*((_r2+_r4+_r6+_r8)/4)*((_r1+_r5+_r7+_r9)/4)*0.8+_r3*_r10*0.2;
	evaluation2=(_r1+_r2+_r3+_r4+_r5+_r6+_r7+_r8+_r9+_r10)/10;
	//cout<<_r1<<","<<_r2<<","<<_r3<<","<<_r4<<","<<_r5<<","<<_r6<<","<<_r7<<","<<_r8<<","<<_r9<<","<<_r10<<","<<_evaluation2<<evaluation2<<endl;
	//cout<<time_random<<","<<time_angle<<","<<time_total<<endl;

	outfile<<naoJointAngles[NAO_HEAD_YAW]<<","<<naoJointAngles[NAO_HEAD_PITCH]<<","<<naoJointAngles[NAO_LEFT_SHOULDER_PITCH]<<","<<naoJointAngles[NAO_LEFT_SHOULDER_ROLL]
	<<","<<naoJointAngles[NAO_LEFT_ELBOW_YAW]<<","<<naoJointAngles[NAO_LEFT_ELBOW_ROLL]<<","<<naoJointAngles[NAO_RIGHT_SHOULDER_PITCH]<<","<<naoJointAngles[NAO_RIGHT_SHOULDER_ROLL]
	<<","<<naoJointAngles[NAO_RIGHT_ELBOW_YAW]<<","<<naoJointAngles[NAO_RIGHT_ELBOW_ROLL]<<","<<naoJointAngles[NAO_LEFT_HIP_ROLL]<<","<<naoJointAngles[NAO_LEFT_HIP_PITCH]<<","
		<<naoJointAngles[NAO_LEFT_KNEE_PITCH]<<","<<naoJointAngles[NAO_LEFT_ANKLE_PITCH]<<","<<naoJointAngles[NAO_LEFT_ANKLE_ROLL]<<","<<naoJointAngles[NAO_RIGHT_HIP_ROLL]<<","
		<<naoJointAngles[NAO_RIGHT_HIP_PITCH]<<","<<naoJointAngles[NAO_RIGHT_KNEE_PITCH]<<","<<naoJointAngles[NAO_RIGHT_ANKLE_PITCH]<<","<<naoJointAngles[NAO_RIGHT_ANKLE_ROLL]<<endl;
	outfile<<HY[0]<<","<<HP[0]<<","<<LSP[0]<<","<<LSR[0]<<","<<LEY[0]<<","<<LER[0]<<","<<RSP[0]<<","<<RSR[0]<<","<<REY[0]<<","<<RER[0]<<","<<LHR[0]<<","<<LHP[0]<<","<<LKP[0]<<","<<
		LAP[0]<<","<<LAR[0]<<","<<RHR[0]<<","<<RHP[0]<<","<<RKP[0]<<","<<RAP[0]<<","<<RAR[0]<<endl;
	outfile<<r1<<","<<r2<<","<<r3<<","<<r4<<","<<r5<<","<<r6<<","<<r7<<","<<r8<<","<<r9<<","<<r10<<","<<evaluation1<<endl;
	outfile<<_r1<<","<<_r2<<","<<_r3<<","<<_r4<<","<<_r5<<","<<_r6<<","<<_r7<<","<<_r8<<","<<_r9<<","<<_r10<<","<<_evaluation2<<","<<evaluation2<<endl;
	outfile<<time_random<<","<<time_angle<<","<<time_total<<endl;

}

void naoRobot::humanAbsoluteFrame(_skeletonVector& skeletonVector, _jointVector jointVector, float RotationAngle)
{
	skeletonVector.x=-jointVector.z*cos(RotationAngle)+jointVector.x*sin(RotationAngle);
	skeletonVector.y=-jointVector.z*sin(RotationAngle)-jointVector.x*cos(RotationAngle);
	skeletonVector.z=jointVector.y;
}
