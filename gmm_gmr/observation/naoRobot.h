#pragma  once
#include <iostream>
#include <fstream>
#include <math.h>
#include <alcommon/almodule.h>
#include <alerror/alerror.h>
#include <alcommon/alproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include "Kinect.h"
#include <stdio.h>  
#include <windows.h>  
#include <time.h> //time_t time()  clock_t clock()  
#include <Mmsystem.h>             //timeGetTime()  
#pragma comment(lib, "Winmm.lib")   //timeGetTime()  

using namespace std;
enum _AngleType
{
	NAO_HEAD_PITCH = 0,
	NAO_HEAD_YAW = 1,
	NAO_LEFT_SHOULDER_PITCH = 2,
	NAO_LEFT_SHOULDER_ROLL = 3,
	NAO_LEFT_ELBOW_YAW = 4,
	NAO_LEFT_ELBOW_ROLL = 5,
	NAO_LEFT_WRIST_YAW = 6,
	NAO_RIGHT_SHOULDER_PITCH = 7,
	NAO_RIGHT_SHOULDER_ROLL	= 8,
	NAO_RIGHT_ELBOW_YAW = 9,
	NAO_RIGHT_ELBOW_ROLL = 10,
	NAO_RIGHT_WRIST_YAW = 11,
	NAO_LEFT_HIP_ROLL = 12,
	NAO_LEFT_HIP_PITCH = 13,
	NAO_LEFT_KNEE_PITCH = 14,
	NAO_LEFT_ANKLE_PITCH = 15,
	NAO_LEFT_ANKLE_ROLL = 16,
	NAO_RIGHT_HIP_ROLL = 17,
	NAO_RIGHT_HIP_PITCH = 18,
	NAO_RIGHT_KNEE_PITCH = 19,
	NAO_RIGHT_ANKLE_PITCH = 20,
	NAO_RIGHT_ANKLE_ROLL = 21,
	NAO_LEFT_ALPHA = 22,
	NAO_LEFT_BETA = 23,
	NAO_LEFT_REFERENCE1 = 24,
	NAO_LEFT_REFERENCE2 = 25,
	NAO_LEFT_TWO_PLANES = 26,
	NAO_LEFT_GAMMA = 27,
	NAO_LEFT_DELTA = 28,
	NAO_RIGHT_ALPHA = 29,
	NAO_RIGHT_BETA = 30,
	NAO_RIGHT_REFERENCE1 = 31,
	NAO_RIGHT_REFERENCE2 = 32,
	NAO_RIGHT_TWO_PLANES = 33,
	NAO_RIGHT_GAMMA = 34,
	NAO_RIGHT_DELTA = 35,
	VAREPSILON_REFERENCE = 36,
	SHOULDER_SPINE1=37,
	SHOULDER_SPINE2=38,
	AngleType_Count	= (VAREPSILON_REFERENCE+1) 
};
enum _VectorType
{
	LEFT_SHOULDER_TO_ELBOW = 0,
	LEFT_ELBOW_TO_WRIST = 1,
	RIGHT_SHOULDER_TO_ELBOW = 2,
	RIGHT_ELBOW_TO_WRIST = 3, 
	LEFT_HIP_TO_RIGHT_HIP = 4,
	LEFT_SHOULDER_TO_RIGHT_SHOULDER = 5,
	LEFT_NORMAL_BY_UPPER_LOWER_ARM = 6,
	LEFT_NORMAL_BY_SHOULDER_UPPER_ARM = 7,
	RIGHT_NORMAL_BY_UPPER_LOWER_ARM = 8,
	RIGHT_NORMAL_BY_SHOULDER_UPPER_ARM = 9,
	LEFT_HIP_TO_KNEE = 10,
	LEFT_KNEE_TO_ANKLE = 11,
	LEFT_ANKLE_TO_FOOT = 12,
	RIGHT_HIP_TO_KNEE = 13,
	RIGHT_KNEE_TO_ANKLE = 14,
	RIGHT_ANKLE_TO_FOOT = 15,
	SPINE_BASE_TO_MID = 16,
	NORMAL_BY_HIP_SPINE = 17,
	HIP_REFERENCE = 18,
	HIP_REFERENCE_INIT = 19,
	SPINE_MID_TO_SHOULDER = 20,
	LEFT_SHOULDER_TO_SPINE_SHOULDER = 21,
	LEFT_UPPER_TORSO = 22,
	LEFT_REFERENCE1 = 23,
	LEFT_HIP_TO_SPINE_BASE = 24,
	SPINE_MID_TO_BASE = 25,
	LEFT_LOWER_TORSO = 26,
	LEFT_REFERENCE2 = 27,
	RIGHT_SHOULDER_TO_SPINE_SHOULDER = 28,
	RIGHT_UPPER_TORSO = 29,
	RIGHT_REFERENCE1 = 30,
	RIGHT_HIP_TO_SPINE_BASE = 31,
	RIGHT_LOWER_TORSO = 32,
	RIGHT_REFERENCE2 = 33,
	LOWER_TORSO = 34,
	PRE_LEFT_HIP_TO_RIGHT_HIP = 35,
	SPINE_BASE_CUR_TO_PRE = 36,
	PRE_LOWER_TORSO = 37,
	SPINE_BASE_TO_SHOULDER = 38,
	LEFT_ANKLE_TO_KNEE = 39,
	LEFT_KNEE_TO_HIP = 40,
	RIGHT_KNEE_TO_HIP = 41,
	RIGHT_ANKLE_TO_KNEE = 42,
	LEFT_ELBOW_TO_SHOULDER = 43,
	LEFT_WRIST_TO_ELBOW = 44,
	RIGHT_ELBOW_TO_SHOULDER = 45,
	RIGHT_WRIST_TO_ELBOW = 46,
	SPINE_SHOULDER_TO_HEAD = 47,
	VectorType_Count = 48
};
enum _HandType{
	LEFT_HAND = 0,
	RIGHT_HAND = 1,
	HandType_Count = ( RIGHT_HAND+1 )
};
enum _skeletonVectorType
{
	HLeftTibia = 0,
	RLeftTibia = 1,
	HLeftThigh = 2,
	RLeftThigh = 3,
	HTorso = 4,
	RTorso = 5,
	HRightThigh = 6,
	RRightThigh = 7,
	HRightTibia = 8,
	RRightTibia = 9,
	HLeftUpperArm = 10,
	RLeftUpperArm = 11,
	HLeftLowerArm = 12,
	RLeftLowerArm = 13,
	HRightUpperArm = 14,
	RRightUpperArm = 15,
	HRightLowerArm = 16,
	RRightLowerArm = 17,
	_HLeftTibia = 18,
	_RLeftTibia = 19,
	_HLeftThigh = 20,
	_RLeftThigh = 21,
	_HTorso = 22,
	_RTorso = 23,
	_HRightThigh = 24,
	_RRightThigh = 25,
	_HRightTibia = 26,
	_RRightTibia = 27,
	_HLeftUpperArm = 28,
	_RLeftUpperArm = 29,
	_HLeftLowerArm = 30,
	_RLeftLowerArm = 31,
	_HRightUpperArm = 32,
	_RRightUpperArm = 33,
	_HRightLowerArm = 34,
	_RRightLowerArm = 35,
	HHead = 36,
	RHead = 37,
	_HHead = 38,
	_RHead = 39,
	HumanDirection = 40,
	AbsoluteFrameReference = 41,
	skeletonVectorType_Count = 42
};
struct _jointVector
{
	float x;
	float y;
	float z;
};
struct _skeletonVector
{
	float x;
	float y;
	float z;
};
#define PI 3.14159265358979323846
namespace AL
{
  // This is a forward declaration of AL:ALBroker which
  // avoids including <alcommon/albroker.h> in this header
  class ALBroker;
}
class naoRobot : public AL::ALModule
{
public:
  naoRobot(boost::shared_ptr<AL::ALBroker> broker,const std::string &name);
  virtual ~naoRobot();
  void startImitation(_kinectData kinectData);
  int p,m,m1,m2,m3,n,r,s,t,DoubleSupportTimes;
  float FrameChange;
private: 
  void getBodyAngles(_kinectData kinectData);
  void clock1();
  void evaluation();
  void observation();
  void ctrHandState(_kinectData kinectData);
  //math method
  void getVector(_kinectData kinectData);
  void getVectorDiff(_jointVector& jointVector, _kinectData kinectData, int joint1, int joint2);
  void humanAbsoluteFrame(_skeletonVector& skeletonVector, _jointVector jointVector, float RotationAngle);
  void getCrossVector(_jointVector jointVector1,_jointVector jointVector2,_jointVector& jointVector);
  float getCos(_jointVector jointVector1,_jointVector jointVector2);
  float getCos1(_skeletonVector skeletonVector1,_skeletonVector skeletonVector2);
  bool isLeftHandOpen;
  bool isRightHandOpen;
  float FootHeight, Lprex,Lprey,Lcurx,Lcury,Ldisx,Rdisx,disx,Rprex,Rprey,Rcurx,Rcury,Ldisy,Rdisy,disy,prea,cura,Ld,Rd,Lprex1,Lprey1,Lcurx1,Lcury1,Ldisx1,Rdisx1,disx1,Rprex1,Rprey1,Rcurx1,Rcury1,Ldisy1,Rdisy1,disy1,prea1,cura1,Ld1,Rd1,theta,dist,b,c,curx,cury,prex,prey,varepsilon,theta_VR,theta_TR,px,py,pz,LLCount,RLCount,LLowerCount,RLowerCount;
  float c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,x31,x32,x33,LandingLAP,LandingLAR,LandingRAP,LandingRAR;
  float R1_1,R1_2,R1_3,R2_1,R2_2,R2_3,R3_1,R3_2,R3_3,R4_1,R4_2,R4_3,R5_1,R5_2,R5_3,R56_1,R56_2,R56_3,R6_1,R6_2,R6_3,R7_1,R7_2,R7_3,cp1,sp1,cp2,sp2,R8_1,R8_2,R8_3,R9_1,R9_2,R9_3,R10_1,R10_2,R10_3,R11_1,R11_2,R11_3,R12_1,R12_2,R12_3,HL,HR;
  float RotationAngle;
  float c7_1,c7_2,c8_1,c8_2,c9_1,c9_2,c10_1,c10_2,s7_1,s7_2,s8_1,s8_2,s9_1,s9_2,s10_1,s10_2;
  float R7_1_1,R7_1_2,R7_1_3,R7_2_1,R7_2_2,R7_2_3,R8_1_1,R8_1_2,R8_1_3,R8_2_1,R8_2_2,R8_2_3,R9_1_1,R9_1_2,R9_1_3,R9_2_1,R9_2_2,R9_2_3,R10_1_1,R10_1_2,R10_1_3,R10_2_1,R10_2_2,R10_2_3;
  float r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,_r1,_r2,_r3,_r4,_r5,_r6,_r7,_r8,_r9,_r10;
  float c1_1_1,c1_1_2,c1_1_3,c1_2_1,c2_1_1,c2_1_2,c2_1_3,c2_2_1,c3_1_1,c3_1_2,c3_2_1,c3_2_2,c4_1_1,c4_1_2,c4_2_1,c4_2_2,c5_1,c5_2,s1_1_1,s1_1_2,s1_1_3,s1_2_1,s2_1_1,s2_1_2,s2_1_3,s2_2_1,s3_1_1,s3_1_2,s3_2_1,s3_2_2,s4_1_1,s4_1_2,s4_2_1,s4_2_2,s5_1,s5_2;
  float V1_1,V1_2,V1_3,V2_1,V2_2,V2_3,V3_1,V3_2,V3_3,V4_1,V4_2,V4_3,V5_1,V5_2,V5_3,V6_1,V6_2,V6_3,V7_1,V7_2,V7_3,V8_1,V8_2,V8_3,V9_1,V9_2,V9_3,V10_1,V10_2,V10_3,V11_1,V11_2,V11_3,V12_1,V12_2,V12_3,V13_1,V13_2,V13_3,V14_1,V14_2,V14_3,V15_1,V15_2,V15_3,V16_1,V16_2,V16_3,V17_1,V17_2,V17_3,V18_1,V18_2,V18_3;
  float evaluation1,evaluation2,_evaluation2;
  float R611,R612,R613,R621,R622,R623,R631,R632,R633;
  vector<float>LHR;
  vector<float>LHP;
  vector<float>LKP;
  vector<float>LAP;
  vector<float>LAR;
  vector<float>LHYP;
  vector<float>RHR;
  vector<float>RHP;
  vector<float>RKP;
  vector<float>RAP;
  vector<float>RAR;
  vector<float>RHYP;
  vector<float>LSR;
  vector<float>LSP;
  vector<float>LER;
  vector<float>LEY;
  vector<float>RSR;
  vector<float>RSP;
  vector<float>RER;
  vector<float>REY;
  vector<float>HY;
  vector<float>HP;
  vector<float>result;
  //clock time
  clock_t  clockInitial, clockRecord, clockRandom;

  LARGE_INTEGER  large_interger;
  double dff;  
  __int64  time_angle1, time_angle2;  
  ofstream outfile;

  int time_total, time_random;
  double time_angle;

  AL::ALMotionProxy motionProxy;
  AL::ALRobotPostureProxy robotProxy;
  float naoJointAngles[AngleType_Count];
  _jointVector jointVector[VectorType_Count];
  _skeletonVector skeletonVector[skeletonVectorType_Count];
};
