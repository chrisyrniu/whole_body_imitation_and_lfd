#include <Windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <conio.h>
#include <alerror/alerror.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include "Kinect.h"
#include "naoRobot.h"
#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8000) ? 1:0)
using namespace std;
_kinectData kinectData;
_kinectData* kinectDataPtr = &kinectData;
int q=0;
char ch;
char ch1;
//This thread provide the data of Kinect
DWORD WINAPI NaoGetDataThread(LPVOID pParam)
{
	while (1)
	{
		q=q+1;
		//The first parameter depend on where your kinectData.dat locats
		ifstream inFile("E:/Nao_Imitation/MyKinectProject/MyKinectProject/Files/kinectData.dat", ios::in | ios::binary); 
		if (!inFile)
		{
			cout << "The inFile open error!" << endl;
			return 0;
		}
		else
		{
			
			inFile.read((char*)kinectDataPtr,sizeof(_kinectData));
			/*cout<<"kinectDataReady"<<kinectDataPtr->kinectDataReady<<endl;
			cout<<"skeletonReady"<<kinectDataPtr->skeletonReady<<endl;
			if(kinectDataPtr->kinectDataReady)
			{
			cout<<kinectDataPtr->bodyJointsArray[JointType_WristLeft].x<<endl;
			cout<<kinectDataPtr->bodyJointsArray[JointType_WristLeft].y<<endl;
			cout<<kinectDataPtr->bodyJointsArray[JointType_WristLeft].z<<endl;
			
			}
			Sleep(500);*/
		}
		inFile.close();
		
	}
}
int main(int argc,char *argv[])
{	
	//Initialize of Kinect data
	kinectDataPtr->kinectDataReady=false;
	kinectDataPtr->handState[0]=false;
	kinectDataPtr->handState[1]=false;
	kinectDataPtr->skeletonReady = false;
	//Connect the PC to the target Nao robot
	const string brokerName = "mybroker";
	const string NaoIp = argv[1];
	const int NaoPort = 9559;
	boost::shared_ptr<AL::ALBroker> broker;	
	try{
		broker=AL::ALBroker::createBroker(brokerName, "0.0.0.0", 54000, argv[1],NaoPort);			
	}catch (const AL::ALError& e){
		cerr<<"catch exception"<<e.what()<<endl;
	}
	naoRobot Nao(broker,brokerName);
	Nao.p=0;
	Nao.m=0;
	Nao.m1=0;
	Nao.m2=0;
	Nao.m3=0;
	Nao.n=0;
	Nao.s=0;
	Nao.t=0;
	Nao.DoubleSupportTimes=0;
	Nao.FrameChange=0;

	HANDLE m_hProcesss = CreateThread(NULL, 0, NaoGetDataThread, 0, 0, 0);
	if((ch=getche())=='s')
	{
		while(1)
		{
			Nao.p=q;
			if(kinectDataPtr->kinectDataReady&&kinectDataPtr->skeletonReady)
				Nao.startImitation(*kinectDataPtr);
			if(KEY_DOWN('E'))break;
		}
	}
	return 0;
}
