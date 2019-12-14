#include "Kinect2Reader.h"
#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;
Kinect2Reader myKinect;
DWORD WINAPI KinectDataThread(LPVOID pParam)
{
	while (1)
	{
		if (myKinect.kinectData.kinectDataReady)
		{
			Kinect2Reader::_kinectData *kinectDataPtr = &myKinect.kinectData;
			ofstream outFile("D:/Nao_Imitation/MyKinectProject/MyKinectProject/Files/kinectData.dat", ios::out | ios::binary);
			if (!outFile)
			{
				cout << "The file open error!" << endl;
				return 1;
			}
			else    
			{
				outFile.write((char*)(kinectDataPtr), sizeof(Kinect2Reader::_kinectData));  
				/*cout << kinectDataPtr->bodyJointsArray[JointType_WristLeft].x << endl;
				cout << kinectDataPtr->bodyJointsArray[JointType_WristLeft].y << endl;
				cout << kinectDataPtr->bodyJointsArray[JointType_WristLeft].z << endl;*/
				/*cout << "kinectDataReady" << kinectDataPtr->kinectDataReady << endl;
				cout << "skeletonReady" << kinectDataPtr->skeletonReady << endl;*/
				//Sleep(2000);
			}
		}
	}
}
int main()
{	
	HRESULT hr = myKinect.InitSensor();
	HANDLE m_hProcesss = CreateThread(NULL, 0, KinectDataThread, 0, 0, 0);
	if (SUCCEEDED(hr))
	{
		while (1)
		{
			myKinect.KinectUpdate();
			myKinect.faceTracking();
			if (waitKey(1) == 27)	break;
		}
	}
	else{
		cout << "kinect initialization failed!" << endl;
		system("pause");

	}
	return 0;
}

