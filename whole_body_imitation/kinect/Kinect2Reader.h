#pragma once
#include <Windows.h>
#include <Kinect.h>
#include <Kinect.Face.h>
#include<iostream>
#include <opencv2\opencv.hpp>
#include<stdio.h>
using namespace cv;
using namespace std;
#define M_PI 3.14159265358979323846
#define BODY_NOT_TRACKED 7
enum KinectHandType{
	LEFT_HAND = 0,
	RIGHT_HAND = 1,
	HandType_Count = (RIGHT_HAND + 1)
};
class Kinect2Reader
{	
public:
	Kinect2Reader();
	~Kinect2Reader();

	HRESULT InitSensor();
	HRESULT KinectUpdate();
	void basicFrameProcess(IColorFrame*	pColorFrame, IDepthFrame* pDepthFrame, IBodyFrame* pBodyFrame, IBodyIndexFrame* pBodyIndexFrame);
	void faceTracking();

	struct _bodyJointsPosition{
		float x;
		float y;
		float z;
	}bodyJointsPosition;
	struct _headRotationAngle{
		float pitch;  //turn upwards and downwwards
		float yaw;	//turn leftwards and rightwards
	}headRotationAngle;

	struct _kinectData{
		_bodyJointsPosition bodyJointsArray[JointType_Count];
		_headRotationAngle headRotationAngle;
		bool handState[HandType_Count];
		bool kinectDataReady;
		bool skeletonReady;
	}kinectData;	
private:
	//Basic process of color, depth, and body infomation
	void processColorFrame(IColorFrame* pColorFrame);
	void processDepthFrame(IDepthFrame*	pDepthFrame);
	void processBodyFrame(IBodyFrame* pBodyFrame, IBodyIndexFrame* pBodyIndexFrame);
	
	//Process body infomation
	void processBody(IBody** ppBodies);
	void drawHandState(const DepthSpacePoint depthSpacePosition, HandState handState);
	void drawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1);
	//Process face infomation
	void drawFacesData(Mat colorImg, int iFace, const RectI* pFaceBox, const PointF* pFacePoints, const Vector4* pFaceRotation, const DetectionResult* pFaceProperties);
	bool validateFaceBoxAndPoints(const RectI* pFaceBox, const PointF* pFacePoints);
	void extractFaceRotationInDegrees(const Vector4* pQuaternion, float* pPitch, float* pYaw, float* pRoll);
	
	void copyJointData2KinectData(_kinectData &kinectData, Joint joints[JointType_Count]);
	void assignment(_bodyJointsPosition &bodyJointsPosition, CameraSpacePoint Position);
	
	IKinectSensor*	m_pKinectSensor;
	IMultiSourceFrameReader*	m_pMultiSourceFrameReader;	
	
	RGBQUAD* m_pColorRGBX;
	
	//For face tracking
	HRESULT h_getBody;
	IBody* ppBodies[BODY_COUNT];

	//Used for mapping the joint coordinates from Kinect space to depth space
	ICoordinateMapper*      m_pCoordinateMapper;

	IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];
	IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];

	Mat skeletonImg;
	Mat depthImg;
	Mat colorImg;
	double	c_FaceRotationIncrementInDegrees;
	DWORD	c_FaceFrameFeatures;
	int trackedID;
	
	static const int        DepthWidth = 512;
	static const int        DepthHeight = 424;
	static const int        ColorWidth = 1920;
	static const int        ColorHeight = 1080;	
	
};
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != nullptr)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}