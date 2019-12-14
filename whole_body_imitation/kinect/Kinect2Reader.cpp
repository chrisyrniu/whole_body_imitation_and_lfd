#include "Kinect2Reader.h"
/// Constructor
Kinect2Reader::Kinect2Reader() :
	m_pKinectSensor(NULL),
	m_pMultiSourceFrameReader(NULL),
	m_pCoordinateMapper(NULL),
	c_FaceRotationIncrementInDegrees(5.0f)
{
	colorImg.create(ColorHeight, ColorWidth, CV_8UC3);
	c_FaceFrameFeatures = FaceFrameFeatures_BoundingBoxInColorSpace
		| FaceFrameFeatures_PointsInColorSpace
		| FaceFrameFeatures_RotationOrientation
		| FaceFrameFeatures_Happy
		| FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures_LookingAway
		| FaceFrameFeatures_Glasses
		| FaceFrameFeatures_FaceEngagement;
	kinectData.handState[0] = false;
	kinectData.handState[1] = false;
	kinectData.kinectDataReady = false;
	kinectData.skeletonReady = false;
	m_pColorRGBX = new RGBQUAD[ColorWidth * ColorHeight];
	depthImg.create(DepthHeight, DepthWidth, CV_8UC1);
	depthImg.setTo(0);
	skeletonImg.create(DepthHeight, DepthWidth, CV_8UC3);
	skeletonImg.setTo(0);
	ppBodies[BODY_COUNT] = { 0 };
	for (int i = 0; i < BODY_COUNT; i++)
	{
		m_pFaceFrameSources[i] = nullptr;
		m_pFaceFrameReaders[i] = nullptr;
	}
}
/// Destructor
Kinect2Reader::~Kinect2Reader()
{
	SafeRelease(m_pCoordinateMapper);
	SafeRelease(m_pMultiSourceFrameReader);
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pKinectSensor);
}

//Initialize Kinect and get Readers
HRESULT Kinect2Reader::InitSensor()
{
	HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))	return hr;
	if (m_pKinectSensor)
	{
		//Open Kinect sensor
		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))	hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

		//Get multiple readers
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes_Depth |
				FrameSourceTypes_Color | FrameSourceTypes_Body | FrameSourceTypes_BodyIndex,
				&m_pMultiSourceFrameReader);
		}
		//Get face reader
		for (int i = 0; i < BODY_COUNT; i++)
		{
			if (SUCCEEDED(hr))	hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
			if (SUCCEEDED(hr))	hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
		}
	}
	if (!m_pKinectSensor || FAILED(hr))	return E_FAIL;

	return hr;
}
HRESULT Kinect2Reader::KinectUpdate()
{
	IColorFrame*	 pColorFrame = NULL;
	IDepthFrame*	 pDepthFrame = NULL;
	IBodyFrame*		 pBodyFrame = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	kinectData.kinectDataReady = false;
	kinectData.skeletonReady = false;
	if (!m_pMultiSourceFrameReader)	return E_POINTER;

	IMultiSourceFrame* pMultiSourceFrame = NULL;
	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	//Get color frame
	if (SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference = NULL;
		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))	hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		SafeRelease(pColorFrameReference);
	}

	//Get depth frame
	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;
		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))	hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		SafeRelease(pDepthFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;
		hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
		if (SUCCEEDED(hr))	hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
		SafeRelease(pBodyIndexFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IBodyFrameReference* pBodyFrameReference = NULL;
		hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
		if (SUCCEEDED(hr))	hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
		SafeRelease(pBodyFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		h_getBody = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
		basicFrameProcess(pColorFrame, pDepthFrame, pBodyFrame, pBodyIndexFrame);
	}

	SafeRelease(pColorFrame);
	SafeRelease(pDepthFrame);
	SafeRelease(pBodyFrame);
	SafeRelease(pBodyIndexFrame);
	SafeRelease(pMultiSourceFrame);
	return hr;
}
void Kinect2Reader::basicFrameProcess(IColorFrame* pColorFrame, IDepthFrame* pDepthFrame, IBodyFrame* pBodyFrame, IBodyIndexFrame* pBodyIndexFrame)
{
	processColorFrame(pColorFrame);
	//processDepthFrame(pDepthFrame);
	processBodyFrame(pBodyFrame, pBodyIndexFrame);
}
void Kinect2Reader::processColorFrame(IColorFrame* pColorFrame)
{
	IFrameDescription* pColorFrameDescription = NULL;
	int nColorWidth = 0;
	int nColorHeight = 0;
	ColorImageFormat imageFormat = ColorImageFormat_None;
	UINT nColorBufferSize = 0;
	RGBQUAD* pColorBuffer = NULL;
	// Get color frame data
	HRESULT hr;
	hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
	if (SUCCEEDED(hr))	hr = pColorFrameDescription->get_Width(&nColorWidth);
	if (SUCCEEDED(hr))	hr = pColorFrameDescription->get_Height(&nColorHeight);
	if (SUCCEEDED(hr))	hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
	if (SUCCEEDED(hr))
	{
		if (imageFormat == ColorImageFormat_Bgra)
		{
			hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
		}
		else if (m_pColorRGBX)
		{
			pColorBuffer = m_pColorRGBX;
			nColorBufferSize = ColorWidth * ColorHeight * sizeof(RGBQUAD);
			hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
		}
		else
		{
			hr = E_FAIL;
		}
	}
	if (pColorBuffer && (nColorWidth == ColorWidth) && (nColorHeight == ColorHeight))
	{
		// Get color image
		Mat ColorImage(nColorHeight, nColorWidth, CV_8UC4, pColorBuffer);
		ColorImage.convertTo(colorImg, CV_8UC3);
	}
	SafeRelease(pColorFrameDescription);
}
void Kinect2Reader::processDepthFrame(IDepthFrame*	pDepthFrame)
{
	UINT nDepthBufferSize = 0;
	UINT16 *pDepthBuffer = NULL;
	//HRESULT hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
	UINT16 *depthArray = new UINT16[DepthHeight * DepthWidth];//深度数据是16位unsigned int
	pDepthFrame->CopyFrameDataToArray(DepthHeight * DepthWidth, depthArray);
	//把深度数据画到MAT中
	uchar* depthData = (uchar*)depthImg.data;
	for (int j = 0; j < DepthHeight * DepthWidth; ++j)
	{
		*depthData = depthArray[j];
		++depthData;
	}
	delete[] depthArray;
	imshow("depthImg", depthImg);
	waitKey(1);
}
//将人体区域图像存到Mat中，存储人体信息到IBody数组中
void Kinect2Reader::processBodyFrame(IBodyFrame* pBodyFrame, IBodyIndexFrame* pBodyIndexFrame)
{
	BYTE *bodyIndexArray = new BYTE[DepthHeight * DepthWidth];
	pBodyIndexFrame->CopyFrameDataToArray(DepthHeight * DepthWidth, bodyIndexArray);

	//人体区域图像是8U1C，而skeletonImg定义为8U3C
	uchar* skeletonDataPtr = (uchar*)skeletonImg.data;
	for (int j = 0; j < DepthHeight * DepthWidth; ++j)
	{
		*skeletonDataPtr = bodyIndexArray[j];
		++skeletonDataPtr;
		*skeletonDataPtr = bodyIndexArray[j];
		++skeletonDataPtr;
		*skeletonDataPtr = bodyIndexArray[j];
		++skeletonDataPtr;
	}
	delete[] bodyIndexArray;

	IBody* ppBodies[BODY_COUNT] = { 0 };

	HRESULT hr;
	//把追踪到的人体信息存到IBody数组中
	hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);

	if (SUCCEEDED(hr))	processBody(ppBodies);

	for (int i = 0; i < _countof(ppBodies); ++i)
	{
		SafeRelease(ppBodies[i]);
	}
}
void Kinect2Reader::processBody(IBody** ppBodies)
{
	HRESULT hr;
	Joint joints[JointType_Count];
	trackedID = BODY_NOT_TRACKED;
	int minStance = 10;

	//Record the ID of the body that is nearest to the sensor
	for (int i = 0; i < BODY_COUNT; ++i)
	{
		BOOLEAN bTracked = false;
		hr = ppBodies[i]->get_IsTracked(&bTracked);
		if (SUCCEEDED(hr) && bTracked)
		{
			hr = ppBodies[i]->GetJoints(_countof(joints), joints);
			if (SUCCEEDED(hr) && joints[JointType_SpineBase].Position.Z<minStance)
			{
				minStance = joints[JointType_SpineBase].Position.Z;
				trackedID = i;
			}
		}
	}
	IBody* pBody = nullptr;
	if (trackedID != BODY_NOT_TRACKED)	 pBody = ppBodies[trackedID];
	if (pBody)
	{
		BOOLEAN bTracked = false;
		//检测到人体
		hr = pBody->get_IsTracked(&bTracked);
		if (SUCCEEDED(hr) && bTracked)
		{
			//手部状态
			HandState leftHandState = HandState_Unknown;
			HandState rightHandState = HandState_Unknown;
			//获取左右手状态
			pBody->get_HandLeftState(&leftHandState);
			pBody->get_HandRightState(&rightHandState);
			//record the hand states
			switch (leftHandState)
			{
			case HandState_Open:
				kinectData.handState[LEFT_HAND] = true;
				break;
			case HandState_Closed:
				kinectData.handState[LEFT_HAND] = false;
				break;
			default:break;
			}
			switch (rightHandState)
			{
			case HandState_Open:
				kinectData.handState[RIGHT_HAND] = true;
				break;
			case HandState_Closed:
				kinectData.handState[RIGHT_HAND] = false;
				break;
			default:break;
			}
			//存储深度坐标系中的关节点位置
			DepthSpacePoint *depthSpacePosition = new DepthSpacePoint[_countof(joints)];

			//获得关节信息：关节编号、关节坐标、追踪状态
			hr = pBody->GetJoints(_countof(joints), joints);

			if (SUCCEEDED(hr))
			{
				////将关节点坐标从Kinect坐标转到深度坐标
				for (int j = 0; j < _countof(joints); ++j)
				{
					m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[j].Position, &depthSpacePosition[j]);
				}
				//存储关节坐标到kinectData中
				if (TrackingState_NotTracked != joints[JointType_SpineShoulder].TrackingState)
				{
					copyJointData2KinectData(kinectData, joints);
					kinectData.skeletonReady = true;
				}
				//------------------------hand state left-------------------------------
				drawHandState(depthSpacePosition[JointType_HandLeft], leftHandState);
				drawHandState(depthSpacePosition[JointType_HandRight], rightHandState);

				//---------------------------body-------------------------------
				drawBone(joints, depthSpacePosition, JointType_Head, JointType_Neck);
				drawBone(joints, depthSpacePosition, JointType_Neck, JointType_SpineShoulder);
				drawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_SpineMid);
				drawBone(joints, depthSpacePosition, JointType_SpineMid, JointType_SpineBase);
				drawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderRight);
				drawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderLeft);
				drawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipRight);
				drawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipLeft);

				// -----------------------Right Arm ------------------------------------ 
				drawBone(joints, depthSpacePosition, JointType_ShoulderRight, JointType_ElbowRight);
				drawBone(joints, depthSpacePosition, JointType_ElbowRight, JointType_WristRight);
				drawBone(joints, depthSpacePosition, JointType_WristRight, JointType_HandRight);
				drawBone(joints, depthSpacePosition, JointType_HandRight, JointType_HandTipRight);
				drawBone(joints, depthSpacePosition, JointType_WristRight, JointType_ThumbRight);

				//----------------------------------- Left Arm--------------------------
				drawBone(joints, depthSpacePosition, JointType_ShoulderLeft, JointType_ElbowLeft);
				drawBone(joints, depthSpacePosition, JointType_ElbowLeft, JointType_WristLeft);
				drawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_HandLeft);
				drawBone(joints, depthSpacePosition, JointType_HandLeft, JointType_HandTipLeft);
				drawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_ThumbLeft);

				// ----------------------------------Right Leg--------------------------------
				drawBone(joints, depthSpacePosition, JointType_HipRight, JointType_KneeRight);
				drawBone(joints, depthSpacePosition, JointType_KneeRight, JointType_AnkleRight);
				drawBone(joints, depthSpacePosition, JointType_AnkleRight, JointType_FootRight);

				// -----------------------------------Left Leg---------------------------------
				drawBone(joints, depthSpacePosition, JointType_HipLeft, JointType_KneeLeft);
				drawBone(joints, depthSpacePosition, JointType_KneeLeft, JointType_AnkleLeft);
				drawBone(joints, depthSpacePosition, JointType_AnkleLeft, JointType_FootLeft);
			}
			delete[] depthSpacePosition;
		}
	}
	imshow("skeletonImg", skeletonImg);
	waitKey(1);
}
void Kinect2Reader::drawHandState(const DepthSpacePoint depthSpacePosition, HandState handState)
{
	//给不同的手势分配不同颜色
	CvScalar color;
	switch (handState) {
	case HandState_Open:
		color = cvScalar(255, 0, 0);
		break;
	case HandState_Closed:
		color = cvScalar(0, 255, 0);
		break;
	case HandState_Lasso:
		color = cvScalar(0, 0, 255);
		break;
	default:
		return;
	}

	circle(skeletonImg, cvPoint(depthSpacePosition.X, depthSpacePosition.Y), 20, color, -1);
}
void Kinect2Reader::drawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))	return;
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))return;

	CvPoint p1 = cvPoint(depthSpacePosition[joint0].X, depthSpacePosition[joint0].Y),
		p2 = cvPoint(depthSpacePosition[joint1].X, depthSpacePosition[joint1].Y);

	//非常确定的骨架，用白色直线，否则用红色直线
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{

		line(skeletonImg, p1, p2, cvScalar(255, 255, 255));
		circle(skeletonImg, p1, 5, cvScalar(0, 255, 255), -1);
	}
	else
	{
		//不确定的骨架，用红色直线
		line(skeletonImg, p1, p2, cvScalar(0, 0, 255));
	}
}
void Kinect2Reader::faceTracking()
{
	HRESULT hr = E_FAIL;
	bool bHaveBodyData = SUCCEEDED(h_getBody);
	IFaceFrame* pFaceFrame = nullptr;
	if (BODY_NOT_TRACKED != trackedID)
		hr = m_pFaceFrameReaders[trackedID]->AcquireLatestFrame(&pFaceFrame);

	BOOLEAN bFaceTracked = false;

	if (SUCCEEDED(hr) && nullptr != pFaceFrame)	hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
	if (SUCCEEDED(hr))
	{
		if (bFaceTracked)
		{
			IFaceFrameResult* pFaceFrameResult = nullptr;
			RectI faceBox = { 0 };
			PointF facePoints[FacePointType_Count];
			Vector4 faceRotation;
			DetectionResult faceProperties[FaceProperty_Count];

			hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);
			// need to verify if pFaceFrameResult contains data before trying to access it
			if (SUCCEEDED(hr) && pFaceFrameResult != nullptr)
			{
				hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);
				if (SUCCEEDED(hr))	hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType_Count, facePoints);
				if (SUCCEEDED(hr))	hr = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
				if (SUCCEEDED(hr))	hr = pFaceFrameResult->GetFaceProperties(FaceProperty_Count, faceProperties);
				if (SUCCEEDED(hr))	drawFacesData(colorImg, trackedID, &faceBox, facePoints, &faceRotation, faceProperties);
			}
			SafeRelease(pFaceFrameResult);
		}
		else
		{
			// face tracking is not valid - attempt to fix the issue
			// a valid body is required to perform this step
			if (bHaveBodyData)
			{
				// check if the corresponding body is tracked 
				// if this is true then update the face frame source to track this body
				IBody* pBody = nullptr;
				if (BODY_NOT_TRACKED != trackedID)	pBody = ppBodies[trackedID];
				if (pBody)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);
					UINT64 bodyTId;
					if (SUCCEEDED(hr) && bTracked)
					{
						// get the tracking ID of this body
						hr = pBody->get_TrackingId(&bodyTId);
						if (SUCCEEDED(hr))
						{
							// update the face frame source with the tracking ID
							m_pFaceFrameSources[trackedID]->put_TrackingId(bodyTId);
						}
					}
				}
			}
		}
	}
	//显示追踪结果
	Mat resizedColorImg;
	//resize the color image for displaying
	resize(colorImg, resizedColorImg, Size(640, 360));
	imshow("Face tracking result", resizedColorImg);
	waitKey(1);
	kinectData.kinectDataReady = true;
	SafeRelease(pFaceFrame);
	if (bHaveBodyData)
	{
		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}
}
void Kinect2Reader::drawFacesData(Mat colorImg, int iFace, const RectI* pFaceBox, const PointF* pFacePoints, const Vector4* pFaceRotation, const DetectionResult* pFaceProperties)
{
	if (validateFaceBoxAndPoints(pFaceBox, pFacePoints))
	{
		//给不同的iFace分配不同颜色
		CvScalar Face_color;
		switch (iFace) {
		case 0:
			Face_color = cvScalar(255, 0, 0);
			break;
		case 1:
			Face_color = cvScalar(0, 255, 0);
			break;
		case 2:
			Face_color = cvScalar(0, 0, 255);
			break;
		case 3:
			Face_color = cvScalar(255, 255, 0);
			break;
		case 4:
			Face_color = cvScalar(0, 255, 255);
			break;
		case 5:
			Face_color = cvScalar(255, 0, 255);
			break;
		default:
			return;
		}
		//画脸部矩形框
		Rect faceBox(Point2f(pFaceBox->Left, pFaceBox->Top), Point2f(pFaceBox->Right, pFaceBox->Bottom));
		rectangle(colorImg, faceBox, Face_color, 2);

		//提取头部转动角度
		float pitch, yaw, roll;
		string pitchText, yawText, rollText, IDText;
		extractFaceRotationInDegrees(pFaceRotation, &pitch, &yaw, &roll);
		kinectData.headRotationAngle.pitch = -pitch*M_PI / 180;
		kinectData.headRotationAngle.yaw = yaw*M_PI / 180;
	}
}
bool Kinect2Reader::validateFaceBoxAndPoints(const RectI* pFaceBox, const PointF* pFacePoints)
{
	bool isFaceValid = false;
	if (pFaceBox != nullptr)
	{
		INT32 screenWidth = ColorWidth;
		INT32 screenHeight = ColorHeight;

		INT32 width = pFaceBox->Right - pFaceBox->Left;
		INT32 height = pFaceBox->Bottom - pFaceBox->Top;

		// check if we have a valid rectangle within the bounds of the screen space
		isFaceValid = width > 0 &&
			height > 0 &&
			pFaceBox->Right <= screenWidth &&
			pFaceBox->Bottom <= screenHeight;

		if (isFaceValid)
		{
			for (int i = 0; i < FacePointType_Count; i++)
			{
				// check if we have a valid face point within the bounds of the screen space                        
				bool isFacePointValid = pFacePoints[i].X > 0.0f &&
					pFacePoints[i].Y > 0.0f &&
					pFacePoints[i].X < ColorWidth &&
					pFacePoints[i].Y < ColorHeight;

				if (!isFacePointValid)
				{
					isFaceValid = false;
					break;
				}
			}
		}
	}

	return isFaceValid;
}
void Kinect2Reader::extractFaceRotationInDegrees(const Vector4* pQuaternion, float* pPitch, float* pYaw, float* pRoll)
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;

	// convert face rotation quaternion to Euler angles in degrees		
	double dPitch, dYaw, dRoll;
	dPitch = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / M_PI * 180.0;
	dYaw = asin(2 * (w * y - x * z)) / M_PI * 180.0;
	dRoll = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / M_PI * 180.0;

	// clamp rotation values in degrees to a specified range of values to control the refresh rate
	double increment = c_FaceRotationIncrementInDegrees;
	*pPitch = static_cast<float>(floor((dPitch + increment / 2.0 * (dPitch > 0 ? 1.0 : -1.0)) / increment) * increment);
	*pYaw = static_cast<float>(floor((dYaw + increment / 2.0 * (dYaw > 0 ? 1.0 : -1.0)) / increment) * increment);
	*pRoll = static_cast<float>(floor((dRoll + increment / 2.0 * (dRoll > 0 ? 1.0 : -1.0)) / increment) * increment);
}

void Kinect2Reader::copyJointData2KinectData(_kinectData &kinectData, Joint joints[JointType_Count])
{
	assignment(kinectData.bodyJointsArray[JointType_SpineBase], joints[JointType_SpineBase].Position);
	assignment(kinectData.bodyJointsArray[JointType_SpineMid], joints[JointType_SpineMid].Position);
	assignment(kinectData.bodyJointsArray[JointType_Neck], joints[JointType_Neck].Position);
	assignment(kinectData.bodyJointsArray[JointType_Head], joints[JointType_Head].Position);
	assignment(kinectData.bodyJointsArray[JointType_ShoulderLeft], joints[JointType_ShoulderLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_ElbowLeft], joints[JointType_ElbowLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_WristLeft], joints[JointType_WristLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_HandLeft], joints[JointType_HandLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_ShoulderRight], joints[JointType_ShoulderRight].Position);
	assignment(kinectData.bodyJointsArray[JointType_ElbowRight], joints[JointType_ElbowRight].Position);
	assignment(kinectData.bodyJointsArray[JointType_WristRight], joints[JointType_WristRight].Position);
	assignment(kinectData.bodyJointsArray[JointType_HandRight], joints[JointType_HandRight].Position);
	assignment(kinectData.bodyJointsArray[JointType_HipLeft], joints[JointType_HipLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_KneeLeft], joints[JointType_KneeLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_AnkleLeft], joints[JointType_AnkleLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_FootLeft], joints[JointType_FootLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_HipRight], joints[JointType_HipRight].Position);
	assignment(kinectData.bodyJointsArray[JointType_KneeRight], joints[JointType_KneeRight].Position);
	assignment(kinectData.bodyJointsArray[JointType_AnkleRight], joints[JointType_AnkleRight].Position);
	assignment(kinectData.bodyJointsArray[JointType_FootRight], joints[JointType_FootRight].Position);
	assignment(kinectData.bodyJointsArray[JointType_SpineShoulder], joints[JointType_SpineShoulder].Position);
	assignment(kinectData.bodyJointsArray[JointType_HandTipLeft], joints[JointType_HandTipLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_ThumbLeft], joints[JointType_ThumbLeft].Position);
	assignment(kinectData.bodyJointsArray[JointType_HandTipRight], joints[JointType_HandTipRight].Position);
	assignment(kinectData.bodyJointsArray[JointType_ThumbRight], joints[JointType_ThumbRight].Position);

}
void Kinect2Reader::assignment(_bodyJointsPosition &bodyJointsPosition, CameraSpacePoint Position)
{
	bodyJointsPosition.x = Position.X;
	bodyJointsPosition.y = Position.Y;
	bodyJointsPosition.z = Position.Z;
}