#include <windows.h>
#include <iostream> 
#include <alerror/alerror.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <math.h>
#include <string>
#include <vector>
#include <deque>

using namespace std;

int main(int argc, char *argv[])
{

	const string brokername="mybroker";
	boost::shared_ptr<AL::ALBroker> broker; 
	const AL::ALValue jointName = " Body";
	try{
		broker=AL::ALBroker::createBroker(brokername, "0.0.0.0", 54000, argv[1],9559);			
	}catch (const AL::ALError& e){
		cerr<<"catch exception"<<e.what()<<endl;
	}

	AL::ALMotionProxy motionProxy;
	AL::ALRobotPostureProxy robotProxy;

	const AL::ALValue RArmNames=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw");
	//1
	AL::ALValue RArmAngleLists;
	RArmAngleLists.arraySetSize(5);
	RArmAngleLists[0]=AL::ALValue::array(1.5765,1.5809,1.5802,1.5055,1.1237,-0.53231,-0.91165,-0.97203,-1.0361,-1.116);
	RArmAngleLists[1]=AL::ALValue::array(-0.12384,-0.13958,-0.15532,-0.16504,-0.33676,-1.0613,-1.294,-0.97851,-0.73864,-0.72895);
	RArmAngleLists[2]=AL::ALValue::array(1.1283,1.2209,1.8351,2.0857,2.0694,1.7714,0.28049,0.020525,-0.097496,-0.13444);
	RArmAngleLists[3]=AL::ALValue::array(0.37694,0.33597,0.28889,0.23269,0.23488,0.28214,0.32857,0.36655,0.41907,0.68455);
	RArmAngleLists[4]=AL::ALValue::array(0,0,0,0,0,0,0,0,0,0);

	AL::ALValue TimeLists;
	TimeLists.arraySetSize(5);
	TimeLists[0]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);
	TimeLists[1]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);
	TimeLists[2]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);
	TimeLists[3]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);
	TimeLists[4]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);


	motionProxy.angleInterpolation(RArmNames, RArmAngleLists, TimeLists, true);

	//2
	RArmAngleLists[0]=AL::ALValue::array(-1.1916,-1.2382,-1.252,-1.2427,-1.2391,-1.254,-1.2809,-0.52845,1.3505,1.5709);
	RArmAngleLists[1]=AL::ALValue::array(-0.15875,-0.33675,-0.61979,-0.9108,-1.2154,-1.4018,-1.2919,-1.1592,-1.0351,-0.90715);
	RArmAngleLists[2]=AL::ALValue::array(-0.1835,-0.23882,-0.27875,-0.37772,-0.34595,0.29472,1.3904,1.9469,2.0138,2.0059);
	RArmAngleLists[3]=AL::ALValue::array(1.0053,1.3211,1.4342,0.86497,0.45125,0.17439,0.12246,0.16121,0.19118,0.22108);
	RArmAngleLists[4]=AL::ALValue::array(0,0,0,0,0,-0.48,-1.08,-1.64,-1.64,-1.64);

	TimeLists[0]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);
	TimeLists[1]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);
	TimeLists[2]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);
	TimeLists[3]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);
	TimeLists[4]=AL::ALValue::array(0.3,0.6,0.9,1.2,1.5,1.8,2.1,2.4,2.7,3.0);

	motionProxy.angleInterpolation(RArmNames, RArmAngleLists, TimeLists, true);

}