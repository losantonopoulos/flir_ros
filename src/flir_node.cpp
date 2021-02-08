#include <ros/ros.h>
#include "ros/init.h"

// System libraries
#include <stdbool.h>
#include <signal.h>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <limits>
#include <sstream>

#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>

// Thermal Grabber library
#include "thermalgrabber.h"

//OpenCV includes
#include <opencv2/core.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

// DEBUG
#define DBG
#define USE_MONO
#define USE_RGB
#define USE_RAW 

const int FRAME_WIDTH  = 640; // Assign Default image width  (in Pixels)
const int FRAME_HEIGHT = 512; // Assign Default image height (in Pixels)
const int FRAME_WIDTH_2  = FRAME_WIDTH/2;
const int FRAME_HEIGHT_2 = FRAME_HEIGHT/2;
const int FRAME_SIZE = FRAME_WIDTH*FRAME_HEIGHT*sizeof(uint16_t);

ThermalGrabber* tGr;
std::mutex mutex_bmp_update;    // Concurrency for thermal data

// Mats for processing
Mat original_raw, remapped, gray;

// Image msgs
image_transport::Publisher pub_raw, pub_rem_mono, pub_rem_rgb;

// Original image data pointer
unsigned char *pOriginal;	// Raw Mat pointer

// This function returns the diagonal distance between two Points 
// (Diagonal Distance - Pythagorean theorem for 2D space)
// (Double Pythagorean theorem for 3D space)
double distance2points(Point2d pt1, Point2d pt2){
	return sqrt( pow((pt2.x - pt1.x) ,2) + pow((pt2.y - pt1.y) ,2));
}

double distance2points(Point3d pt1, Point3d pt2){
	return sqrt( pow((pt2.x - pt1.x) ,2) + pow((pt2.y - pt1.y) ,2) + pow((pt2.z - pt1.z) ,2));
}

// Proper Shutdown
void mySigintHandler(int sig)
{
	ROS_INFO("flir_node will shut down...");
	//call shutdown()
	ros::shutdown();
}

/*
	This is the class created to be assigned with the capturre callback
*/
class Capture
{
public:
    void capture();
};

void callbackTauImage(TauRawBitmap& tauRawBitmap,void* caller)
{

	if (tauRawBitmap.data == NULL || tauRawBitmap.width != FRAME_WIDTH || tauRawBitmap.height != FRAME_HEIGHT)
	return;

	mutex_bmp_update.try_lock();
	{
		// Aquire Data
		short unsigned int  *pTauRaw;		// Bitmap pointer
		pTauRaw = tauRawBitmap.data;
		memcpy(pOriginal , pTauRaw, FRAME_SIZE);

		sensor_msgs::ImagePtr image_msg_raw, image_msg_mono, image_msg_rgb; 

		//ROS_INFO("FRAME");

		// Publish RAW Radiometric Feed
		#ifdef USE_RAW 
		image_msg_raw = cv_bridge::CvImage(std_msgs::Header(), "mono16", original_raw).toImageMsg();
		pub_raw.publish(image_msg_raw);
		#endif

		ushort max_16_temp = 0x0;
		ushort min_16_temp = 0xFFFF;

		// Find minimum and maximum limits
		for(int i=0; i < FRAME_WIDTH; i++){
			
			for(int j=0; j < FRAME_HEIGHT; j++){
				
				if(original_raw.at<ushort>(j, i) > max_16_temp) {

					max_16_temp = original_raw.at<ushort>(j, i);
				}

				if(original_raw.at<ushort>(j, i) < min_16_temp) {

					min_16_temp = original_raw.at<ushort>(j, i);
				}

			}
		}

		// Compute lamda 
		double lamda = 255.0f / double(max_16_temp - min_16_temp);

		// Apply remmap 16 -> 8 bit based on lamda
		for(int i=0; i < FRAME_WIDTH; i++){
			
			for(int j=0; j < FRAME_HEIGHT; j++){
				
				gray.at<uchar>(j, i) = (original_raw.at<ushort>(j, i) - min_16_temp)*lamda;
			}
		}

		//Publish Grayscale Feed
		#ifdef USE_MONO
		image_msg_mono = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
		pub_rem_mono.publish(image_msg_mono);
		#endif

		#ifdef USE_RGB
		// Apply a colormap for easier understanding
		applyColorMap(gray, remapped, COLORMAP_HOT); // COLORMAP_JET , COLORMAP_HOT , COLORMAP_TURBO

		// Publish RGB Colormapped Frame
		image_msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", remapped).toImageMsg();
		pub_rem_rgb.publish(image_msg_rgb);
		#endif
	

	}
	mutex_bmp_update.unlock();	
}

// Assigning the ThermalGrabber callback
void Capture::capture()
{
    ROS_INFO("Starting Capture");

    tGr = new ThermalGrabber(callbackTauImage, this);

    int mWidth  = tGr->getResolutionWidth();
    int mHeight = tGr->getResolutionHeight();

    ROS_INFO("Resolution w/h : %d / %d", mWidth, mHeight);

    // enable TLinear in high resolution on TauCores
    tGr->enableTLinearHighResolution();    
}
/*
void rawCallback(const sensor_msgs::ImageConstPtr& msg){

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
		original_raw = cv_ptr->image;

		//process_flir_feed();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

*/

int main(int argc,char *argv[]){

	ros::init(argc, argv, "flir_node"); // Initialize the node

	ros::NodeHandle nh;
	
	ros::Subscriber image_sub;

	// We need Image Transport in order to publish images
	image_transport::ImageTransport it(nh);

	// Setup Image Publishers
	#ifdef USE_RAW 
	pub_raw = it.advertise("/flir_ros/image_raw", 1);
	#endif

	#ifdef USE_MONO
	pub_rem_mono = it.advertise("/flir_ros/image_remapped_mono", 1);
	#endif
	
	#ifdef USE_RGB
	pub_rem_rgb  = it.advertise("/flir_ros/image_remapped_rgb", 1);
	#endif
	original_raw   	= Mat (FRAME_HEIGHT, FRAME_WIDTH, CV_16UC1); //
	gray  			= Mat (FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1, Scalar(0)); //
	remapped	   	= Mat (FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3, Scalar(0,0,0)); //CV_8UC3

	// Assign Pointer
	pOriginal = original_raw.data;

	// Create FLIR access instance
	Capture* c = new Capture();
	c->capture();

	// Let ROS handle callbacks
	ros::spin();

	// Free memory reserved for image processing Mats
	original_raw.release();
	remapped.release();
	gray.release();

	return 0;
}
