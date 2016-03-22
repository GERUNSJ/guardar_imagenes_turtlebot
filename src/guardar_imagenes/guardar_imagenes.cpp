/* Este es un nodo de ROS Indigo para guardar imagenes de /camera/depth_registered/image_rect_raw
 * y de /camera/rgb/color/image_rect_raw de un Turtlebot1 para luego procesarlas con otro programa. 
 * Las de profundidad se guardan como unsigned int de 16 bits 
 * y 1 canal, las de color se guardan como unsigned int de 8 bits en 3 canales.
 * Se utiliza un suscriptor sincronizado para guardar el par de imagenes que estén más
 * cercanas en el tiempo.
 * LAS IMAGENES SE GUARDAN ADONDE SE EJECUTE EL NODO.
 * ---------------------------------------------------------
 * Creado por Fabricio Emder y Pablo Aguado en el 2016 */


/* This is a ROS Indigo node for saving Turtlebot images from the /camera/depth_registered/image_rect_raw
 * and /camera/rgb/color/image_rect_raw topics, for further processing outside of the program. 
 * Depth images are saved as 1 channel 16 bit unsigned int PNGs (CV_16UC1), and 
 * RGB images are saved as 3 channel 8 bit unsigned int PNGs (CV_8UC3).
 * A synchronized subscriber is used, for saving the pair of images that are most closer in time.
 * THE IMAGES ARE SAVED WHEREVER THE NODE IS RUN.
 * Created by Fabricio Emder and Pablo Aguado - 2016
 * */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>	// OpenCV
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>


/* La política de sincronización exacta NO FUNCIONA para los topics seleccionados, ya 
 * que la Kinect o sus drivers no los están publicando con la misma marca temporal.
 * Ver más en http://wiki.ros.org/message_filters 
 * 
 * Exact sychronization policy IS NOT WORKING for the topics used, because the kinect
 * is not publishing them with the same timestamp. 
 * See more in http://wiki.ros.org/message_filters  */

//#define EXACT
#define APPROXIMATE


#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif

 
 
using namespace std;
//using namespace sensor_msgs;
using namespace message_filters;


// Contador para la numeración de los archivos.
// Counter for filenames.
unsigned int cnt = 1;



// Handler / callback
void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
{
		//ROS_INFO_STREAM("Adentro del callback\n");
	  cv_bridge::CvImagePtr img_ptr_rgb;
		cv_bridge::CvImagePtr img_ptr_depth;
    try{
        img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }
    try{
        img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    cv::Mat& mat_depth = img_ptr_depth->image;
		cv::Mat& mat_rgb = img_ptr_rgb->image;
		
		char file_rgb[100];
		char file_depth[100];
		
		sprintf( file_rgb, "%04d_rgb.png", cnt );
		sprintf( file_depth, "%04d_depth.png", cnt );
		
		vector<int> png_parameters;
		png_parameters.push_back( CV_IMWRITE_PNG_COMPRESSION );
		/* We save with no compression for faster processing.
		 * Guardamos PNG sin compresión para menor retardo. */
		png_parameters.push_back( 9 ); 
		
		cv::imwrite( file_rgb , mat_rgb, png_parameters );
		cv::imwrite( file_depth, mat_depth, png_parameters );
		
		ROS_INFO_STREAM(cnt << "\n");
		ROS_INFO_STREAM("Imágenes guardadas - "
		                "Images saved\n");
		
		cnt++;
		
}





int main(int argc, char** argv)
{
	// Initialize the ROS system and become a node.
  ros::init(argc, argv, "guardar_imagenes");
  ros::NodeHandle nh;
	
	
	message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/camera/depth_registered/hw_registered/image_rect_raw" , 1 );
	message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/camera/rgb/image_rect_color" , 1 );

	
#ifdef EXACT
	typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif
#ifdef APPROXIMATE
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif
	
	
  // ExactTime or ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
  sync.registerCallback(boost::bind(&callback, _1, _2));

	
	while(ros::ok())
	{
		char c;
		
		ROS_INFO_STREAM("\nIngrese 'a' para guardar un par de imágenes o 'b' para guardar 300 imágenes\n"
		                "Enter 'a' to save a pair of images or 'b' to automatically save 300 images\n");
		cin.get(c);
		cin.ignore();
		c = tolower(c);
		ROS_INFO_STREAM("You entered " << c << "\n");
		
		if( c == 'a' )
		{
				/* Le damos el control a la función callback cuando haya imágenes.
				* We give control to the callback function.*/
				ros::spinOnce();	
		}
		
		else if( c == 'b' )
		{
			unsigned int cntb = 1;
			while( cntb <= 300 )
			{
				ros::spinOnce();
			}
		}
		
		else break;

	}
	ROS_INFO_STREAM("Cerrando nodo\nClosing node\n");

  return 0;
}