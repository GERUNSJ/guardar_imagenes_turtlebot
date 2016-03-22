#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//#define EXACT
#define APPROXIMATE

/*Este es un nodo de ROS Hydro para guardar imagenes de /camera/depth_registered/image_raw
 * y de /camera/rgb/color/image_raw de un Turtlebot1 para luego procesarlas con otro programa. 
 * Las de profundidad se guardan como unsigned int de 16 bits 
 * y 1 canal, las de color se guardan como unsigned int de 8 bits en 3 canales.
 * Se guardan adonde se corra el nodo.
 * ---------------------------------------------------------
 * Creado por Fabricio Emder y Pablo Aguado en el 2014 */
 
using namespace std;
//using namespace sensor_msgs;
using namespace message_filters;

// void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
// {
//   // Solve all of perception here...
// }

unsigned int cnt = 1;

void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
{
		ROS_INFO_STREAM("Adentro del callback\n");
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
		
		sprintf( file_rgb, "image%d_rgb.png", cnt );
		sprintf( file_depth, "image%d_depth.png", cnt );
		
		vector<int> png_parameters;
		png_parameters.push_back( CV_IMWRITE_PNG_COMPRESSION );
		png_parameters.push_back( 0 );
		
		cv::imwrite( file_rgb , mat_rgb, png_parameters );
		cv::imwrite( file_depth, mat_depth, png_parameters );
		
		cnt++;
		
		
/*		
		char file2[100];
	//cnt++;                               			//each time an image comes increment cnt
	sprintf(file1,"imagen%d_color.png",cnt);		//Solo guarda en 16 bits si es png o tiff 
	sprintf(file2,"imagen%d_color.tiff",cnt);        

    std::vector<int> parametros;					//Objeto vector de C++.
    parametros.push_back(CV_IMWRITE_PNG_COMPRESSION);
    parametros.push_back(0);

	imwrite(file1,mat,parametros);
	imwrite(file2,mat);*/

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "guardar_imagenes");

  ros::NodeHandle nh;
//   message_filters::Subscriber<Image> image_sub(nh, "image", 1);
//   message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
	
// 	subscriber_depth = nh.subscribe("/camera/depth_registered/hw_registered/image_rect_raw", 1 );
// 	subscriber_rgb = nh.subscribe("/camera/rgb/image_color", 1 );
	
	message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/camera/depth_registered/hw_registered/image_rect_raw" , 1 );
	message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/camera/rgb/image_rect_color" , 1 );

  //typedef sync_policies::ExactTime<Image, CameraInfo> MySyncPolicy;
#ifdef EXACT
	typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif
#ifdef APPROXIMATE
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
  sync.registerCallback(boost::bind(&callback, _1, _2));

  //ros::spin();
	
	    ROS_INFO_STREAM("aApretar enter para guardar las imagenes\n");
    
	while(ros::ok())
	{
		while (1)
		{
			if ('\n' == getchar())
			   break;
		}
		ROS_INFO_STREAM("Agarré un enter\n");
		ros::spinOnce();	//Le da el control a los handlers
	}

  return 0;
}