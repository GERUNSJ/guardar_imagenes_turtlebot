#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*Este es un nodo de ROS Hydro para guardar imagenes de /camera/depth_registered/image_raw
 * para luego procesarlas con otro programa. Se guardan como unsigned int de 16 bits.
 * Se guardan en el workspace del nodo*/
 

volatile int cnt = 0;

void RetornoImagen(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr img_ptr;
    try{
        img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    cv::Mat &mat = img_ptr->image;

	char file1[100];
	char file2[100];
	cnt++;                               	//each time an image comes increment cnt
	sprintf(file1,"imagen%d.png",cnt);		//Solo guarda en 16 bits si es png o tiff 
	sprintf(file2,"imagen%d.tiff",cnt);        

    std::vector<int> parametros;
    parametros.push_back(CV_IMWRITE_PNG_COMPRESSION);
    parametros.push_back(0);

	imwrite(file1,mat,parametros);
	imwrite(file2,mat);
	ROS_INFO_STREAM("Imagen guardada\n");
}


int main(int argc, char **argv)
{
    //initialize the ROS system and become a node.
    ros::init(argc, argv, "obtener_depth");
    ros::NodeHandle nh;


    //image_transport::ImageTransport it(nh);
    ros::Subscriber image_sub_;

    image_sub_ = nh.subscribe("/camera/depth_registered/image_raw", 1, &RetornoImagen);

    cvStartWindowThread();
	while(ros::ok())
	{
		while (1)
		{
			if ('\n' == getchar())
			   break;
		}

		ros::spinOnce();
	}
    return 0;
}
