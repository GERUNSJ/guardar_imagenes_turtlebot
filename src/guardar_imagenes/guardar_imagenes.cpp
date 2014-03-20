#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*Este es un nodo de ROS Hydro para guardar imagenes de /camera/depth_registered/image_raw
 * y de /camera/rgb/color/image_raw de un Turtlebot1 para luego procesarlas con otro programa. 
 * Las de profundidad se guardan como unsigned int de 16 bits 
 * y 1 canal, las de color se guardan como unsigned int de 8 bits en 3 canales.
 * Se guardan adonde se corra el nodo.
 * ---------------------------------------------------------
 * Creado por Fabricio Emder y Pablo Aguado en el 2014 */
 

volatile int cnt = 0;		//Contador para ponerle nombre a las imagenes.

void retorno_depth(const sensor_msgs::Image& msg)
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
	cnt++;                                			//each time an image comes increment cnt
	sprintf(file1,"imagen%d_depth.png",cnt);		//Solo guarda en 16 bits si es png o tiff 
	sprintf(file2,"imagen%d_depth.tiff",cnt);        

    std::vector<int> parametros;					//Objeto vector de C++.
    parametros.push_back(CV_IMWRITE_PNG_COMPRESSION);
    parametros.push_back(0);

	imwrite(file1,mat,parametros);
	imwrite(file2,mat);
	
	ROS_INFO_STREAM("Imagen de profundidad guardada\n");
}

void retorno_color(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr img_ptr;
    try{
        img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    cv::Mat &mat = img_ptr->image;

	char file1[100];
	char file2[100];
	cnt++;                               			//each time an image comes increment cnt
	sprintf(file1,"imagen%d_color.png",cnt);		//Solo guarda en 16 bits si es png o tiff 
	sprintf(file2,"imagen%d_color.tiff",cnt);        

    std::vector<int> parametros;					//Objeto vector de C++.
    parametros.push_back(CV_IMWRITE_PNG_COMPRESSION);
    parametros.push_back(0);

	imwrite(file1,mat,parametros);
	imwrite(file2,mat);
	
	ROS_INFO_STREAM("Imagen color guardada\n");
}


int main(int argc, char **argv)
{
    //initialize the ROS system and become a node.
    ros::init(argc, argv, "guardar_imagenes");
    ros::NodeHandle nh;


    //image_transport::ImageTransport it(nh);
    ros::Subscriber suscriptor_depth;
    ros::Subscriber suscriptor_color;

    suscriptor_depth = nh.subscribe("/camera/depth_registered/image_raw", 1, &retorno_depth);
    suscriptor_color = nh.subscribe("/camera/rgb/color/image_raw"), 1, &retorno_color);

    ROS_INFO_STREAM("Apretar enter para guardar las imagenes\n");
    
	while(ros::ok())
	{
		while (1)
		{
			if ('\n' == getchar())
			   break;
		}

		ros::spinOnce();	//Le da el control a los handlers
	}
    return 0;
}
