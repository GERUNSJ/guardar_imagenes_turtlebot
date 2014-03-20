#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

volatile int cnt = 0;

void RetornoImagen(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr ddd;
    try{
        ddd = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    double minVal, maxVal;
    cv::Mat &mat = ddd->image;
    cv::Mat img2;
    cv::Mat invertida;
    cv::imshow("windowName",mat );
    //mat.convertTo(img2,CV_8U, 255.0/(maxVal - minVal), 100.0 );
    minMaxLoc(mat, &minVal, &maxVal,NULL,NULL); //find minimum and maximum intensities
    img2=mat;
    //img2=img2-minVal;
    //img2=img2*3;
    //img2=img2*(30000.0/(maxVal));
    mat.convertTo(img2,CV_16UC1,65535.0/(maxVal - minVal), -minVal * 65535.0/(maxVal - minVal));
    cv::imshow("ventana2",img2);
    //cv::subtract(cv::Scalar::all(65535),mat,invertida);
    //cv::imshow("ventana3",invertida);
    cv::waitKey(3);

//ROS_INFO("asdasd");

char file1[100];
char file2[100];
cnt++;                               //each time an image comes increment cnt
sprintf(file1,"/home/pablo/deteccion_ros/imagen%d.png",cnt);        //different filename so it doesnt overwrite
sprintf(file2,"/home/pablo/deteccion_ros/imagen%d.tiff",cnt);        
/*
std::vector<int> p;
p[0] = CV_IMWRITE_PNG_COMPRESSION;
p[1] = 1; // compression factor*/	//LOS VECTORES NO SE TRABAJAN ASÍ. (sí se pueden acceder así).

    std::vector<int> pp;
    pp.push_back(CV_IMWRITE_PNG_COMPRESSION);
    pp.push_back(0);


       // imwrite(file, mat, p);
	//imwrite("/home/pablo/deteccion_ros/imagen%d.png",img2);
	//imwrite("/home/pablo/deteccion_ros/imagen%d.png",mat,pp);
	//imwrite("/home/pablo/deteccion_ros/imagen%d.tiff",mat);
	imwrite(file1,img2,pp);
	imwrite(file2,img2);



    ROS_INFO_STREAM("El minimo es:" << minVal);
    ROS_INFO_STREAM("El maximo es:" << maxVal);
    // Output modified video stream
    //ros::Publisher Pub = nh.publish(ddd->toImageMsg());

}

int main(int argc, char **argv)
{
	char a;
    //initialize the ROS system and become a node.
    ros::init(argc, argv, "obtener_depth");
    ros::NodeHandle nh;


    //image_transport::ImageTransport it(nh);
    ros::Subscriber image_sub_;
    ros::Rate rate(1);

    cvNamedWindow("windowName", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("ventana2", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("ventana3", CV_WINDOW_AUTOSIZE);

    image_sub_ = nh.subscribe("/camera/depth_registered/image_raw", 1, &RetornoImagen);

    cvStartWindowThread();
while(ros::ok())
{

	std::cout << "Apretar a" << "\n";
	do
	{
	std::cin >> a;
	}
	while(a!='a');	
ros::spinOnce();

//ROS_INFO("Hello, ROS!");

rate.sleep();

}

    cvDestroyWindow("windowName");
    cvDestroyWindow("ventana2");
    cvDestroyWindow("ventana3");
    return 0;


}
