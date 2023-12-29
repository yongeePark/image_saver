#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp> // this is to us cv2::imread function
#include <cv_bridge/cv_bridge.h>

#include<fstream>
#include <thread> // this is to use sleep function
#include <chrono>

using namespace std;
void LoadImages(const std::string &strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRGB,
                std::vector<std::string> &vstrImageFilenamesD, std::vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    for(int i=0;i<argc;i++)
    {
        std::cout<<"argv "<<i<<" : "<<argv[i]<<std::endl;
    }

    ros::init(argc,argv,"image publihser");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image = it.advertise("/camera/color/image_raw", 1);
    image_transport::Publisher pub_depth = it.advertise("/camera/depth/image_rect_raw", 1);

    sensor_msgs::ImagePtr image_msg;
    sensor_msgs::ImagePtr depth_msg;

    std::vector<string> vstrImageFilenamesRGB;
    std::vector<string> vstrImageFilenamesD;
    std::vector<double> vTimestamps;

    std::string strAssociationFilename = string(argv[1]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    int delay_time;
    if (!nh_param.getParam(ros::this_node::getName()+"/delay_time",delay_time))
    {        std::cout<<"Please set delay time"<<std::endl
        <<"shut down the program"<<std::endl;
        return 1;
    }
    int start_index = 0;
    if (nh_param.getParam(ros::this_node::getName()+"/start_index",start_index))
    {        
        std::cout<<"Start index is set to "<<start_index<<std::endl;
    }

    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
    ros::Rate loop_rate(1000.0/delay_time);
    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=start_index; ni<nImages; ni++)
    {   
        std::cout<<"Print image "<<ni+1<<" of "<<nImages<<std::endl;
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[2])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[2])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        std::cout<<"image H,W : "<<imRGB.rows<<", "<<imRGB.cols<<std::endl;
        double tframe = vTimestamps[ni];
        if(imRGB.empty())
        {
            std::cerr << std::endl << "Failed to load image at: "
                 << string(argv[2]) << "/" << vstrImageFilenamesRGB[ni] << std::endl;
            return 1;
        }

        // Publish Image

        image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", imRGB).toImageMsg();
        depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", imD).toImageMsg();

        ros::Time pub_time = ros::Time::now();
        image_msg->header.stamp = pub_time;
        depth_msg->header.stamp = pub_time;

        pub_image.publish(image_msg);
        pub_depth.publish(depth_msg);
        // std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
        loop_rate.sleep();
        if(!ros::ok())
        {   break;  }
        
    }

}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    std::ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        std::string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
}
