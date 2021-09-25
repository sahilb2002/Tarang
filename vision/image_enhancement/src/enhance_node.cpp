#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<image_enhancement/color_correction.h>
#include<image_enhancement/fusion.h>

using namespace std;
using namespace cv;

class enhance{
    private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it;
    image_transport::Publisher pub_;
    image_transport::Subscriber sub_;
    double per,lamda,sf,sg,sigma;
    int level;

    public:
    
    enhance(): it(nh_){
        nh_.getParam("/color_correction/percentile",per);
        nh_.getParam("/color_correction/lamda",lamda);
        nh_.getParam("/color_correction/sigma_f",sf);
        nh_.getParam("/color_correction/sigma_g",sg);
        nh_.getParam("/fusion/pyr_level",level);
        nh_.getParam("/fusion/exposedness_sigma",sigma);
        pub_ = it.advertise("enhanced",100);
        sub_ = it.subscribe("raw_image",100,&enhance::enhance_callback,this);
    }
    
    void enhance_callback(const sensor_msgs::ImageConstPtr& img_msg){
    
    Mat org_img1 = cv_bridge::toCvShare(img_msg,"bgr8")->image;
    // enhancing the image....
    Mat org_img;
    resize(org_img1,org_img,Size(256,256));
    Mat inp1 = white_balance(org_img,per,lamda);
    Mat inp2_1 = biletral(inp1,5,sf,sg);
    Mat inp2 = clahe(inp2_1);
    Mat result = laplace_blending(inp1,inp2,sigma,level);
    
    // publishing the image to "enhanced" topic
    sensor_msgs::ImagePtr result_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",result).toImageMsg();
    pub_.publish(result_msg);
    }
};

int main(int argc, char** argv){
    ros::init(argc,argv,"enhancer");
    enhance en;
    ros::spin();
}
