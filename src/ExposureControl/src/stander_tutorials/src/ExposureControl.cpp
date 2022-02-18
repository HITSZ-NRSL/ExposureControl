/**
 * @file ExposureControl.cpp
 * @author Yu Wang(19B953030@stu.hit.edu.cn); Haoyao Chen(hychen5@hit.edu.cn); Shiwu Zhang; Wencan Lu. 
 * @date 2022-02-18
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#include <ros/ros.h>  
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pthread.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "std_msgs/Int32MultiArray.h" 
#include "std_msgs/String.h"
#include <math.h>  
#include <Eigen/Core>
#include <signal.h>
#include <opencv2/core/eigen.hpp> 
#include <algorithm>

#define THREAD_NUMS 5

using namespace std;
using namespace cv;
using namespace Eigen;
int weighh_flag = 1;
static uchar t[256];
float g[256];
vector<double>gamma_parament;

vector<Mat>LUT_GAMMA;
double exposure_time = 333;
double exposure_gain = 4;
double search_gamma = 1;

cv::Mat g_derivative_table(1, 256, CV_64FC1); 
std::vector<double> steps;
std::vector<double> W(256);
ros::Publisher chatter_pub;
ros::Subscriber sub;
double alfa, beta;
struct paramThread
{
    int w;
    int h;
    uchar * data;
    double* quality;
    Mat *grad_x;
    Mat *grad_y;
    Mat *gray_image;
};
struct row_col_gradient
{
    int row;
    int col;
    double gradient;
    bool is_overexposure;
    bool is_underexposure;
};

struct thread_out
{
    double quality;
    Mat grad_x;
    Mat grad_y;
    Mat gray_image;
};

void set_g_derivative_table();
thread_out calculate_m(Mat color_image);
void Lut_generate();
vector<cv::Mat> splitImage(cv::Mat image, int num, int type);
cv::Mat catImage(vector<cv::Mat> v, int type); 
static int get_near_index(float value);
void * threadProcess(void* args); 
void correct_vector_creat(uchar* gamma_i, float num);
thread_out muti_thread(Mat gray_image);
thread_out calculate_m(Mat gray_image);
thread_out heuristic_search(Mat gray_image);
void exposure_adjust(Mat color_image, double& exposure_time, double& exposure_gain);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);



int main(int argc, char **argv)
{
    fstream in2("/home/wangyu/data/g.txt"); 
    cin.rdbuf(in2.rdbuf()); 
    for (int i = 0; i < 256; i++)
    {
        cin >> g[i]; 
    }  
    set_g_derivative_table();
    int S = 255;
    double p = 0.5;
    double k = 5;
    if(weighh_flag == 0)
    {
        for(int i = 0; i <= 255; i++)
        {
            if ( i <= p*S)
                W[i] = pow(sin(M_PI_2 * i / (p * S)), k)-0.0001;
            else
                W[i] = pow(sin(M_PI_2 - M_PI_2 * (i - p*S) / (S - p * S)), k)-0.0001;
        }
    }
    else
    {    
        for(int i = 0; i <= 255; i++)
        {
            if ( i <= 127)
                W[i] = 1/(1 +0.25*exp(0.2*(63.5-i)))-0.001;
            else
                W[i] = 1/(1 +0.25*exp(0.2*(i-63.5-128)))-0.001;//regara 0.001 as penalty for over/under exposed
        }
    }
    for(int i = 0;i<13;i++)
    {
        uchar gamma_0[256];
        double s = 0.5 + 0.125*i;
        correct_vector_creat(gamma_0, s);
        gamma_parament.push_back(s);
        Mat lut(1, 256, CV_8UC1, gamma_0);
        LUT_GAMMA.push_back(lut.clone());
    }    

    ros::init(argc, argv, "exposure_control");
    ros::NodeHandle n;
    //image_transport::ImageTransport it(n);
    chatter_pub = n.advertise<std_msgs::Int32MultiArray>("/camera2/set_exposure", 1000);
    sub = n.subscribe("/camera/color/image_raw", 1000, imageCallback); 
    //ros::Rate loop_rate(10);
    ROS_INFO("Please send message!");  
    ros::spin();  
    return 0 ;
} 


void set_g_derivative_table()
{
    std::vector<double> params(6, 0);
    // 拟合5次多项式，然后对g_derivative赋值
    
    params[0] = -3.212;
    params[1] = 0.07363;
    params[2] = -0.001012;
    params[3] = 8.015e-06;
    params[4] = -2.967e-08;
    params[5] = 4.1e-11 ;
    
    for( int i = 0; i < 256; ++i)
    {
        double g_derivative = 5*params[5]*i*i*i*i+
        4*params[4]*i*i*i+3*params[3]*i*i*+2*params[2]*i+params[1];
        g_derivative_table.at<double>(0, i) = g_derivative;
    }    
    steps.resize(6750);
    int step_ = 1;
    for (size_t i = 0; i < steps.size(); i++)
    {
        if ( i < 10)
            steps[i] = step_;
        else if (i < 20)
            steps[i] = step_*1;
        else if (i < 50)
            steps[i] = step_*3;
        else if (i < 100)
            steps[i] = step_*5;
        else if (i < 150)
            steps[i] = step_*7;
        else if (i < 333)
            steps[i] = step_*15;
        else if (i < 601)
            steps[i] = step_*30;
        else if (i < 901)
            steps[i] = step_*30;
        else if (i < 1251)
            steps[i] = step_*30;
        else if (i < 6750)
            steps[i] = step_*45;
        }
}

// void Lut_generate()
// {
//     for(int i = 0;i<13;i++)
//     {
//         uchar gamma_0[256];
//         double s = 0.5 + 0.125*i;
//         correct_vector_creat(gamma_0, s);
//         Mat lut(1, 256, CV_8UC1, gamma_0);
//         LUT_GAMMA.push_back(lut);
//     }
//     cout<<LUT_GAMMA[12]<<endl;
//     cout<<LUT_GAMMA[0]<<endl;

// }

vector<cv::Mat> splitImage(cv::Mat image, int num, int type){
    int rows = image.rows;
    int cols = image.cols;
    vector<cv::Mat> v;
    if (type == 0) {
        for (size_t i = 0; i < num; i++) {
            int star = rows / num * i;
            int end = rows / num * (i + 1);
            if (i == num - 1) {
                end = rows;
            }
            //cv::Mat b = image.rowRange(star, end);
            v.push_back(image.rowRange(star, end));
        }
    }
    else if (type == 1) {
        for (size_t i = 0; i < num; i++) {
            int star = cols / num * i;
            int end = cols / num * (i + 1);
            if (i == num - 1) {
                end = cols;
            }
            v.push_back(image.colRange(star, end).clone());
        }
    }
    return  v;
}

cv::Mat catImage(vector<cv::Mat> v, int type) {
    cv::Mat dest = v.at(0);
    for (size_t i = 1; i < v.size(); i++)
    {
        if (type == 0)//´¹Ö±Æ´½Ó
        {
            cv::vconcat(dest, v.at(i), dest);
        }
        else if (type == 1)//Ë®Æ½Æ´½Ó
        {
            cv::hconcat(dest, v.at(i), dest);
        }
    }
    return dest;
}

static int get_near_index(float value)
{
    int ret_index;                        
    int mid_index;                       
    int left_index;                       
    int right_index;                      
    float left_abs;                      
    float right_abs;                      

    ret_index = 0;
    left_index = 0;
    right_index = 256 - 1;
    mid_index = 0;
    left_abs = 0;
    right_abs = 0;

    while (left_index != right_index) {
        mid_index = (right_index + left_index) / 2;
        if (value <= g[mid_index]) {
            right_index = mid_index;
        }
        else {
            left_index = mid_index;
        }
        if (right_index - left_index < 2) {
            break;
        }
    }
    left_abs = fabs(g[left_index] - value);
    right_abs = fabs(g[right_index] - value);
    ret_index = right_abs <= left_abs ? right_index : left_index;
    return ret_index;
}

void * threadProcess(void* args) {

    pthread_t myid = pthread_self();
    paramThread *para = (paramThread *)args;
    int w = para->w;
    int h = para->h;
    cv::Mat image(h, w, CV_8UC1, (uchar *)para->data);
    thread_out out;
    out = calculate_m(image);
    *(para->quality) = out.quality;
    *(para->grad_x) = out.grad_x;
    *(para->grad_y) = out.grad_y;
    //cout << *(para->quality) << endl;
    pthread_exit(NULL);
    return NULL;
}

void correct_vector_creat(uchar* gamma_i, float num)
{
    float temp;
    int xx;
    for (int i = 0; i <= 255; i++)
    {
        temp = g[i] + log(num);
        if (temp >= 2.0842)
            temp = 2.0842;
        else if (temp <= -5.1898)
            temp = -5.1898;
        xx = get_near_index(temp);
        *(gamma_i + i) = xx;
    }
}

thread_out muti_thread(Mat gray_image)
{
    int type = 0;
    vector<cv::Mat> v = splitImage(gray_image, THREAD_NUMS, type);
    vector<double> q(THREAD_NUMS);
    Mat L(v[0].rows, v[0].cols, CV_64FC1, Scalar::all(0));
    Mat M(v[0].rows, v[0].cols, CV_64FC1, Scalar::all(0));
    vector<Mat> grad_xs(THREAD_NUMS,L);
    vector<Mat> grad_ys(THREAD_NUMS,L);
    paramThread args[THREAD_NUMS];
    pthread_t pt_base[THREAD_NUMS]; 
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        args[i].h = v.at(i).rows;
        args[i].w = v.at(i).cols;
        args[i].data = v.at(i).data;
        args[i].quality = &q[i];
        args[i].grad_x = &grad_xs[i];
        args[i].grad_y = &grad_ys[i];
        pthread_create(&pt_base[i], NULL, &threadProcess, (void *)(&args[i]));
    }
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        pthread_join(pt_base[i], NULL);
    }
    thread_out output;
    vector<Mat>grad_x_split,grad_y_split;
    for (size_t i = 0; i < THREAD_NUMS; i++)
    {
        output.quality = output.quality + q[i];
        grad_x_split.push_back(grad_xs[i]);
        grad_y_split.push_back(grad_ys[i]);
    }
    output.grad_x = catImage(grad_x_split,type);
    output.grad_y = catImage(grad_y_split,type);
    output.gray_image = gray_image;
    return output;
}


thread_out calculate_m(Mat gray_image)
{
    cv::Mat blurred_image, gradient_image;
    thread_out out;
    //cv::resize(gray1.85721e+07_image,gray_image,Size(640,480),0,0,INTER_LINEAR);
    //cv::GaussianBlur(gray_image, blurred_image, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
    cv::Mat grad_x, grad_y;
    cv::Sobel(gray_image, grad_x, CV_64FC1, 1, 0, 3, 1.0, 0, cv::BORDER_DEFAULT);
    cv::Sobel(gray_image, grad_y, CV_64FC1, 0, 1, 3, 1.0, 0, cv::BORDER_DEFAULT);
    magnitude(grad_x, grad_y, gradient_image);
    double sum_gradient = 0;
    double *p;
    uchar *q;
    int temp = 0;
    for (int i = 0; i < gradient_image.rows; i++)
    {
        p = gradient_image.ptr<double>(i);
        q = gray_image.ptr<uchar>(i);
        for (int j = 0; j < gradient_image.cols; j++)
        {
            temp = q[j];
            sum_gradient = sum_gradient + W[temp] * p[j];
        }
    }
    sum_gradient = sum_gradient;
    out.quality = sum_gradient;
    out.grad_x = grad_x;
    out.grad_y = grad_y;
    out.gray_image = gray_image;
    return out;
}

thread_out heuristic_search(Mat gray_image)
{
    Mat grad_x_opt,grad_y_opt,gray_image_opt;
    Mat estimation_left = gray_image;
    Mat estimation_right= gray_image;
    thread_out M_opt = muti_thread(gray_image);
    grad_x_opt = M_opt.grad_x;
    grad_y_opt = M_opt.grad_y;
    gray_image_opt = gray_image;

    double alfa = 0.5;
    double beta = 2;
    int n = 3;
    search_gamma = 1;
    int index = 0;
    while(n--)//3-layers
    {
        
        //cout<<gamma_parament[index]<<endl;
        cout<<"alfa    "<<alfa<<"    beta    "<<beta<<endl;
        bool search_flag = true;
        if(search_flag)
        {
            index = alfa*8-4;
            Mat lut_left = LUT_GAMMA[index].clone();
            LUT(M_opt.gray_image,lut_left,estimation_left);
            thread_out M_left = muti_thread(estimation_left);

            index = beta*8-4;
            Mat lut_right = LUT_GAMMA[index].clone();
            LUT(M_opt.gray_image,lut_right,estimation_right);
            thread_out M_right = muti_thread(estimation_right);

            if(M_left.quality > M_right.quality&&M_left.quality > 1.1*M_opt.quality)
            {
                M_opt = M_left;
                search_gamma = search_gamma*alfa;
                beta = 0.5*(beta+1);
            }
            else if(M_left.quality < M_right.quality&&M_right.quality > 1.1*M_opt.quality)
            {
                M_opt = M_right;
                search_gamma = search_gamma * beta;
                alfa = 0.5*(alfa+1);
            }
            else
            {
                alfa = 0.5*(alfa+1);
                beta = 0.5*(beta+1);            
            }
        }
        else
        {
            index = alfa*8-4;
            Mat lut_left = LUT_GAMMA[index].clone();
            LUT(M_opt.gray_image,lut_left,estimation_left);
            thread_out M_left = muti_thread(estimation_left);
            if(M_left.quality > 1.02*M_opt.quality)//layer_2
            {
                M_opt = M_left;
                search_gamma = search_gamma*alfa;
                beta = 0.5*(beta+1);
                continue;
            }
            index = beta*8-4;
            Mat lut_right = LUT_GAMMA[index].clone();
            LUT(M_opt.gray_image,lut_right,estimation_right);
            thread_out M_right = muti_thread(estimation_right);
            if(M_right.quality > 1.02*M_opt.quality)//layer_2
            {
                M_opt = M_right;
                search_gamma = search_gamma * beta;
                alfa = 0.5*(alfa+1);
                continue;
            }
            alfa = 0.5*(alfa+1);
            beta = 0.5*(beta+1);
        }
    }
    // imshow("33",M_opt.gray_image);
    // waitKey(1);
    return M_opt;
}
void exposure_adjust(Mat color_image, double& exposure_time, double& exposure_gain)
{
        Mat gray_image;
        double exposure_time_search,exposure_time_refine;
        cvtColor(color_image, gray_image, CV_BGR2GRAY);
        thread_out M_opt = heuristic_search(gray_image);
        // cout<<"suit search_gamma"<<" "<<search_gamma<<endl;
        double k = 0.1;
        exposure_time_search = exposure_time *(1 + k*(search_gamma - 1));
        exposure_time = exposure_time_search;
        Mat grad_opt_x = M_opt.grad_x; 
        Mat grad_opt_y = M_opt.grad_y;
        Mat gray_opt = M_opt.gray_image;
        cv::Mat g_derivative, grad_g_x, grad_g_y;

        cv::Mat temp;
        cv::LUT(gray_opt, g_derivative_table, temp);
        cv::divide(1 / exposure_time_search, temp, g_derivative);
        cv::Sobel(g_derivative, grad_g_x, CV_64FC1, 1, 0, 3, 1.0, 0, cv::BORDER_DEFAULT);
        cv::Sobel(g_derivative, grad_g_y, CV_64FC1, 0, 1, 3, 1.0, 0, cv::BORDER_DEFAULT);
        double sum_gradient = 0;
        double sum_weight = 0;
        double *p_x;
        double *p_y;
        double *pp_x;
        double *pp_y;
        uchar *q;
        int value = 0 ;
        for (int i = 0; i < grad_opt_x.rows; i++)
        {
            p_x = grad_opt_x.ptr<double>(i);
            p_y = grad_opt_y.ptr<double>(i);
            q = gray_opt.ptr<uchar>(i);
            pp_x = grad_g_x.ptr<double>(i);
            pp_y = grad_g_y.ptr<double>(i);
            for (int j = 0; j < grad_opt_x.cols; j++)
            {
                double gradient;
                value = 100;
                if((p_x[j]*p_x[j]+p_y[j]*p_y[j])>0.11*0.11)
                {
                    if (value > 225)
                    {
                        gradient = W[value]*(-2.0);
                    }else if (value < 30)
                    {
                        gradient = W[value]*2.0;
                    }else{
                        gradient = W[value] * 2 * (p_x[j] * pp_x[j] + p_y[j] * pp_y[j]);
                    }
                    sum_gradient += gradient;
                    sum_weight += W[value];
                }
            }
        }
        sum_weight = sum_weight;
        sum_gradient = sum_gradient/sum_weight;
        if (fabs(sum_gradient) < 2.1&&fabs(sum_gradient) > 0.5) // prevent camera gelam
        {
            exposure_time_refine = exposure_time_search + steps[exposure_time_search]*sum_gradient;
            exposure_time = std::round(exposure_time_refine);            
        }
        else
        {
            exposure_time = exposure_time_search;
        }
        if (exposure_time > 666)
        {
            exposure_gain = exposure_gain*exposure_time/666;
            exposure_time = 666;
        }
        if (exposure_time < 6)
        {
             exposure_gain = exposure_gain*exposure_time/6;
             exposure_time = 6;
         }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat colorImg = cv_ptr->image; 
    Mat temp;
    imshow("view",colorImg);  
    cv::waitKey(1);
    //GaussianBlur(colorImg,colorImg,Size(3,3),0);
    exposure_adjust(colorImg, exposure_time, exposure_gain);
    std_msgs::Int32MultiArray array;
    array.data.clear();
    array.data.push_back(exposure_time);
    array.data.push_back(exposure_gain);
    // cout<<exposure_time<<"  "<<exposure_gain<<endl;
    chatter_pub.publish(array);
    //ROS_INFO("I publish");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}