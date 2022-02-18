/**
 * @file KLT_EXPOSURE.cpp
 * @author Yu Wang(19B953030@stu.hit.edu.cn); Haoyao Chen(hychen5@hit.edu.cn); Shiwu Zhang; Wencan Lu. 
 * @email hychen5@hit.edu.cn
 * @date 2022-02-18
 * 
 * @copyright Copyright (c) 2022, nROS-Lab, HITsz
 * 
 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream> 
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace cv;
using namespace std;
using namespace	Eigen;

int col;
int raw;
bool inBorder(const cv::Point2f &pt)
{
	const int BORDER_SIZE = 1;
	int img_x = cvRound(pt.x);
	int img_y = cvRound(pt.y);
	return BORDER_SIZE <= img_x && img_x < raw - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < col - BORDER_SIZE;
}
double distance(cv::Point2f pt1, cv::Point2f pt2)
{
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	return sqrt(dx * dx + dy * dy);
}

int find_feature_matches(Mat img_1, Mat img_2)
{
	std::vector<cv::Point2f> old_pts; 
	std::vector<cv::Point2f> new_pts;

	cv::goodFeaturesToTrack(img_1, old_pts, 300, 0.01, 30);
	std::vector<uchar> status;
	std::vector<float> err;
	cv::calcOpticalFlowPyrLK(img_1, img_2, old_pts, new_pts, status, err, cv::Size(21, 21), 4);
	int track = 0;
	if (1)
	{
		vector<uchar> reverse_status;
		vector<cv::Point2f> reverse_pts = old_pts;
		cv::calcOpticalFlowPyrLK(img_2, img_1, new_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
		for (size_t i = 0; i < status.size(); i++)
		{
			if (status[i] && reverse_status[i] && distance(old_pts[i], reverse_pts[i]) <= 0.5)
			{
				status[i] = 1;
			}
			else
				status[i] = 0;
		}
	}

	for (int i = 0; i < int(new_pts.size()); i++)
		if (status[i] && !inBorder(new_pts[i]))
			status[i] = 0;

	for (size_t i = 0; i < new_pts.size(); i++)
	{
		if (status[i])
			track++;
	}
	return track;
}

int main()
{
	Mat srcGrayImage;

	Mat Shim_left = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/left/Shim.jpg");
	Mat Zhang_left = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/left/Zhang.jpg");
	Mat Kim_left = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/left/Kim.jpg");
	Mat Shin_left = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/left/Shin.jpg");
	Mat Our_left = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/left/Our.jpg");

	Mat Shim_right = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/right/Shim.jpg");
	Mat Zhang_right = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/right/Zhang.jpg");
	Mat Kim_right = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/right/Kim.jpg");
	Mat Shin_right = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/right/Shin.jpg");
	Mat Our_right = imread("../../Result_Exp_AR_ver/outdoor_weed2_3010Lux/right/Our.jpg");



	cvtColor(Kim_left, Kim_left, CV_BGR2GRAY);
	medianBlur(Kim_left, Kim_left, 3);
	cvtColor(Kim_right, Kim_right, CV_BGR2GRAY);
	medianBlur(Kim_right, Kim_right, 3);

	cvtColor(Zhang_left, Zhang_left, CV_BGR2GRAY);
	medianBlur(Zhang_left, Zhang_left, 3);
	cvtColor(Zhang_right, Zhang_right, CV_BGR2GRAY);
	medianBlur(Zhang_right, Zhang_right, 3);

	cvtColor(Our_left, Our_left, CV_BGR2GRAY);
	medianBlur(Our_left, Our_left, 3);
	cvtColor(Our_right, Our_right, CV_BGR2GRAY);
	medianBlur(Our_right, Our_right, 3);

	cvtColor(Shim_left, Shim_left, CV_BGR2GRAY);
	medianBlur(Shim_left, Shim_left, 3);
	cvtColor(Shim_right, Shim_right, CV_BGR2GRAY);
	medianBlur(Shim_right, Shim_right, 3);

	cvtColor(Shin_left, Shin_left, CV_BGR2GRAY);
	medianBlur(Shin_left, Shin_left, 3);
	cvtColor(Shin_right, Shin_right, CV_BGR2GRAY);
	medianBlur(Shin_right, Shin_right, 3);

	col = Shin_right.cols;
	raw = Shin_right.rows;

	int Shim_Match = find_feature_matches(Shim_left, Shim_right);
	cout << "Shim_Match " << Shim_Match << " pairs of feature points" << endl;

	int Zhang_Match = find_feature_matches(Zhang_left, Zhang_right);
	cout << "Zhang_Match " << Zhang_Match << "  pairs of feature points" << endl;

	int Kim_Match = find_feature_matches(Kim_left, Kim_right);
	cout << "Kim_Match " << Kim_Match << " pairs of feature points" << endl;

	int Shin_Match = find_feature_matches(Shin_left, Shin_right);
	cout << "Shin_Match " << Shin_Match << " pairs of feature points" << endl;

	int Our_Match = find_feature_matches(Our_left, Our_right);
	cout << "Our_Match " << Our_Match << " pairs of feature points" << endl;


	return 0;
}
