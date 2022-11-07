#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;

int main(int argc, char **argv)
{
    cv::Mat image, output;
    image = cv::imread("/home/ty/Desktop/genetic_recombination_scoutx/src/a01_image_process/images/lab.jpeg");

    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(60);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(image, keypoints);
    cv::drawKeypoints(image, keypoints, output);

    cv::namedWindow("input", 0);
    cv::resizeWindow("input", 600, 400);
    cv::imshow("input", image);

    cv::namedWindow("output", 0);
    cv::resizeWindow("output", 600, 400);
    cv::imshow("output", output);

    cv::waitKey();

    return 0;
}