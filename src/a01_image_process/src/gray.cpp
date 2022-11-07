#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char **argv)
{
    cv::Mat image, gray;
    image = cv::imread("/home/ty/Desktop/genetic_recombination_scoutx/src/a01_image_process/images/lab.jpeg");
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::namedWindow("input", 0);
    cv::resizeWindow("input", 600, 400);
    cv::imshow("input", image);

    cv::namedWindow("output", 0);
    cv::resizeWindow("output", 600, 400);
    cv::imshow("output", gray);

    cv::waitKey();

    return 0;
}