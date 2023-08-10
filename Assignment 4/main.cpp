#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
const int point_num = 4;


void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < point_num) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}


void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    int point_num = control_points.size();
    std::vector<cv::Point2f> next_control_point;
    next_control_point.reserve(point_num - 1);
    for (int i=0; i<point_num-1; i++){
        float x = t * control_points[i].x + (1 - t) * control_points[i+1].x;
        float y = t * control_points[i].y + (1 - t) * control_points[i+1].y;
        next_control_point.emplace_back(x, y);
    }
    if (point_num == 2) 
        return next_control_point[0];
    else
        return recursive_bezier(next_control_point, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    float t = 0, delta = 0.00001;
    while (t < 1){
        t += delta;
        cv::Point2f tmp_point = recursive_bezier(control_points, t);
        //window.at<cv::Vec3b>(tmp_point.y, tmp_point.x)[1] = 255;
        for (int i=tmp_point.y-3; i<tmp_point.y+3; i++)
            for (int j=tmp_point.x-3; j<tmp_point.x+3; j++){
                float dist = std::sqrt(std::pow(i-tmp_point.y, 2) + std::pow(j-tmp_point.x, 2));
                float k = std::exp(-dist) * 2;
                window.at<cv::Vec3b>(i, j)[1] = std::max((int)window.at<cv::Vec3b>(i, j)[1], (int)(255 * k));
            }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == point_num) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
