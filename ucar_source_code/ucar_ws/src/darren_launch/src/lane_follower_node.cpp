#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <image_transport/image_transport.h>
#include <iostream>
#include <vector>
#include <queue>

const int HISTORY_LENGTH = 5;
// 全局变量
int consecutive_right_turn_count = 0;
int consecutive_left_turn_count = 0;

const int TURN_COUNT_THRESHOLD = 2; // 连续检测到拐点的次数阈值

class LaneFollower {
public:
    LaneFollower()
        : frame_center_(320), Kp_(0.0055), Kd_(0.00015), angular_turn_msg_(1.57), rate_(10), max_control_signal_(1.0), prev_error_(0.0),window_size(2),slope_threshold(1),threshold(10),follow_lane_flag_(false), line_detected_(false), turn_right_(false), turn_left_(false) {
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_sub_ = it.subscribe("/usb_cam/image_raw", 1, &LaneFollower::imageCallback, this);
        processed_image_pub_ = it.advertise("/processed_image", 1);
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        control_sub_ = nh.subscribe("/start_following", 1, &LaneFollower::controlCallback, this);
        task_completed_pub_ = nh.advertise<std_msgs::Bool>("/task_completed", 1);
        binary_image_pub = it.advertise("binary_image", 1);
    }

    void run() {
        ros::spin();
    }

private:
    image_transport::Subscriber image_sub_;
    image_transport::Publisher processed_image_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber control_sub_;
    ros::Publisher task_completed_pub_;
    image_transport::Publisher binary_image_pub ;

    int frame_center_;
    double Kp_;
    double Kd_;
    double max_control_signal_;
    double prev_error_;
    bool follow_lane_flag_;
    bool line_detected_;
    bool turn_left_;
    bool turn_right_;
    double angular_turn_msg_;
    int rate_;
    int window_size; //= 10; // 窗口大小
    double slope_threshold;
    int threshold;// = 1; // 斜率阈值
    // 历史数据结构
    std::deque<int> left_history;
    std::deque<int> right_history; 

    enum State { FOLLOWING_LANE, DETECTING_TURN_RIGHT,DETECTING_TURN_LEFT,DETECTING_TURN_CIRCLE };
    State current_state_ = FOLLOWING_LANE;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (follow_lane_flag_) {
            followLane(msg);
        }
    }

    void followLane(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            if (msg->encoding == "bgr8") {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } else if (msg->encoding == "rgb8") {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
            } else {
                ROS_ERROR("Unsupported encoding: %s", msg->encoding.c_str());
                return;
            }
            cv::flip(cv_ptr->image, cv_ptr->image, 1);
        } catch (std::exception& e) {
            ROS_ERROR("CvBridge Error: %s", e.what());
            return;
        }

        int height = cv_ptr->image.rows;
        int width = cv_ptr->image.cols;
        std::vector<cv::Point> mid_points, left_points, right_points;
        cv::Mat img_with_lines;
        processImage(cv_ptr->image, mid_points, left_points, right_points, img_with_lines);

        // // 发布处理后的图像
        // cv_bridge::CvImage processed_img_msg;
        // processed_img_msg.header = msg->header;
        // processed_img_msg.encoding = sensor_msgs::image_encodings::BGR8;
        // processed_img_msg.image = img_with_lines;
        // processed_image_pub_.publish(processed_img_msg.toImageMsg());

        switch (current_state_) {
            case FOLLOWING_LANE:
                handleLaneFollowing(mid_points, left_points, right_points);
                break;
            case DETECTING_TURN_RIGHT:
                handleTurnRIGHT(mid_points, left_points, right_points);
                break;
            case DETECTING_TURN_LEFT:
                handleTurnLEFT(mid_points, left_points, right_points);
                break;
            case DETECTING_TURN_CIRCLE:
                handleTurnCIRCLE(mid_points, left_points, right_points);
                break;
        }
    }

    

    void processImage(const cv::Mat& img, std::vector<cv::Point>& mid_points, std::vector<cv::Point>& left_points, std::vector<cv::Point>& right_points, cv::Mat& img_with_lines) {
        int height = img.rows;
        int width = img.cols;

        // 调整 lower_half 的矩形区域，从图像底部的 150 行到 20 行
        int start_row = height - 120;  //90  //70
        int end_row = height - 50;   //40   //50
        int num_rows = end_row - start_row;

        cv::Mat lower_half = img(cv::Rect(0, start_row, width, num_rows));

        cv::Mat gray;
        cv::cvtColor(lower_half, gray, cv::COLOR_BGR2GRAY);
        cv::Mat binary;
        cv::threshold(gray, binary, 185, 255, cv::THRESH_BINARY);

        // 发布二值化图像
        sensor_msgs::ImagePtr binary_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", binary).toImageMsg();
        binary_image_pub.publish(binary_msg);

        img_with_lines = img.clone();  // 用于绘制线条的图像
        int crossing_line_count = 0;
     // 遍历图像底部到顶部的每一行
        for (int i = binary.rows - 1; i >= 0; --i) {
            cv::Mat row = binary.row(i);
            int mid_index = binary.cols / 2;

             // 在左半边和右半边寻找非零像素点，确定左右边界点
            std::vector<cv::Point> left_indices, right_indices;
            cv::findNonZero(row(cv::Rect(0, 0, mid_index, 1)), left_indices);
             cv::findNonZero(row(cv::Rect(mid_index, 0, binary.cols - mid_index, 1)), right_indices);

            // 左边界点
            cv::Point left_point;
            if (!left_indices.empty()) {
                left_point = cv::Point(left_indices.back().x, i + start_row);
            } else {
                left_point = cv::Point(0, i + start_row);
            }
            left_points.push_back(left_point);

            // 右边界点
            cv::Point right_point;
            if (!right_indices.empty()) {
                right_point = cv::Point(right_indices.front().x + mid_index, i + start_row);
            } else {
                right_point = cv::Point(binary.cols - 1, i + start_row);
            }
            right_points.push_back(right_point);

            // // 检测第50到55行的跨线情况44-49
            if (i >= 64 && i <= 79 && cv::sum(row)[0] > (0.40 * width * 255)) {
                crossing_line_count++;
            }
            // if (cv::sum(row)[0] > (0.60 * width * 255)) {s
            //     ROS_WARN("Crossing line detected.");
            //     line_detected_ = true; //0.4
            // }

            // // 检测到足够的左右边界点后退出循环
            // if (!left_indices.empty() && !right_indices.empty()) {
            //     break;
            // }
        }
        // for (int i = binary.rows - 1; i >= binary.rows - 6 ; --i) {
        //     cv::Mat row = binary.row(i);
        //     // 检测第50到55行的跨线情况44-49
        //     if ( cv::sum(row)[0] > (0.40 * width * 255)) {
        //         crossing_line_count++;
        //     }
        // }
         

        // std::cout << "Left Boundary Points:" << std::endl;
        // for (const auto& point : left_points) {
        //     std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
        // }

        // // 打印右边界点集
        // std::cout << "Right Boundary Points:" << std::endl;
        // for (const auto& point : right_points) {
        //     std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
        // }
        // 如果在第50到55行中多次检测到跨线情况，则认为跨线
        if (crossing_line_count > 2) {
            ROS_WARN("Crossing line detected.");
            line_detected_ = true;
        }

        // 计算中线点，即左右边界点的中点
        for (size_t i = 0; i < left_points.size(); ++i) {
            mid_points.push_back((left_points[i] + right_points[i]) / 2);
        }

    //     // 在图像上绘制检测到的点
    //  for (const auto& pt : left_points) {
    //         cv::circle(img_with_lines, pt, 5, cv::Scalar(0, 255, 0), -1);  // 绿色圆点，左边界点
    //  }
    //  for (const auto& pt : right_points) {
    //      cv::circle(img_with_lines, pt, 5, cv::Scalar(0, 0, 255), -1);  // 红色圆点，右边界点
    //  }
    //  for (const auto& pt : mid_points) {
    //         cv::circle(img_with_lines, pt, 5, cv::Scalar(255, 0, 0), -1);  // 蓝色圆点，中线点
    //   }


    }


    bool isIncreasing(const std::vector<cv::Point>& points) {
        if (points.size() < 2) {
        return true;  // 少于2个元素的数组可以视为递增
        }

        int non_increasing_count = 0;

        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].x <= points[i - 1].x) {
                non_increasing_count++;
                if (non_increasing_count > 2) {
                    return false;  // 超过允许的最大非递增对数
                }
            }
        }

        return true;  // 如果数组中的元素在允许的容差内均满足递增条件
    }


    void handleLaneFollowing( std::vector<cv::Point>& mid_points,  std::vector<cv::Point>& left_points,  std::vector<cv::Point>& right_points) {
        if (!mid_points.empty()) {
            int chosen_mid_point = chooseMidPoint(mid_points, "average");
            int deviation = frame_center_ - chosen_mid_point;

            ROS_INFO("Frame Center: %d, Lane Center: %d, Deviation: %d", frame_center_, chosen_mid_point, deviation);

            double control_signal = controlCar(deviation);
            ROS_INFO("Control Signal: %f", control_signal);

            publishControl(control_signal);
           // bool turn_detected = detectTurn(left_points, right_points,threshold);
            // bool turn_left_=detectLeftTurn(left_points,threshold);
            // bool turn_right_=detectRightTurn(right_points,threshold);
                    
            // if (turn_detected)
            // {
            //ROS_INFO("turn_detected");
            // if (turn_left_ && turn_right_&&line_detected_ )
            // {
            //         stopFollowing();
            //         std_msgs::Bool task_completed_msg;
            //         task_completed_msg.data = true;
            //         task_completed_pub_.publish(task_completed_msg);
            //         ROS_INFO("Task completed.");
            // }
                
            // if ( turn_left_ &&!isIncreasing(right_points))
            // {
            //         current_state_ = DETECTING_TURN_LEFT;
            //         turn_right_=false;
            //         ROS_INFO("Switching to DETECTING_TURN state (Left Boundary).");
            // }
            // else if(turn_right_&&!isIncreasing(left_points)){
            //         current_state_ = DETECTING_TURN_RIGHT;
            //         turn_left_=false;
            //         ROS_INFO("Switching to DETECTING_TURN state (Right Boundary).");
            // }
                
           // }

           //-----------------------------------
            if (detectRightTurn(right_points) &&!isArc(left_points)) {
                ROS_INFO("Executing right turn.");
                current_state_ = DETECTING_TURN_RIGHT;
                turn_left_=false;
                ROS_INFO("Switching to DETECTING_TURN state (Right Boundary).");
                right_points.clear();  // 清除右边点集
                left_points.clear();  // 清除左边点集
                // 在此处添加右转的相关代码
            }

            // if (detectLeftTurn(left_points)&&!isArc(right_points)) {
            //     ROS_INFO("Executing left turn.");
            //     current_state_ = DETECTING_TURN_LEFT;
            //     turn_right_=false;
            //     ROS_INFO("Switching to DETECTING_TURN state (Left Boundary).");
            //     right_points.clear();  // 清除右边点集
            //     left_points.clear();  // 清除左边点集
          
            // // 在此处添加左转的相关代码
            // }
            
            
            // if (isBoundaryLine(left_points) && !isBoundaryLine(right_points)) {
            //     turn_left_ = true;
            //     current_state_ = DETECTING_TURN_LEFT;
            //     ROS_INFO("Switching to DETECTING_TURN state (Left Boundary).");
            // } 
            // if (isBoundaryLine(right_points) && isBoundaryLine(left_points)) {
            //     turn_right_ = true;
            //     current_state_ = DETECTING_TURN_RIGHT;
            //     ROS_INFO("Switching to DETECTING_TURN state (Right Boundary).");
            // }
            // if (!isBoundaryLine(left_points) ||!isBoundaryLine(right_points)) {
            //     if (isArc(left_points)||isArc(right_points)) {
                    
            //         current_state_ = DETECTING_TURN_CIRCLE;
            //         ROS_INFO("Switching to DETECTING_TURN state (CIRCLE).");
            //     } 
            // }
            if (isBoundaryLine(left_points) && isBoundaryLine(right_points)) {
                ROS_WARN("Both left and right lines are boundary lines. Stopping the vehicle.");
                stopFollowing();
                std_msgs::Bool task_completed_msg;
                task_completed_msg.data = true;
                task_completed_pub_.publish(task_completed_msg);
                ROS_INFO("Task completed message published.");
            }
           if (line_detected_) {
                ROS_WARN("Crossing line detected. Stopping the vehicle.");
                stopFollowing();
                std_msgs::Bool task_completed_msg;
                task_completed_msg.data = true;
                task_completed_pub_.publish(task_completed_msg);
                return;

           }
            
        } else {
            ROS_WARN("No mid points detected. Skipping control.");
        }
    }

    void handleTurnLEFT(const std::vector<cv::Point>& mid_points, const std::vector<cv::Point>& left_points, const std::vector<cv::Point>& right_points) {
        if (line_detected_) {
            ROS_INFO("Crossing line detected. Preparing to turn.");
            stopFollowing();
            follow_lane_flag_ = true;
            line_detected_= false;
            //current_state_ = PERFORMING_TURN; 
            rotate(-95); // Left turn
            ros::Duration(0.5).sleep();  // 休眠1秒
            ROS_INFO("结束休眠");
            turn_left_=false;
            ROS_INFO("Crossing line detected. left_turned_90.");
            current_state_ = FOLLOWING_LANE;
            ROS_INFO("Switching to FOLLOWING_LANE state ");
        } else {
            //publishForwardMotion();
            if (!mid_points.empty()) {
            int chosen_mid_point = chooseMidPoint(mid_points, "average");
            int deviation = frame_center_ - chosen_mid_point;

            //ROS_INFO("Frame Center: %d, Lane Center: %d, Deviation: %d", frame_center_, chosen_mid_point, deviation);

            double control_signal = controlCar(deviation);
            ROS_INFO("Control Signal: %f", control_signal);

            publishControl(control_signal);
        //    if (!isBoundaryLine(left_points) && !isBoundaryLine(right_points))
        //     {
        //         current_state_ = FOLLOWING_LANE;
        //         ROS_INFO("Switching to FOLLOWING_LANE state ");
        //     }
           
            }

        }
    }

    void handleTurnRIGHT(const std::vector<cv::Point>& mid_points, const std::vector<cv::Point>& left_points, const std::vector<cv::Point>& right_points) {
        if (line_detected_) {
            ROS_INFO("Crossing line detected. Preparing to turn.");
            stopFollowing();
            follow_lane_flag_ = true;
            line_detected_= false;
            rotate(95);  // Right turns
            ros::Duration(0.5).sleep();  // 休眠1秒
            ROS_INFO("结束休眠");
            turn_right_=false;
            ROS_INFO("Crossing line detected. right_turned_90.");
            current_state_ = FOLLOWING_LANE;
            ROS_INFO("Switching to FOLLOWING_LANE state ");
        } else {
            //publishForwardMotion();
            if (!mid_points.empty()) {
            int chosen_mid_point = chooseMidPoint(mid_points, "average");
            int deviation = frame_center_ - chosen_mid_point;

            //ROS_INFO("Frame Center: %d, Lane Center: %d, Deviation: %d", frame_center_, chosen_mid_point, deviation);

            double control_signal = controlCar(deviation);
            ROS_INFO("Control Signal: %f", control_signal);

            publishControl(control_signal);
        //    if (!isBoundaryLine(left_points) && !isBoundaryLine(right_points))
        //     {
        //         current_state_ = FOLLOWING_LANE;
        //         ROS_INFO("Switching to FOLLOWING_LANE state ");
        //     }
            
            
            }

        }
    }

    void handleTurnCIRCLE(const std::vector<cv::Point>& mid_points, const std::vector<cv::Point>& left_points, const std::vector<cv::Point>& right_points) {
        if (!isArc(left_points)&&!isArc(right_points)) {
            ROS_INFO("CIRCLE_COMPLETED.");
            current_state_ = FOLLOWING_LANE;
            ROS_INFO("Switching to FOLLOWING_LANE state ");
        } else {
            //publishForwardMotion();
            if (!mid_points.empty()) {
            int chosen_mid_point = chooseMidPoint(mid_points, "average");
            int deviation = frame_center_ - chosen_mid_point;

            //ROS_INFO("Frame Center: %d, Lane Center: %d, Deviation: %d", frame_center_, chosen_mid_point, deviation);

            double control_signal = controlCar(deviation);
            ROS_INFO("Control Signal: %f", control_signal);

            publishControl(control_signal);
            // if (!isBoundaryLine(left_points) && !isBoundaryLine(right_points)) {
                
            //     current_state_ = FOLLOWING_LANE;
            //     ROS_INFO("Switching to FOLLOWING_LANE state 1111");
            // } 
            }

        }
    }
    // bool isBoundaryLine(const std::vector<cv::Point>& points) {
    //     if (points.empty()) {
    //         return false;
    //     }
    //     return std::all_of(points.begin(), points.end(), [](const cv::Point& pt) {
    //         return pt.x == 0 || pt.x == 639;
    //     });
    // }

    bool isArc(const std::vector<cv::Point>& points) {
        // 确保点数足够可靠地检测
        if (points.size() < 20) {
            return false;
        }

        // 计算质心（平均 x 和 y 值）
        double mean_x = 0.0, mean_y = 0.0;
        for (const auto& pt : points) {
            mean_x += pt.x;
            mean_y += pt.y;
        }
        mean_x /= points.size();
        mean_y /= points.size();

        // 计算协方差矩阵的元素
        double sum_x2 = 0.0, sum_y2 = 0.0, sum_xy = 0.0;
        for (const auto& pt : points) {
            double dx = pt.x - mean_x;
            double dy = pt.y - mean_y;
            sum_x2 += dx * dx;
            sum_y2 += dy * dy;
            sum_xy += dx * dy;
        }

        // 计算协方差矩阵的特征值
        double lambda1 = (sum_x2 + sum_y2 + std::sqrt((sum_x2 + sum_y2) * (sum_x2 + sum_y2) - 4 * (sum_x2 * sum_y2 - sum_xy * sum_xy))) / 2;
        double lambda2 = (sum_x2 + sum_y2 - std::sqrt((sum_x2 + sum_y2) * (sum_x2 + sum_y2) - 4 * (sum_x2 * sum_y2 - sum_xy * sum_xy))) / 2;

        // 计算曲率
        double curvature = std::abs(lambda1 - lambda2) / (lambda1 + lambda2);

        // 设置曲率阈值，以确定是否是弧线
        double curvature_threshold = 1;  // 根据需要调整阈值

        std::cout << "Curvature圆弧: " << curvature << std::endl;

        if (curvature > curvature_threshold && curvature<10)
        {
            return true;
        }
        else{
             return false;
        }

       
    }

    bool isStraightLine(const std::vector<cv::Point>& points) {
        // 确保点数足够可靠地检测
        if (points.size() < 20) {
            return true;
        }

        // 计算质心（平均 x 和 y 值）
        double mean_x = 0.0, mean_y = 0.0;
        for (const auto& pt : points) {
            mean_x += pt.x;
            mean_y += pt.y;
        }
        mean_x /= points.size();
        mean_y /= points.size();

        // 计算线性回归所需的和
        double sum_xy = 0.0, sum_x_squared = 0.0;
        for (const auto& pt : points) {
            sum_xy += (pt.x - mean_x) * (pt.y - mean_y);
            sum_x_squared += (pt.x - mean_x) * (pt.x - mean_x);
        }

        // 避免除以零的情况
        if (std::abs(sum_x_squared) < 1e-6) {
            return false;
        }

        // 计算回归线的斜率（m）和截距（b）
        double m = sum_xy / sum_x_squared;
        double b = mean_y - m * mean_x;

        // 计算残差（每个点到回归线的距离）
        double total_residual = 0.0;
        for (const auto& pt : points) {
            double predicted_y = m * pt.x + b;
            double residual = std::abs(pt.y - predicted_y);
            total_residual += residual;
        }

        // 计算平均残差（按点数归一化）
        double avg_residual = total_residual / points.size();

        // 设置残差阈值，以确定是否是直线
        double threshold = 0.4;  // 根据需要调整阈值
        std::cout << "avg_residual: " << avg_residual << std::endl;
        if(avg_residual >threshold){
            return false;
        }
        else{
            return true;//avg_residual < threshold;
        }
        
    }

double calculateCurvature(const std::vector<cv::Point>& points, int index) {
    if (index <= 0 || index >= points.size() - 1) {
        return 0;
    }

    cv::Point prev = points[index - 1];
    cv::Point curr = points[index];
    cv::Point next = points[index + 1];

    double dx1 = curr.x - prev.x;
    double dy1 = curr.y - prev.y;
    double dx2 = next.x - curr.x;
    double dy2 = next.y - curr.y;

    double curvature = std::abs(dx1 * dy2 - dy1 * dx2) / std::pow(dx1 * dx1 + dy1 * dy1, 1.5);
    // 过滤异常大的曲率值
    if (curvature > 20) {
        curvature = 0.1;
    }

    return curvature;
}

bool detectRightTurn(const std::vector<cv::Point>& right_points) {
    if (right_points.size() < 3) {
        std::cout << "Insufficient points to detect a right turn." << std::endl;
        return false;
    }

    int minIndexRight = -1;
    int minXRight = std::numeric_limits<int>::max();

    for (size_t i = 0; i < right_points.size(); ++i) {
        if (right_points[i].x < minXRight) {
            minXRight = right_points[i].x;
            minIndexRight = i;
        }
    }

    if (minIndexRight != -1 && minIndexRight > 1 && minIndexRight < right_points.size() - 3) {
        double slope_prev = static_cast<double>(right_points[minIndexRight].y - right_points[minIndexRight - 3].y) /
                            (right_points[minIndexRight].x - right_points[minIndexRight - 3].x);
        double slope_next = static_cast<double>(right_points[minIndexRight + 3].y - right_points[minIndexRight].y) /
                            (right_points[minIndexRight + 3].x - right_points[minIndexRight].x);
        double curvature_prev = std::abs(slope_prev);
        double curvature_next = std::abs(slope_next);

        ROS_INFO("Curvature_prev R: %f", curvature_prev);
        ROS_INFO("Curvature_next R: %f", curvature_next);

        if (std::abs(curvature_prev - curvature_next) >= 0.1 && std::abs(curvature_prev - curvature_next) < 10) {
            ROS_INFO("Curvature difference R: %f", std::abs(curvature_prev - curvature_next));
            consecutive_right_turn_count++;
            std::cout << "Right turn detected at index " << minIndexRight << ". Turn count: " << consecutive_right_turn_count << std::endl;

            if (consecutive_right_turn_count >= TURN_COUNT_THRESHOLD) {
                std::cout << "Right turn confirmed." << std::endl;
                consecutive_right_turn_count = 0;
                return true;
            }
        } else {
            consecutive_right_turn_count = 0;
        }
    }

    return false;
}

bool detectLeftTurn(const std::vector<cv::Point>& left_points) {
    if (left_points.size() < 3) {
        std::cout << "Insufficient points to detect a left turn." << std::endl;
        return false;
    }

    int maxIndexLeft = -1;
    int maxXLeft = std::numeric_limits<int>::min();

    for (size_t i = 0; i < left_points.size(); ++i) {
        if (left_points[i].x > maxXLeft) {
            maxXLeft = left_points[i].x;
            maxIndexLeft = i;
        }
    }

    if (maxIndexLeft != -1 && maxIndexLeft > 1 && maxIndexLeft < left_points.size() - 3) {
        double slope_prev = static_cast<double>(left_points[maxIndexLeft].y - left_points[maxIndexLeft - 3].y) /
                            (left_points[maxIndexLeft].x - left_points[maxIndexLeft - 3].x);
        double slope_next = static_cast<double>(left_points[maxIndexLeft + 3].y - left_points[maxIndexLeft].y) /
                            (left_points[maxIndexLeft + 3].x - left_points[maxIndexLeft].x);
        double curvature_prev = std::abs(slope_prev);
        double curvature_next = std::abs(slope_next);

        ROS_INFO("Curvature_prev L: %f", curvature_prev);
        ROS_INFO("Curvature_next L: %f", curvature_next);

        if (std::abs(curvature_prev - curvature_next) >= 0.1 && std::abs(curvature_prev - curvature_next) < 10) {
            ROS_INFO("Curvature difference L: %f", std::abs(curvature_prev - curvature_next));
            consecutive_left_turn_count++;
            std::cout << "Left turn detected at index " << maxIndexLeft << ". Turn count: " << consecutive_left_turn_count << std::endl;

            if (consecutive_left_turn_count >= TURN_COUNT_THRESHOLD) {
                std::cout << "Left turn confirmed." << std::endl;
                consecutive_left_turn_count = 0;
                return true;
            }
        } else {
            consecutive_left_turn_count = 0;
        }
    }

    return false;
}


    // // 检测左右边界的拐点
    // bool detectTurn(const std::vector<cv::Point>& left_points, const std::vector<cv::Point>& right_points, int window_size, double slope_threshold) {
    //     // 检测左边界的拐点
    //     if (left_points.size() >= 3) {
    //         for (size_t i = 1; i < left_points.size() - 1; ++i) {
    //             // 计算相邻点的斜率
    //             double slope1 = static_cast<double>(left_points[i].y - left_points[i - 1].y) /
    //                         (left_points[i].x - left_points[i - 1].x);
    //             double slope2 = static_cast<double>(left_points[i + 1].y - left_points[i].y) /
    //                         (left_points[i + 1].x - left_points[i].x);

    //             // 检查斜率的变化是否大于阈值
    //             if (std::abs(slope2 - slope1) > slope_threshold) {
    //                 ROS_INFO("std::abs(slope2 - slope1): %f",std::abs(slope2 - slope1));
    //                 turn_left_ = true; // 这里的 turn_left_ 变量需要在类中定义并初始化
    //                 return true;
    //             }
    //         }
    //     }

    //     // 检测右边界的拐点
    //     if (right_points.size() >= 3) {
    //         for (size_t i = 1; i < right_points.size() - 1; ++i) {
    //             // 计算相邻点的斜率
    //             double slope1 = static_cast<double>(right_points[i].y - right_points[i - 1].y) /
    //                         (right_points[i].x - right_points[i - 1].x);
    //             double slope2 = static_cast<double>(right_points[i + 1].y - right_points[i].y) /
    //                         (right_points[i + 1].x - right_points[i].x);

    //             // 检查斜率的变化是否大于阈值
    //             if (std::abs(slope2 - slope1) > slope_threshold) {
    //                 ROS_INFO("std::abs(slope2 - slope1): %f", std::abs(slope2 - slope1));
    //                 turn_right_ = true; // 这里的 turn_right_ 变量需要在类中定义并初始化

    //                 return true;
    //             }
    //         }
    //     }

    //     return false;
    // }

    // // 函数：检测拐点
    // bool detectTurn(const std::vector<cv::Point>& left_points, const std::vector<cv::Point>& right_points) {
    //     // 更新历史数据
    //     if (left_history.size() >= HISTORY_LENGTH) {
    //         left_history.pop_front(); // 移除最旧的数据点
    //     }
    //     if (right_history.size() >= HISTORY_LENGTH) {
    //         right_history.pop_front(); // 移除最旧的数据点
    //     }
    //     left_history.push_back(left_points.back().x); // 添加最新的数据点的 x 坐标
    //     right_history.push_back(right_points.back().x); // 添加最新的数据点的 x 坐标

    //     // 计算左边界线的平均值
    //     int sum_left = 0;
    //     for (int value : left_history) {
    //         sum_left += value;
    //     }
    //     float avg_left = static_cast<float>(sum_left) / left_history.size();

    //     // 计算右边界线的平均值
    //     int sum_right = 0;
    //     for (int value : right_history) {
    //         sum_right += value;
    //     }
    //     float avg_right = static_cast<float>(sum_right) / right_history.size();

    //     // 检测拐点
    //     bool detected = false;
    //     if (left_history.size() >= 2) {
    //         float last_avg_left = static_cast<float>(left_history.back() + left_history.front()) / 2;
    //         if (std::abs(last_avg_left - avg_left) > 30) { // 根据实际情况调整阈值
    //             std::cout << "Detected left turn point." << std::endl;
    //             ROS_INFO("last_avg_left: %f",std::abs(last_avg_left - avg_left));
    //             turn_left_ = true; // 这里的 turn_left_ 变量需要在类中定义并初始化
    //             detected = true;
    //         }
    //     }
    //     if (right_history.size() >= 2) {
    //         float last_avg_right = static_cast<float>(right_history.back() + right_history.front()) / 2;
    //         if (std::abs(last_avg_right - avg_right) > 30) { // 根据实际情况调整阈值
    //             std::cout << "Detected right turn point." << std::endl;
    //             ROS_INFO("last_avg_right: %f", std::abs(last_avg_right - avg_right));
    //             turn_right_ = true; // 这里的 turn_right_ 变量需要在类中定义并初始化
    //             detected = true;
    //         }
    //     }

    //    return detected;
    // }

    bool isBoundaryLine(const std::vector<cv::Point>& points) {
        if (points.size() < 20) {  // 确保有足够数量的点
            return false;
        }

        int boundary_count = 0;
        for (const auto& pt : points) {
            if (pt.x == 0 || pt.x == 639) {
                boundary_count++;
            }
     }

        // 如果有超过 80% 的点在边界上，则认为是边界线
     if (boundary_count >= points.size() * 0.8) {
           // 检查点是否均匀分布在边界上
            int first_y = points.front().y;
           int last_y = points.back().y;
           if (last_y - first_y > points.size() / 2) {  // 检查y值的分布是否广泛
             return true;
            }
        }

     return false;
    }
    int chooseMidPoint(const std::vector<cv::Point>& mid_points, const std::string& strategy) {
        int num_points = static_cast<int>(mid_points.size());
        if (num_points == 0) {
            return frame_center_;
        }

        if (strategy == "median") {
            // 略去中位数计算部分，仅展示平均值计算
        } else {  // Default to average
            int num_points_to_average = std::min(num_points, 5); // 最多计算前5个中点的平均值
            int sum_x = 0;

            for (int i = 0; i < num_points_to_average; ++i) {
                sum_x += mid_points[i].x;
            }

            return static_cast<int>(sum_x / num_points_to_average);
        }
    }

    // int chooseMidPoint(const std::vector<cv::Point>& mid_points, const std::string& strategy) {
    //     int num_points = static_cast<int>(mid_points.size());
    //     if (num_points == 0) {
    //         return frame_center_;
    //     }

    //     if (strategy == "median") {
    //         std::vector<int> mid_x_coords;
    //         for (const auto& pt : mid_points) {
    //             mid_x_coords.push_back(pt.x);
    //         }
    //         std::nth_element(mid_x_coords.begin(), mid_x_coords.begin() + num_points / 2, mid_x_coords.end());
    //         return mid_x_coords[num_points / 2];
    //     } else {  // Default to average
    //         return static_cast<int>(std::accumulate(mid_points.begin(), mid_points.end(), cv::Point(0, 0),
    //                                                [](cv::Point sum, const cv::Point& pt) { return sum + pt; }).x / num_points);
    //     }
    // }

    double controlCar(int error) {
        double control_signal = Kp_ * error + Kd_ * (error - prev_error_);
        prev_error_ = error;
        return clamp(control_signal, -max_control_signal_, max_control_signal_);
    }

    void publishControl(double control_signal) {
        geometry_msgs::Twist twist;
        twist.linear.x = 0.25;
        twist.angular.z = control_signal;
        cmd_vel_pub_.publish(twist);
    }

    


    void stopFollowing() {
        follow_lane_flag_ = false;
        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_pub_.publish(twist);
    }

    
    void rotate(double angle) {
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;

        if (angle >= 0) {
           if (angle <= 180) {
               cmd_vel_msg.angular.z = -angular_turn_msg_;
           } else {
              angle = 360 - angle;
               cmd_vel_msg.angular.z = angular_turn_msg_;
           }
      } else {
           angle = -angle; // 将负值角度转换为正值
           if (angle <= 180) {
               cmd_vel_msg.angular.z = angular_turn_msg_;
           } else {
                angle = 360 - angle;
                cmd_vel_msg.angular.z = -angular_turn_msg_;
         }
     }

        double angular_duration = angle / angular_turn_msg_ / 180.0 * M_PI;
        int ticks = static_cast<int>(angular_duration * rate_);

     ROS_INFO("Starting rotation for %d ticks", ticks);

        ros::Rate rate(rate_);
        for (int i = 0; i < ticks; ++i) {
            cmd_vel_pub_.publish(cmd_vel_msg);
            rate.sleep();
     }

     // Stop the rotation
     cmd_vel_msg.angular.z = 0.0;
     cmd_vel_pub_.publish(cmd_vel_msg);
    }   

    void controlCallback(const std_msgs::Bool::ConstPtr& msg) {
        follow_lane_flag_ = msg->data;
        if (follow_lane_flag_) {
            current_state_ = FOLLOWING_LANE;
            ROS_INFO("Starting lane following.");
        } else {
            stopFollowing();
            ROS_INFO("Stopping lane following.");
        }
    }

    double clamp(double value, double min_value, double max_value) {
        return std::max(min_value, std::min(value, max_value));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_follower_node");
    LaneFollower lane_follower;
    lane_follower.run();
    return 0;
}
