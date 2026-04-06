#include <ros/ros.h>
#include <prometheus_msgs/UAVState.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

class UAVStateSender {
private:
    ros::NodeHandle nh_;
    ros::Subscriber uav_state_sub_;
    
    // UDP通信相关
    int udp_socket_;
    struct sockaddr_in windows_addr_;
    
    // 配置参数
    std::string windows_ip_;
    int windows_port_;
    int send_rate_;
    bool save_to_file_;
    std::string log_file_;
    
    // 文件流
    std::ofstream log_stream_;
    
    // 统计
    int message_count_;
    ros::Time last_print_time_;
    
public:
    UAVStateSender() : nh_("~"), message_count_(0) {
        // 加载参数
        nh_.param<std::string>("windows_ip", windows_ip_, "10.193.133.230");
        nh_.param<int>("windows_port", windows_port_, 8889);
        nh_.param<int>("send_rate", send_rate_, 2);
        nh_.param<bool>("save_to_file", save_to_file_, true);
        nh_.param<std::string>("log_file", log_file_, "/home/amov/uav_data_log.csv");
        
        // 初始化UDP socket
        initUDPSocket();
        
        // 订阅正确的无人机状态话题
        uav_state_sub_ = nh_.subscribe("/uav1/prometheus/state", 10, 
                                       &UAVStateSender::stateCallback, this);
        
        // 如果需要保存到文件，打开文件
        if (save_to_file_) {
            log_stream_.open(log_file_, std::ios::out | std::ios::app);
            if (log_stream_.is_open()) {
                log_stream_ << "timestamp,pos_x,pos_y,pos_z,yaw" << std::endl;
                ROS_INFO("Log file opened: %s", log_file_.c_str());
            } else {
                ROS_WARN("Cannot open log file: %s", log_file_.c_str());
            }
        }
        
        last_print_time_ = ros::Time::now();
        
        ROS_INFO("========================================");
        ROS_INFO("UAV State Sender Initialized");
        ROS_INFO("Subscribing to: /uav1/prometheus/state");
        ROS_INFO("Target Windows IP: %s:%d", windows_ip_.c_str(), windows_port_);
        ROS_INFO("Send rate: %d Hz", send_rate_);
        ROS_INFO("========================================");
    }
    
    ~UAVStateSender() {
        if (udp_socket_ >= 0) {
            close(udp_socket_);
        }
        if (log_stream_.is_open()) {
            log_stream_.close();
        }
    }
    
    void initUDPSocket() {
        // 创建UDP socket
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            ROS_ERROR("Failed to create UDP socket");
            return;
        }
        
        // 配置Windows端地址
        memset(&windows_addr_, 0, sizeof(windows_addr_));
        windows_addr_.sin_family = AF_INET;
        windows_addr_.sin_port = htons(windows_port_);
        
        if (inet_pton(AF_INET, windows_ip_.c_str(), &windows_addr_.sin_addr) <= 0) {
            ROS_ERROR("Invalid Windows IP address: %s", windows_ip_.c_str());
            return;
        }
        
        ROS_INFO("UDP socket created successfully");
    }
    
    void stateCallback(const prometheus_msgs::UAVState::ConstPtr& msg) {
        message_count_++;
        
        // 获取当前时间戳（毫秒）
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        
        // 构造JSON格式数据（只包含MsgType, timestamp, position, yaw）
        std::string json_data = formatAsJSON(msg, timestamp);
        
        // 通过UDP发送
        ssize_t sent = sendto(udp_socket_, json_data.c_str(), json_data.length(), 0,
                               (struct sockaddr*)&windows_addr_, sizeof(windows_addr_));
        
        if (sent < 0) {
            ROS_ERROR("Failed to send UDP packet");
        }
        
        // 保存到文件
        if (save_to_file_ && log_stream_.is_open()) {
            saveToFile(msg, timestamp);
        }
        
        // 定期打印状态（每秒一次）
        ros::Time current_time = ros::Time::now();
        if ((current_time - last_print_time_).toSec() >= 1.0) {
            ROS_INFO("Received %d messages, last pos: (%.2f, %.2f, %.2f), yaw: %.2f rad", 
                     message_count_,
                     msg->position[0], msg->position[1], msg->position[2],
                     msg->attitude[2]);  // attitude[2] 是 yaw
            message_count_ = 0;
            last_print_time_ = current_time;
        }
    }
    
    std::string formatAsJSON(const prometheus_msgs::UAVState::ConstPtr& msg, long timestamp) {
        std::ostringstream oss;
        oss << "{";
        oss << "\"MsgType\":0,";
        oss << "\"timestamp\":" << timestamp << ",";
        oss << "\"position\":[" << msg->position[0] << "," 
                               << msg->position[1] << "," 
                               << msg->position[2] << "],";
        oss << "\"yaw\":" << msg->attitude[2];  // 只输出方位朝向（偏航角）
        oss << "}";
        return oss.str();
    }
    
    void saveToFile(const prometheus_msgs::UAVState::ConstPtr& msg, long timestamp) {
        log_stream_ << timestamp << ","
                   << msg->position[0] << "," << msg->position[1] << "," << msg->position[2] << ","
                   << msg->attitude[2]  // 只保存 yaw
                   << std::endl;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_state_sender");
    UAVStateSender sender;
    ros::spin();
    return 0;
}
