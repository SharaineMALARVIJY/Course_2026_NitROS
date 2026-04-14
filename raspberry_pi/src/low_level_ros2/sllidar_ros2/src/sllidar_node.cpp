/*
 *  SLLIDAR ROS2 NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include "sl_lidar.h"
#include "math.h"
#include <signal.h>
#include <vector>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)
#define ROS2VERSION "1.0.1"

using namespace sl;

bool need_exit = false;

class SLlidarNode : public rclcpp::Node
{
  private:    
    void init_param()
    {
        this->declare_parameter<std::string>("channel_type","serial");
        this->declare_parameter<std::string>("tcp_ip", "192.168.0.7");
        this->declare_parameter<int>("tcp_port", 20108);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("serial_baudrate",1000000);
        this->declare_parameter<std::string>("frame_id","laser_frame");
        this->declare_parameter<bool>("inverted", false);
        this->declare_parameter<bool>("angle_compensate", false);
        this->declare_parameter<std::string>("scan_mode",std::string());
        this->declare_parameter<float>("scan_frequency",10.0);
        
        // Paramètres du filtre
        this->declare_parameter<float>("angle_min_filter", -1.75);
        this->declare_parameter<float>("angle_max_filter", 1.75);

        this->get_parameter("channel_type", channel_type);
        this->get_parameter("tcp_ip", tcp_ip); 
        this->get_parameter("tcp_port", tcp_port);
        this->get_parameter("serial_port", serial_port); 
        this->get_parameter("serial_baudrate", serial_baudrate);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("inverted", inverted);
        this->get_parameter("angle_compensate", angle_compensate);
        this->get_parameter("scan_mode", scan_mode);
        this->get_parameter("scan_frequency", scan_frequency);
        this->get_parameter("angle_min_filter", angle_min_filter);
        this->get_parameter("angle_max_filter", angle_max_filter);
    }

    bool getSLLIDARDeviceInfo(ILidarDriver * drv)
    {
        sl_result     op_result;
        sl_lidar_response_device_info_t devinfo;

        op_result = drv->getDeviceInfo(devinfo);
        if (SL_IS_FAIL(op_result)) {
            if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
                RCLCPP_ERROR(this->get_logger(),"Error, operation time out. SL_RESULT_OPERATION_TIMEOUT! ");
            } else {
                RCLCPP_ERROR(this->get_logger(),"Error, unexpected error, code: %x",op_result);
            }
            return false;
        }

        // print out the device serial number, firmware and hardware version number..
        char sn_str[37] = {'\0'}; 
        for (int pos = 0; pos < 16 ;++pos) {
            sprintf(sn_str + (pos * 2),"%02X", devinfo.serialnum[pos]);
        }
        RCLCPP_INFO(this->get_logger(),"SLLidar S/N: %s",sn_str);
        RCLCPP_INFO(this->get_logger(),"Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
        RCLCPP_INFO(this->get_logger(),"Hardware Rev: %d",(int)devinfo.hardware_version);
        return true;
    }

    bool checkSLLIDARHealth(ILidarDriver * drv)
    {
        sl_result     op_result;
        sl_lidar_response_device_health_t healthinfo;
        op_result = drv->getHealth(healthinfo);
        if (SL_IS_OK(op_result)) { 
            RCLCPP_INFO(this->get_logger(),"SLLidar health status : %d", healthinfo.status);
            switch (healthinfo.status) {
                case SL_LIDAR_STATUS_OK:
                    RCLCPP_INFO(this->get_logger(),"SLLidar health status : OK.");
                    return true;
                case SL_LIDAR_STATUS_WARNING:
                    RCLCPP_INFO(this->get_logger(),"SLLidar health status : Warning.");
                    return true;
                case SL_LIDAR_STATUS_ERROR:
                    RCLCPP_ERROR(this->get_logger(),"Error, SLLidar internal error detected. Please reboot the device to retry.");
                    return false;
                default:
                    RCLCPP_ERROR(this->get_logger(),"Error, Unknown internal error detected. Please reboot the device to retry.");
                    return false;

            }
        } else {
            RCLCPP_ERROR(this->get_logger(),"Error, cannot retrieve SLLidar health code: %x", op_result);
            return false;
        }
    }

    bool stop_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req; (void)res;
        if(!drv) return false;
        drv->setMotorSpeed(0);
        return true;
    }

    bool start_motor(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req; (void)res;
        if(!drv) return false;
        drv->setMotorSpeed();
        drv->startScan(0,1);
        return true;
    }

    static float getAngle(const sl_lidar_response_measurement_node_hq_t& node)
    {
        return node.angle_z_q14 * 90.f / 16384.f;
    }

    void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
                  sl_lidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, rclcpp::Time start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
                  std::string frame_id)
    {
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = start;
        scan_msg->header.frame_id = frame_id;

        bool reversed = (angle_max > angle_min);
        if (reversed) {
            scan_msg->angle_min = angle_min;
            scan_msg->angle_max = angle_max;
        } else {
            scan_msg->angle_min = angle_max;
            scan_msg->angle_max = angle_min;
        }
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count-1);

        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(node_count-1);
        scan_msg->range_min = 0.15;
        scan_msg->range_max = max_distance;

        scan_msg->intensities.resize(node_count);
        scan_msg->ranges.resize(node_count);

        bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
            size_t index = reverse_data ? (node_count - 1 - i) : i;
            
            if (read_value == 0.0)
                scan_msg->ranges[index] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[index] = read_value;
            scan_msg->intensities[index] = (float) (nodes[i].quality >> 2);
        }
        pub->publish(*scan_msg);
    }

public:    

  public:
    SLlidarNode() : Node("sllidar_node")
    {
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));
        init_param();
        
        // Initialisation UNIQUE du driver ici
        drv = *createLidarDriver();
        IChannel* _channel = *createSerialPortChannel(serial_port, serial_baudrate);

        if (SL_IS_FAIL((drv)->connect(_channel))) {
            RCLCPP_ERROR(this->get_logger(),"Error connecting to LIDAR.");
        }
    }

    ~SLlidarNode() {
        if (drv) {
            RCLCPP_INFO(this->get_logger(), "Fermeture propre : Arrêt du moteur LIDAR.");
            drv->setMotorSpeed(0);
            drv->stop();
            drv->disconnect();
        }
    }
    
    int work_loop()
    {        
        // On ne rappelle PAS init_param() ni createLidarDriver() ici car c'est déjà fait
        if (!drv) return -1;

        getSLLIDARDeviceInfo(drv);
        checkSLLIDARHealth(drv);

        stop_motor_service = this->create_service<std_srvs::srv::Empty>("stop_motor", std::bind(&SLlidarNode::stop_motor,this,std::placeholders::_1,std::placeholders::_2));
        start_motor_service = this->create_service<std_srvs::srv::Empty>("start_motor", std::bind(&SLlidarNode::start_motor,this,std::placeholders::_1,std::placeholders::_2));

        drv->setMotorSpeed();
        LidarScanMode current_scan_mode;
        drv->startScan(false, true, 0, &current_scan_mode);

        int points_per_circle = (int)(1000*1000/current_scan_mode.us_per_sample/scan_frequency);
        angle_compensate_multiple = points_per_circle/360.0 + 1;
        if(angle_compensate_multiple < 1) angle_compensate_multiple = 1.0;
        max_distance = (float)current_scan_mode.max_distance;

        while (rclcpp::ok() && !need_exit) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t count = _countof(nodes);

            rclcpp::Time start_scan_time = this->now();
            sl_result op_result = drv->grabScanDataHq(nodes, count);
            double scan_duration = (this->now() - start_scan_time).seconds();

            if (op_result == SL_RESULT_OK) {
                drv->ascendScanData(nodes, count);

                if (angle_compensate) {
                    const int comp_count = 360 * angle_compensate_multiple;
                    std::vector<sl_lidar_response_measurement_node_hq_t> comp_nodes(comp_count);
                    // Utilisation des accolades {} pour éviter les warnings "missing initializer"
                    std::fill(comp_nodes.begin(), comp_nodes.end(), sl_lidar_response_measurement_node_hq_t{});

                    for (size_t i = 0; i < count; i++) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle_rad = DEG2RAD(getAngle(nodes[i]));
                            if (angle_rad > M_PI) angle_rad -= 2 * M_PI;

                            if (angle_rad < angle_min_filter || angle_rad > angle_max_filter) continue;

                            int angle_idx = (int)(getAngle(nodes[i]) * angle_compensate_multiple);
                            if (angle_idx >= comp_count) angle_idx = comp_count - 1;
                            
                            for (size_t j = 0; j < angle_compensate_multiple; j++) {
                                if (angle_idx + j < (size_t)comp_count) comp_nodes[angle_idx + j] = nodes[i];
                            }
                        }
                    }
                    publish_scan(scan_pub, comp_nodes.data(), comp_count, start_scan_time, scan_duration, inverted, 
                                DEG2RAD(0.0f), DEG2RAD(360.0f), max_distance, frame_id);
                }
                else {
                    // --- CORRECTION : FILTRAGE EN MODE NON-COMPENSÉ ---
                    std::vector<sl_lidar_response_measurement_node_hq_t> filtered;
                    for (size_t i = 0; i < count; i++) {
                        if (nodes[i].dist_mm_q2 == 0) continue;
                        
                        float angle = DEG2RAD(getAngle(nodes[i]));
                        if (angle > M_PI) angle -= 2 * M_PI;

                        if (angle >= angle_min_filter && angle <= angle_max_filter) {
                            filtered.push_back(nodes[i]);
                        }
                    }

                    if (filtered.size() > 1) {
                        // Crucial : Recalculer les angles min/max sur les points filtrés pour éviter l'étirement sur RViz
                        float a_min = DEG2RAD(getAngle(filtered.front()));
                        float a_max = DEG2RAD(getAngle(filtered.back()));
                        publish_scan(scan_pub, filtered.data(), filtered.size(), start_scan_time, scan_duration, inverted,
                                     a_min, a_max, max_distance, frame_id);
                    }
                }
            }
            rclcpp::spin_some(this->get_node_base_interface());
        }
        return 0;
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service;
    std::string channel_type, serial_port, frame_id, scan_mode, tcp_ip;
    int serial_baudrate, tcp_port;
    bool inverted, angle_compensate;
    float max_distance, scan_frequency, angle_min_filter, angle_max_filter;
    size_t angle_compensate_multiple;
    ILidarDriver * drv;    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SLlidarNode>();
    
    try {
        node->work_loop();
    } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Exception : %s", e.what());
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}