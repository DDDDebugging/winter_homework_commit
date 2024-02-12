//
// Created by mijiao on 24-1-30.
//

#ifndef HOMEWORK_HOMEWORK_NODE_H
#define HOMEWORK_HOMEWORK_NODE_H

#include <websocketpp/config/asio_no_tls.hpp>

#include <websocketpp/server.hpp>

#include <iostream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

using Server = websocketpp::server<websocketpp::config::asio>;
using message_ptr = Server::message_ptr;

class Homework : public rclcpp::Node {
private:
    Server server;
    websocketpp::connection_hdl hdl;
    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr clickPointSubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;


public:
    Homework() : Node("homework_node") {
        clickPointSubscription = create_subscription<geometry_msgs::msg::Point32>(
                "/click_position",
                10,
                [this](geometry_msgs::msg::Point32::SharedPtr msg) {
                    try {
                        float info[2]{msg->x, msg->y};
                        server.send(hdl, &info, sizeof(info), websocketpp::frame::opcode::value::binary);
                        RCLCPP_INFO_STREAM(this->get_logger(),
                                           "Send position: (" << msg->x << " " << msg->y << ")");
                    } catch (websocketpp::exception const &e) {
                        RCLCPP_FATAL_STREAM(this->get_logger(),
                                            "Send failed because: " << "(" << e.what() << ")");
                    }
                });
        imagePublisher = create_publisher<sensor_msgs::msg::Image>("/raw_image", 10);

        std::thread server_thread([this]() {
            try {
                server.set_access_channels(websocketpp::log::alevel::none);
                server.clear_access_channels(websocketpp::log::alevel::none);
                server.init_asio();
                server.listen(23333);
                server.set_open_handler([this](websocketpp::connection_hdl _hdl) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "connect success!");
                    hdl = _hdl;
                });
                server.set_message_handler([this](websocketpp::connection_hdl, message_ptr msg) {
                    cv::Mat frame;
                    cv::_InputArray buffer(msg->get_payload().data(), msg->get_payload().length());
                    frame = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
                    imagePublisher->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());

                });
                server.start_accept();
                server.run();
            } catch (websocketpp::exception const &e) {
                using namespace std::chrono_literals;
                RCLCPP_FATAL_STREAM(this->get_logger(), e.what());
                exit(0);
            }
        });
        server_thread.detach();

        std::thread front_thread([this]() {
            std::cout<<system(FRONT_END);
        });
        front_thread.detach();
    }
};

#endif //HOMEWORK_HOMEWORK_NODE_H
