#pragma once

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"


#include "vector"
#include "iostream"
#include "functional"
#include "chrono"


using msgTransform = geometry_msgs::msg::TransformStamped;
using msgPoint = geometry_msgs::msg::Point;


struct TransformationMatrix2D {
  double m[3][3];

  void print() const {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        std::cout << m[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
};


class TransformUtils {
    
public:
    TransformUtils(std::shared_ptr<rclcpp::Node>);
    msgTransform getTransform(std::unique_ptr<tf2_ros::Buffer>& ,std::string, std::string);
    msgTransform findTransform(msgTransform&, msgPoint&);
    void sendTransform(std::shared_ptr<tf2_ros::StaticTransformBroadcaster>&, msgTransform& , std::string, std::string);
    TransformationMatrix2D createTransformationMatrix(double x, double y, double theta);
    TransformationMatrix2D invertTransformationMatrix(const TransformationMatrix2D &matrix);

private:
    std::shared_ptr<rclcpp::Node> node_;
};