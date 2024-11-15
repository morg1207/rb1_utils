
#include "rb1_utils/transform_utils.hpp"

TransformUtils::TransformUtils(std::shared_ptr<rclcpp::Node> node) : node_(node){ 

}

msgTransform TransformUtils::getTransform( std::unique_ptr<tf2_ros::Buffer>& tf_buffer, std::string to_frame, std::string from_frame) {
    RCLCPP_DEBUG(node_->get_logger(), " ");
    RCLCPP_DEBUG(node_->get_logger(), "********* [get transform] ********");
    RCLCPP_DEBUG(node_->get_logger(), " ");

    msgTransform msg_transform;
    msg_transform.transform.translation.z = std::numeric_limits<double>::max();
    
    rclcpp::Rate rate(20);

    while (msg_transform.transform.translation.z == std::numeric_limits<double>::max() && rclcpp::ok()) {
        try {
            msg_transform = tf_buffer->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
            RCLCPP_DEBUG(node_->get_logger(), "Transformation has been obtained.");
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(node_->get_logger(), "Could not transform %s to %s: %s", to_frame.c_str(), from_frame.c_str(), ex.what());
        }
        rate.sleep();
    }

    // Imprimir los valores de la transformación
    float x = msg_transform.transform.translation.x;
    float y = msg_transform.transform.translation.y;
    float z = msg_transform.transform.translation.z;
    double roll, pitch, yaw;

    tf2::Quaternion q(
        msg_transform.transform.rotation.x,
        msg_transform.transform.rotation.y,
        msg_transform.transform.rotation.z,
        msg_transform.transform.rotation.w
    );
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_DEBUG(node_->get_logger(), "Transformation from [%s] to [%s]:", from_frame.c_str(), to_frame.c_str());
    RCLCPP_DEBUG(node_->get_logger(), "Translation: x = %.2f, y = %.2f, z = %.2f", x, y, z);
    RCLCPP_DEBUG(node_->get_logger(), "Rotation: roll = %.2f, pitch = %.2f, yaw = %.2f", roll, pitch, yaw);

    RCLCPP_DEBUG(node_->get_logger(), "********************");
    
    return msg_transform;

}

msgTransform TransformUtils::findTransform(msgTransform& transform, msgPoint& position_object){
  
  // Obtengo la posicion del objeto
  float pos_object[3];
  pos_object[0] = position_object.x;
  pos_object[1] = position_object.y;
  pos_object[2] = position_object.z;
  
  float x;
  float y;
  double roll, pitch, yaw;

  x = transform.transform.translation.x;
  y = transform.transform.translation.y;

  tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w);

  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  msgTransform t;
  t.transform.translation.x = x + pos_object[0] * std::sin(pos_object[1]) * std::sin(yaw) + pos_object[0] * std::cos(pos_object[1]) * std::cos(yaw);
  t.transform.translation.y = y + pos_object[0] * std::cos(pos_object[1]) * std::sin(yaw) - pos_object[0] * std::sin(pos_object[1]) * std::cos(yaw);
  t.transform.translation.z = 0.0;

  tf2::Quaternion q_t;
  q_t.setRPY(roll, pitch - M_PI, yaw - pos_object[2] + M_PI); // Sin rotación

  t.transform.rotation.x = q_t.x();
  t.transform.rotation.y = q_t.y();
  t.transform.rotation.z = q_t.z();
  t.transform.rotation.w = q_t.w();

  return t;
}

void TransformUtils::sendTransform(std::shared_ptr<tf2_ros::StaticTransformBroadcaster>& tf_broadcaster, msgTransform& msg_transform, std::string child_frame, std::string parent_frame){

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent_frame;
  t.header.stamp = node_->now();

  t.child_frame_id = child_frame;

  t.transform = msg_transform.transform;

  tf_broadcaster->sendTransform(t);

  RCLCPP_DEBUG(node_->get_logger(), "Transformation published: parent_frame [%s], child_frame [%s]", parent_frame.c_str(), child_frame.c_str());

}


TransformationMatrix2D TransformUtils::createTransformationMatrix(double x, double y, double theta) {
TransformationMatrix2D matrix;

matrix.m[0][0] = std::cos(theta);
matrix.m[0][1] = -std::sin(theta);
matrix.m[0][2] = x;

matrix.m[1][0] = std::sin(theta);
matrix.m[1][1] = std::cos(theta);
matrix.m[1][2] = y;

matrix.m[2][0] = 0;
matrix.m[2][1] = 0;
matrix.m[2][2] = 1;

return matrix;
}

TransformationMatrix2D TransformUtils::invertTransformationMatrix(const TransformationMatrix2D &matrix) {
TransformationMatrix2D inverse;

// Transponer la submatriz de rotación
inverse.m[0][0] = matrix.m[0][0];
inverse.m[0][1] = matrix.m[1][0];
inverse.m[1][0] = matrix.m[0][1];
inverse.m[1][1] = matrix.m[1][1];

// Calcular la nueva traslación
inverse.m[0][2] =
        -(matrix.m[0][0] * matrix.m[0][2] + matrix.m[1][0] * matrix.m[1][2]);
inverse.m[1][2] =
        -(matrix.m[0][1] * matrix.m[0][2] + matrix.m[1][1] * matrix.m[1][2]);

// La última fila sigue siendo la misma
inverse.m[2][0] = 0;
inverse.m[2][1] = 0;
inverse.m[2][2] = 1;

return inverse;
}