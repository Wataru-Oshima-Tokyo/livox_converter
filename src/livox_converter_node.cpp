#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>  // Adjust this include path if necessary
#include <livox_ros_driver2/CustomMsg.h> // Adjust this include path if necessary
#include <omp.h> // Include the OpenMP header

ros::Publisher pub;

// Callback function to handle incoming Livox data
void livoxMsgCallback(const livox_ros_driver::CustomMsg::ConstPtr& input_msg) {
    livox_ros_driver2::CustomMsg output_msg;

    // Manually copy the data from the input_msg to the output_msg
    output_msg.header = input_msg->header;
    output_msg.timebase = input_msg->timebase;
    output_msg.point_num = input_msg->point_num;
    output_msg.lidar_id = input_msg->lidar_id;
    // Iterate over the points and copy them individually
    output_msg.points.resize(input_msg->points.size()); // Preallocate space for points

    // Parallelize the loop with OpenMP
    #pragma omp parallel for
    for (size_t i = 0; i < input_msg->points.size(); ++i) {
        const auto& point = input_msg->points[i];
        auto& new_point = output_msg.points[i];
        
        new_point.x = point.x;
        new_point.y = point.y;
        new_point.z = point.z;
        new_point.reflectivity = point.reflectivity;
        new_point.tag = point.tag;
        new_point.line = point.line;
    }

    // Now, publish the message to the new topic for livox_ros_driver2
    pub.publish(output_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_converter_node");
    ros::NodeHandle nh;

    // Subscribe to the original topic from livox_ros_driver
    ros::Subscriber sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, livoxMsgCallback);

    // Publisher for the new topic, using the message type from livox_ros_driver2
    pub = nh.advertise<livox_ros_driver2::CustomMsg>("/livox/lidar_custom", 10);

    ros::spin();

    return 0;
}
