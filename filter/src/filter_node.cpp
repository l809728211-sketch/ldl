#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class PointCloudProcessor : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    // 回调函数
    void process_pointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);

public:
    explicit PointCloudProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

PointCloudProcessor::PointCloudProcessor(const rclcpp::NodeOptions & options)
: Node("pointcloud_processor", options) {
    std::string input_topic = "/segmentation/obstacle";
    std::string output_topic = "/processed_pointcloud";

    // 订阅点云话题
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessor::process_pointcloud, this, std::placeholders::_1)
    );

    // 发布处理后的点云话题
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, rclcpp::QoS(10)
    );

    RCLCPP_INFO(this->get_logger(), "处理节点运行");
}

void PointCloudProcessor::process_pointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 创建一个新的 PointCloud2 消息用于存储过滤后的点云
    sensor_msgs::msg::PointCloud2 filtered_cloud;

    // 初始化 filtered_cloud 的头信息
    filtered_cloud.header = msg->header;

    // 初始化字段
    filtered_cloud.fields = msg->fields;

    // 确保点云数据格式正确
    size_t point_step = msg->point_step;
    size_t data_size = msg->data.size();

    // 创建一个向量来存储过滤后的点云数据
    std::vector<uint8_t> filtered_data;

    // 遍历所有点云数据
    for (size_t i = 0; i < data_size; i += point_step) {
        // 提取 x, y, z 坐标
        float x = *reinterpret_cast<const float*>(msg->data.data() + i);
        float y = *reinterpret_cast<const float*>(msg->data.data() + i + 4);
        float z = *reinterpret_cast<const float*>(msg->data.data() + i + 8);

        // 过滤条件：保留 x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0 && z >= -1.0 && z <= 1.0 的点
        // 如果你想保留其他点，可以调整条件
        if (x >= -0.5 && x <= 0.5 && y >= -0.4 && y <= 0.4 && z >= -0.50 && z <= 1.55) {
            continue;
        }    
            // 将符合条件的点云数据添加到 filtered_data
            filtered_data.insert(filtered_data.end(), msg->data.begin() + i, msg->data.begin() + i + point_step);
        
    }

    // 更新 filtered_cloud 的宽度和行步长
    filtered_cloud.width = filtered_data.size() / point_step;
    filtered_cloud.height = 1;
    filtered_cloud.row_step = filtered_cloud.width * point_step;
    filtered_cloud.point_step = point_step;
    filtered_cloud.is_bigendian = msg->is_bigendian;
    filtered_cloud.is_dense = msg->is_dense;
    filtered_cloud.data = std::move(filtered_data);

    // 发布过滤后的点云
    if (filtered_cloud.width > 0) {
        publisher_->publish(filtered_cloud);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudProcessor)