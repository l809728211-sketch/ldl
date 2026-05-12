#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

// RGB 转 HSV 的极速换算函数 (专为嵌入式优化)
void RGBtoHSV(uint8_t r_in, uint8_t g_in, uint8_t b_in, float &h, float &s, float &v) {
    float r = r_in / 255.0f;
    float g = g_in / 255.0f;
    float b = b_in / 255.0f;
    float cmax = std::max({r, g, b});
    float cmin = std::min({r, g, b});
    float delta = cmax - cmin;

    v = cmax;
    if (cmax > 0.0001f) {
        s = delta / cmax;
    } else {
        s = 0; h = 0; return;
    }

    if (delta < 0.0001f) {
        h = 0;
    } else if (cmax == r) {
        h = 60.0f * ((g - b) / delta);
    } else if (cmax == g) {
        h = 60.0f * ((b - r) / delta + 2.0f);
    } else {
        h = 60.0f * ((r - g) / delta + 4.0f);
    }
    if (h < 0.0f) h += 360.0f;
}

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
    
    // 【融合特性1】：默认参数保持与你原来雷达一致，但允许 Launch 文件动态修改
    std::string input_topic = this->declare_parameter<std::string>("input_topic", "/segmentation/obstacle");
    std::string output_topic = this->declare_parameter<std::string>("output_topic", "/processed_pointcloud");

    // 【融合特性2】：声明相机的颜色与高度过滤参数 (带默认值)
    this->declare_parameter("y_max_safe_height", 0.18); 
    this->declare_parameter("grass_h_min", 35.0);
    this->declare_parameter("grass_h_max", 85.0);
    this->declare_parameter("grass_s_min", 0.2); 
    this->declare_parameter("flower_s_min", 0.5); 
    this->declare_parameter("flower_v_min", 0.5); 
    this->declare_parameter("rock_s_max", 0.25);  
    this->declare_parameter("root_v_max", 0.35);  

    // 订阅点云话题
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessor::process_pointcloud, this, std::placeholders::_1)
    );

    // 发布处理后的点云话题
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic, rclcpp::QoS(10)
    );

    RCLCPP_INFO(this->get_logger(), "智能处理节点已运行，监听话题: %s", input_topic.c_str());
}

void PointCloudProcessor::process_pointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 获取相机专用的动态参数 (对于雷达数据虽然获取了但不会用到)
    float y_max_safe_height = this->get_parameter("y_max_safe_height").as_double();
    float grass_h_min = this->get_parameter("grass_h_min").as_double();
    float grass_h_max = this->get_parameter("grass_h_max").as_double();
    float grass_s_min = this->get_parameter("grass_s_min").as_double();
    float flower_s_min = this->get_parameter("flower_s_min").as_double();
    float flower_v_min = this->get_parameter("flower_v_min").as_double();
    float rock_s_max = this->get_parameter("rock_s_max").as_double();
    float root_v_max = this->get_parameter("root_v_max").as_double();

    // 创建一个新的 PointCloud2 消息用于存储过滤后的点云
    sensor_msgs::msg::PointCloud2 filtered_cloud;
    filtered_cloud.header = msg->header;
    filtered_cloud.fields = msg->fields;
    filtered_cloud.is_bigendian = msg->is_bigendian;
    filtered_cloud.is_dense = msg->is_dense;

    size_t point_step = msg->point_step;
    size_t data_size = msg->data.size();

    // 创建一个向量来存储过滤后的点云数据，预留空间提高性能
    std::vector<uint8_t> filtered_data;
    filtered_data.reserve(data_size);

    // 动态寻找 x, y, z 和 rgb 的偏移量 (更健壮的做法)
    int x_offset = -1, y_offset = -1, z_offset = -1, rgb_offset = -1;
    for (const auto& field : msg->fields) {
        if (field.name == "x") x_offset = field.offset;
        if (field.name == "y") y_offset = field.offset;
        if (field.name == "z") z_offset = field.offset;
        if (field.name == "rgb" || field.name == "rgba") rgb_offset = field.offset;
    }

    // 防御性编程：如果没有找到xyz，直接返回
    if (x_offset == -1 || y_offset == -1 || z_offset == -1) return;

    // ==========================================================
    // 分支逻辑：自动判断是 雷达数据(无色) 还是 相机数据(有色)
    // ==========================================================
    if (rgb_offset == -1) {
        // 【模式 A：原始雷达模式】(无 rgb 字段)
        // 遍历所有点云数据，仅执行你的原始 bounding box 过滤逻辑
        for (size_t i = 0; i < data_size; i += point_step) {
            float x = *reinterpret_cast<const float*>(&msg->data[i + x_offset]);
            float y = *reinterpret_cast<const float*>(&msg->data[i + y_offset]);
            float z = *reinterpret_cast<const float*>(&msg->data[i + z_offset]);

            // 过滤条件：保留指定范围之外的点 (去除车体自身)
            if (x >= -0.5 && x <= 0.5 && y >= -0.4 && y <= 0.4 && z >= -0.50 && z <= 1.55) {
                continue;
            }    
            // 将符合条件的点云数据添加到 filtered_data
            filtered_data.insert(filtered_data.end(), msg->data.begin() + i, msg->data.begin() + i + point_step);
        }
    } else {
        // 【模式 B：深度相机模式】(有 rgb 字段)
        // 遍历所有点云数据，执行 高度过滤 + 颜色语义过滤
        for (size_t i = 0; i < data_size; i += point_step) {
            float x = *reinterpret_cast<const float*>(&msg->data[i + x_offset]);
            float y = *reinterpret_cast<const float*>(&msg->data[i + y_offset]);
            float z = *reinterpret_cast<const float*>(&msg->data[i + z_offset]);

            // 1. 过滤车体自身 (通用保护)
            if (x >= -0.5 && x <= 0.5 && y >= -0.4 && y <= 0.4 && z >= -0.50 && z <= 1.55) {
                continue;
            }

            // 2. 高度过滤：贴地面的草、小土包直接压过去 (相机坐标系Y轴向下)
            if (y > y_max_safe_height) {
                continue;
            }

            // 3. 色彩语义提取
            uint8_t b = msg->data[i + rgb_offset];
            uint8_t g = msg->data[i + rgb_offset + 1];
            uint8_t r = msg->data[i + rgb_offset + 2];

            float h_val, s_val, v_val;
            RGBtoHSV(r, g, b, h_val, s_val, v_val);

            // A：硬石头/深树根 (低饱和度或低亮度) -> 保留
            bool is_rock_or_root = (s_val < rock_s_max) || (v_val < root_v_max);
            // B：草 (绿色) -> 丢弃
            bool is_grass = (h_val >= grass_h_min && h_val <= grass_h_max && s_val >= grass_s_min);
            // C：野花 (鲜艳高亮) -> 丢弃
            bool is_flower = (s_val >= flower_s_min && v_val >= flower_v_min);

            // 综合判断
            if (is_rock_or_root) {
                filtered_data.insert(filtered_data.end(), msg->data.begin() + i, msg->data.begin() + i + point_step);
            } else if (is_grass || is_flower) {
                continue; 
            } else {
                filtered_data.insert(filtered_data.end(), msg->data.begin() + i, msg->data.begin() + i + point_step);
            }
        }
    }

    // 更新 filtered_cloud 的尺寸信息
    filtered_cloud.width = filtered_data.size() / point_step;
    filtered_cloud.height = 1;
    filtered_cloud.row_step = filtered_cloud.width * point_step;
    filtered_cloud.point_step = point_step;
    filtered_cloud.data = std::move(filtered_data);

    // 发布过滤后的点云
    if (filtered_cloud.width > 0) {
        publisher_->publish(filtered_cloud);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudProcessor)