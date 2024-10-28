#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/detection.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "yolov8_tensorrt_ros/yolov8.hpp"

const std::vector<std::string> CLASS_NAMES = {
    "person",         "bicycle",    "car",           "motorcycle",    "airplane",     "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",    "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",        "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",     "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball",  "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",       "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",       "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",        "donut",         "cake",
    "chair",          "couch",      "potted plant",  "bed",           "dining table", "toilet",        "tv",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",   "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",        "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"};

const std::vector<std::vector<unsigned int>> COLORS = {
    {0, 114, 189},   {217, 83, 25},   {237, 177, 32},  {126, 47, 142},  {119, 172, 48},  {77, 190, 238},
    {162, 20, 47},   {76, 76, 76},    {153, 153, 153}, {255, 0, 0},     {255, 128, 0},   {191, 191, 0},
    {0, 255, 0},     {0, 0, 255},     {170, 0, 255},   {85, 85, 0},     {85, 170, 0},    {85, 255, 0},
    {170, 85, 0},    {170, 170, 0},   {170, 255, 0},   {255, 85, 0},    {255, 170, 0},   {255, 255, 0},
    {0, 85, 128},    {0, 170, 128},   {0, 255, 128},   {85, 0, 128},    {85, 85, 128},   {85, 170, 128},
    {85, 255, 128},  {170, 0, 128},   {170, 85, 128},  {170, 170, 128}, {170, 255, 128}, {255, 0, 128},
    {255, 85, 128},  {255, 170, 128}, {255, 255, 128}, {0, 85, 255},    {0, 170, 255},   {0, 255, 255},
    {85, 0, 255},    {85, 85, 255},   {85, 170, 255},  {85, 255, 255},  {170, 0, 255},   {170, 85, 255},
    {170, 170, 255}, {170, 255, 255}, {255, 0, 255},   {255, 85, 255},  {255, 170, 255}, {85, 0, 0},
    {128, 0, 0},     {170, 0, 0},     {212, 0, 0},     {255, 0, 0},     {0, 43, 0},      {0, 85, 0},
    {0, 128, 0},     {0, 170, 0},     {0, 212, 0},     {0, 255, 0},     {0, 0, 43},      {0, 0, 85},
    {0, 0, 128},     {0, 0, 170},     {0, 0, 212},     {0, 0, 255},     {0, 0, 0},       {36, 36, 36},
    {73, 73, 73},    {109, 109, 109}, {146, 146, 146}, {182, 182, 182}, {219, 219, 219}, {0, 114, 189},
    {80, 183, 189},  {128, 128, 0}};

class YOLOv8LifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    YOLOv8LifecycleNode()
    : rclcpp_lifecycle::LifecycleNode("yolov8_lifecycle_node") {
        this->declare_parameter<int>("image_width", 640);
        this->declare_parameter<int>("image_height", 640);
        this->declare_parameter<float>("score_threshold", 0.25f);
        this->declare_parameter<float>("iou_threshold", 0.65f);
        this->declare_parameter<std::string>("model_path", "path/to/model");

        this->get_parameter("model_path", model_path);
        this->get_parameter("image_width", image_width);
        this->get_parameter("image_height", image_height);
        this->get_parameter("score_threshold", score_threshold);
        this->get_parameter("iou_threshold", iou_threshold);
    }

    ~YOLOv8LifecycleNode() {}

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(this->get_logger(), "Configuring");
        yolov8 = new YOLOv8(model_path);
        yolov8->make_pipe(true);
        publisher_ = this->create_publisher<yolov8_msgs::msg::DetectionArray>("/yolov8_tensorrt_node/detections", 10);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/yolov8_tensorrt_node/debug_image", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&YOLOv8LifecycleNode::image_callback, this, std::placeholders::_1));
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(this->get_logger(), "Activating");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(this->get_logger(), "Deactivating");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(this->get_logger(), "Cleaning up");
        delete yolov8;
        yolov8 = nullptr;
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received an image, processing...");
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat res;
            cv::Size size        = cv::Size{image_width, image_height};
            std::vector<Object> objs;
            try {
                auto start = std::chrono::steady_clock::now();
                yolov8->copy_from_Mat(cv_image, size);
                yolov8->infer();
                yolov8->postprocess(objs, score_threshold, iou_threshold, 100, 80);
                auto end = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                RCLCPP_INFO(this->get_logger(), "Inference time: %ld ms", duration);
                yolov8->draw_objects(cv_image, res, objs, CLASS_NAMES, COLORS);
                sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", res).toImageMsg();
                image_publisher_->publish(*out_msg);
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            yolov8_msgs::msg::DetectionArray detection_array;
            detection_array.header = msg->header;  // 将 ROS 消息头传递给检测数组

            for (const auto& obj : objs) {
                yolov8_msgs::msg::Detection detection;
                detection.class_id = obj.label;
                detection.class_name = CLASS_NAMES[obj.label];  // 确保 CLASS_NAMES 包含正确的类别名称
                detection.score = obj.prob;

                // 2D边界框中心位置和方向
                detection.bbox.center.position.x = obj.rect.x + obj.rect.width / 2.0;
                detection.bbox.center.position.y = obj.rect.y + obj.rect.height / 2.0;
                detection.bbox.center.theta = 0;  // 如果没有方向信息，则默认为0

                // 边界框的尺寸
                detection.bbox.size.x = obj.rect.width;
                detection.bbox.size.y = obj.rect.height;

                // 其他字段保持默认或空值
                // detection.bbox3d, detection.mask, detection.keypoints, detection.keypoints3d

                detection_array.detections.push_back(detection);
            }

            publisher_->publish(detection_array);
        }
    }

    std::string model_path;
    int image_width;
    int image_height;
    float score_threshold;
    float iou_threshold;
    YOLOv8* yolov8;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp_lifecycle::LifecyclePublisher<yolov8_msgs::msg::DetectionArray>::SharedPtr publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<YOLOv8LifecycleNode>();  // 替换为您的生命周期节点类名
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
