
#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include "geometry_msgs/msg/point_stamped.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

const int previewWidth = 320;
const int previewHeight = 180;
const int nnWidth = 640;
const int nnHeight = 640;




dai::Pipeline createPipeline(bool syncNN, std::string nnPath) {
    dai::Pipeline pipeline;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    xoutDepth->setStreamName("depth");
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    auto detectionNetwork = pipeline.create<dai::node::YoloDetectionNetwork>();  // Use YoloDetectionNetwork
    auto nnOut = pipeline.create<dai::node::XLinkOut>();


    auto imageManip = pipeline.create<dai::node::ImageManip>();
    imageManip->setMaxOutputFrameSize(previewWidth*previewHeight*3);
    imageManip->initialConfig.setResize(previewWidth, previewHeight);
    imageManip->initialConfig.setCropRect(0,0,1,1);
    imageManip->initialConfig.setFrameType(dai::RawImgFrame::Type::BGR888p);
    imageManip->setKeepAspectRatio(false);


    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");

    colorCam->setPreviewSize(nnWidth, nnHeight);
    colorCam->setPreviewKeepAspectRatio(false);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(10);

    // StereoDepth
    int stereo_confidence = 200;
    bool lrcheck = true;
    bool extended = false;
    bool subpixel = true;
    int LRchecktresh = 5;
    stereo->initialConfig.setConfidenceThreshold(stereo_confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);

    // testing YOLOv8n DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);

    detectionNetwork->setCoordinateSize(4);
    detectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    detectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
    detectionNetwork->setIouThreshold(0.9f);
    detectionNetwork->setBlobPath(nnPath);
    detectionNetwork->setNumInferenceThreads(2);
    detectionNetwork->input.setBlocking(false);

    detectionNetwork->setNumClasses(1);  // Set number of classes to 1 for "cone"

    // Link plugins CAM -> PREVIEW -> XLINK
    //              CAM -> IMAGE_MANIP -> NN
    colorCam->preview.link(detectionNetwork->input);
    colorCam->preview.link(imageManip->inputImage);
    imageManip->out.link(xlinkOut->input);

    //detectionNetwork->passthrough.link(imageManip->out);

    detectionNetwork->out.link(nnOut->input);
    return pipeline;
}

class ConeDetectorNode : public rclcpp::Node {
    public:
    ConeDetectorNode() : Node("cone_detector_node", rclcpp::NodeOptions().enable_logger_service(true)) {
        RCLCPP_INFO(this->get_logger(), "Cone Detector Node has started");
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // log start
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Cone Detector Node");

    auto node = std::make_shared<ConeDetectorNode>();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("color/cone_markers", 10);


    std::string tfPrefix, resourceBaseFolder, nnPath;
    std::string cameraParamUri = "package://cone_detector/params/camera";
    std::string nnName(BLOB_NAME);
    bool syncNN;
    int bad_params = 0;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", cameraParamUri);
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("sync_nn", syncNN);
    node->declare_parameter<std::string>("nnName", "");

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", cameraParamUri);
    node->get_parameter("sync_nn", syncNN);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg;

    // subscribe to scan topic
    scan_subscription_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, [node,&scan_msg](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_msg = msg;
    });

    resourceBaseFolder = ament_index_cpp::get_package_share_directory("cone_detector");
    resourceBaseFolder = resourceBaseFolder + "/resources";

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("resourceBaseFolder is empty: Send the path to the resource folder containing NNBlob in \'resourceBaseFolder\' parameter");
    }
    // Uses the path from param if passed or else uses from BLOB_PATH from CMAKE
    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {
        node->get_parameter("nnName", nnName);
    }

    nnPath = resourceBaseFolder + "/" + nnName;
    // log nnName
    RCLCPP_INFO(node->get_logger(), "Using NN: %s", nnPath.c_str());
    dai::Pipeline pipeline = createPipeline(syncNN, nnPath);
    dai::Device device(pipeline);

    std::shared_ptr<dai::DataOutputQueue> previewQueue = device.getOutputQueue("preview", 2, false);
    std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = device.getOutputQueue("detections", 2, false);
    auto stereoQueue = device.getOutputQueue("depth", 2, false);

    std::string color_uri = cameraParamUri + "/" + "color.yaml";

    const int queueDepth = 1;
    
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);


    bool lazy = false;
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgb_publisher(
        previewQueue,
        node,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                    &rgbConverter,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                    std::placeholders::_1,
                    std::placeholders::_2),
        queueDepth,
        color_uri,
        "color", 
        lazy);

    // publish depth camera
    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>> depthPublish;
    // auto calibrationHandler = std::make_shared<camera_info_manager::CameraInfoManager>(node, tfPrefix + "_right_camera_optical_frame", color_uri);
    int monoWidth = 640;
    int monoHeight = 480;
    auto calibrationHandler = device.readCalibration();

    // todo: change back to right camera
    // dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame", true);
    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_rgb_camera_optical_frame", true);

    auto stereoCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    bool useDepth = true;
    if(useDepth) {
        depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            stereoQueue,
            node,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &depthConverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            stereoCameraInfo,
            "stereo");
    }


    dai::rosBridge::ImgDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", previewWidth, previewHeight, false);
    dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ImgDetections> detection_publisher(
        nNetDataQueue,
        node,
        std::string("color/cone_detections"),
        std::bind(
            &dai::rosBridge::ImgDetectionConverter::toRosMsg, 
            &detConverter,
            std::placeholders::_1,
            std::placeholders::_2),
        queueDepth);
    

    nNetDataQueue.get()->addCallback([&](std::shared_ptr<dai::ADatatype> data) {
        auto detections = std::dynamic_pointer_cast<dai::ImgDetections>(data);

        visualization_msgs::msg::MarkerArray marker_array;
        //marker_array.markers.resize(detections.get()->detections.size());

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "oak_rgb_camera_optical_frame";
        marker.header.stamp = node->now();
        marker.ns = "cone_markers";

        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(marker);


        for(int i=0; i < detections.get()->detections.size(); ++i) {
            auto frame_id = "oak_rgb_camera_optical_frame";
            auto stamp = node->now();


            geometry_msgs::msg::PointStamped cone_location;
            cone_location.header.stamp = node->now();
            cone_location.header.frame_id = frame_id;

            auto & d = detections.get()->detections[i];

            // FOV numbers from here, then adjusted:
            //    https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9095/
            float x_fov = 1.1 * 69 * M_PI / 180.0;
            float y_fov = 0.7 * 54 * M_PI / 180.0;

            // x and y readings go between 0 and 1
            // +x is right, +y is down

            // calculations using width
            float x_width = d.xmax-d.xmin;
            float x_center = (d.xmax+d.xmin)/2.0;
            float x_angle = (x_center - 0.5) * x_fov;
            float width_radians = (d.xmax-d.xmin) * x_fov;
            float cone_width = 0.38;
            float cone_distance_by_width = (cone_width/2.0) / tan(width_radians/2.0);

            // calculations using height
            float y_height = d.ymax-d.ymin;
            float y_center = (d.ymax+d.ymin)/2.0;
            float y_angle = (y_center - 0.5) * y_fov;
            float height_radians = (d.ymax-d.ymin) * y_fov;
            float cone_height = 0.45; // bigger than natural height of 0.45 because image is stretched to square?
            float cone_distance_by_height = (cone_height/2.0) / tan(height_radians/2.0);


            // calculate distance using scan
            float scan_distance = std::numeric_limits<float>::infinity();
            int min_scan_index = -1;
            if(scan_msg != nullptr) {
                // we'll just look at the center of the scan
                int center_count = 20;
                for(int i = scan_msg->ranges.size() - center_count; i < scan_msg->ranges.size(); i++) {
                    if(scan_msg->ranges[i] < scan_distance) {
                    scan_distance = scan_msg->ranges[i];
                    min_scan_index = i;
                    }
                }
                for(int i = 0; i < center_count; i++) {
                    if(scan_msg->ranges[i] < scan_distance) {
                    scan_distance = scan_msg->ranges[i];
                    min_scan_index = i;
                    }
                }
            }

            // scanner is 15 cm behind camera
            scan_distance -= 0.15;


            // take the smaller of the two visual distances, since clipping can make it seem further away
            float visual_cone_distance = std::min(cone_distance_by_width, cone_distance_by_height);

            // if cone is close and scan is close, use scan distance
            float cone_distance = (visual_cone_distance < 1 && scan_distance < 1) ? scan_distance : visual_cone_distance;


            if(i == 0) {
                // log distance by height and width
                RCLCPP_DEBUG(node->get_logger(), "Distance by scan: %f", scan_distance);
                RCLCPP_DEBUG(node->get_logger(), "Distance by width: %f", cone_distance_by_width);
                RCLCPP_DEBUG(node->get_logger(), "Distance by height: %f", cone_distance_by_height);
            }



            auto x = cone_distance * sin(x_angle);
            auto y = cone_distance * sin(y_angle);
            auto z = cone_distance;



            cone_location.point.x = x;
            cone_location.point.y = y;
            cone_location.point.z = z;

            visualization_msgs::msg::Marker marker;


            marker.header.stamp = stamp;
            marker.header.frame_id = frame_id;
            marker.ns = "cone_markers";
            marker.id = i;

            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::MODIFY;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.x = 0.707;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 0.707;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.45;
            // bright orange cone coloer
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
            // marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(marker);


        }
        marker_array_publisher->publish(marker_array);
    });

    detection_publisher.addPublisherCallback();
    if(useDepth) {
        depthPublish->addPublisherCallback();
    }


    rgb_publisher.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spinning Cone Detector Node");
    rclcpp::spin(node);

    return 0;
}
