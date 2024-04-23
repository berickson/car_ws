
#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


#include "geometry_msgs/msg/point_stamped.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

const int previewWidth = 320;
const int previewHeight = 180;
const int nnWidth = 640;
const int nnHeight = 640;


dai::Pipeline createPipeline(bool syncNN, std::string nnPath) {
    dai::Pipeline pipeline;
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

    void callback(std::shared_ptr<dai::ImgDetections> det) {
        // if(det->detections.size() > 0) {
        //     geometry_msgs::msg::PointStamped cone_location;
        //     cone_location.header.stamp = node->now();
        //     cone_location.header.frame_id = "oak_rgb_camera_optical_frame";
        //     cone_location.point.x = det->detections[0].bbox.center.x;
        //     cone_location.point.y = det->detections[0].bbox.center.y;
        //     cone_location.point.z = 0;
        //     cone_location_publisher->publish(cone_location);
        // }
    }


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cone_detector_node");

    // create a PointStamped publisher for cone location
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cone_location_publisher = node->create_publisher<geometry_msgs::msg::PointStamped>("cone_location", 10);


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

    std::string color_uri = cameraParamUri + "/" + "color.yaml";

    const int queueDepth = 1;
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);


    bool lazy = false;
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
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

    dai::rosBridge::ImgDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", previewWidth, previewHeight, false);
    dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ImgDetections> detectionPublish(
        nNetDataQueue,
        node,
        std::string("color/cone_detections"),
        std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        queueDepth);
    

    nNetDataQueue.get()->addCallback([&](std::shared_ptr<dai::ADatatype> data) {
        auto detections = std::dynamic_pointer_cast<dai::ImgDetections>(data);
        geometry_msgs::msg::PointStamped cone_location;
        cone_location.header.stamp = node->now();
        cone_location.header.frame_id = "oak_rgb_camera_optical_frame";
        auto & d = detections.get()->detections[0];

        if(detections.get()->detections.size() > 0) {
            float x_fov_degrees = 69;
            float y_fov_degrees = 54;

            float x_resolution = 1.0; // x and y go between 0 and 1
            float y_resolution = 1.0;

            float x_width = d.xmax-d.xmin;
            float x_center = (d.xmax+d.xmin)/2.0;
            float x_angle_degrees = -1 * (x_resolution/2.0 - (x_center) )/x_resolution * x_fov_degrees;
            float x_width_degrees = (d.xmax-d.xmin) * x_fov_degrees / x_resolution;
            float width_radians = x_width_degrees * (M_PI / 180.0); 
            float cone_width = 0.30; // meters, width of the cone about camera height up
            float cone_distance_width = (cone_width/2.0) / sin(width_radians/2.0);

            // do the same thing for cone height
            float y_height = d.ymax-d.ymin;
            float y_center = (d.ymax+d.ymin)/2.0;
            float y_angle_degrees = -1 * (y_resolution/2.0 - (y_center) )/y_resolution * y_fov_degrees;
            float y_height_degrees = (d.ymax-d.ymin) * y_fov_degrees / y_resolution;
            float height_radians = y_height_degrees * (M_PI / 180.0);
            float cone_height = 0.6; // bigger than natural height of 0.45 because image is stretched to square?
            float cone_distance_height = (cone_height/2.0) / sin(height_radians/2.0);

            float cone_distance = std::min(cone_distance_width, cone_distance_height);


            cone_location.point.x = cone_distance * sin(x_angle_degrees * M_PI / 180.0);
            cone_location.point.y = cone_distance * sin(y_angle_degrees * M_PI / 180.0);
            cone_location.point.z = cone_distance; // * cos(x_angle_degrees * M_PI / 180.0) * cos(y_angle_degrees * M_PI / 180.0);

        } else {
            cone_location.point.x = 0;
            cone_location.point.y = 0;
            cone_location.point.z = 0;
        }
        cone_location_publisher->publish(cone_location);
    });

    detectionPublish.addPublisherCallback();


    rgbPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.

    rclcpp::spin(node);

    return 0;
}
