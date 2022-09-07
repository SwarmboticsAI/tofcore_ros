#include "sensor_node.hpp"

#include <t10utils/t10_sensor.hpp>
#include <t10utils/cartesian_transform.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <truesense_msgs/msg/frame.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr double SENSOR_PIXEL_SIZE_MM = 0.02; // camera sensor pixel size 20x20 um
constexpr int WIDTH = 320;
constexpr int WIDTH2 = 160;
constexpr int HEIGHT = 240;
constexpr int HEIGHT2 = 120;

constexpr auto PARAM_STREAM_TYPE = "stream_type";

T10Sensor::T10Sensor()
    : Node("t10_sensor", "t10")
{
  rclcpp::QoS pub_qos(10);
  pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  // Setup topic pulbishers
  pub_ambient_ = this->create_publisher<sensor_msgs::msg::Image>("ambient", pub_qos);
  pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("depth", pub_qos);
  pub_amplitude_ = this->create_publisher<sensor_msgs::msg::Image>("amplitude", pub_qos);
  pub_pcd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", pub_qos);
  pub_dcs_ = this->create_publisher<truesense_msgs::msg::Frame>("frame_raw", pub_qos);

  // connect to interface
  (void)interface_.subscribeCameraInfo([&](std::shared_ptr<t10utils::CameraInfo> ci) -> void
                                       { (void)ci; /*updateCameraInfo(ci);*/ });
  (void)interface_.subscribeFrame([&](std::shared_ptr<t10utils::Frame> f) -> void
                                  { updateFrame(*f); });

  cartesianTransform_.initLensTransform(SENSOR_PIXEL_SIZE_MM, WIDTH, HEIGHT, lensCenterOffsetX_, lensCenterOffsetY_, lensType_);

  // Setup ROS parameters

  this->declare_parameter(PARAM_STREAM_TYPE, "distance_amplitude");
  // Setup a callback so that we can react to parameter changes from the outside world.
  parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&T10Sensor::on_set_parameters_callback, this, _1));

  // Update all parameters
  auto params = this->get_parameters(this->list_parameters({}, 1).names);
  this->on_set_parameters_callback(params);
}

rcl_interfaces::msg::SetParametersResult T10Sensor::on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  interface_.stopStream();

  interface_.setIntegrationTime(500, 0, 0, 0);
  interface_.setModulation(0, 0);

  for (const auto &parameter : parameters)
  {
    if (parameter.get_name() == PARAM_STREAM_TYPE)
    {
      auto value = parameter.as_string();
      RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : \"%s\"", parameter.get_name().c_str(), value.c_str());
      if (value == "distance")
      {
        interface_.stopStream();
        interface_.streamDistance();
      }
      else if (value == "grayscale")
      {
        interface_.stopStream();
        interface_.streamGrayscale();
      }
      else if (value == "distance_amplitude")
      {
        interface_.stopStream();
        interface_.streamDistanceAmplitude();
      }
      else if (value == "dcs")
      {
        interface_.streamDCS();
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown stream type: \"%s\"", value.c_str());
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  // Here update class attributes, do some actions, etc.
  return result;
}

void T10Sensor::publish_amplData(const t10utils::Frame &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = "unknown";
  img.height = static_cast<uint32_t>(frame.height);
  img.width = static_cast<uint32_t>(frame.width);
  img.encoding = "mono16";
  img.step = img.width * frame.px_size;
  img.is_bigendian = 0;
  img.data = frame.amplData;
  pub.publish(img);
}

void T10Sensor::publish_distData(const t10utils::Frame &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = "unknown";
  img.height = static_cast<uint32_t>(frame.height);
  img.width = static_cast<uint32_t>(frame.width);
  img.encoding = "mono16";
  img.step = img.width * frame.px_size;
  img.is_bigendian = 0;
  img.data = frame.distData;
  pub.publish(img);
}

void T10Sensor::publish_pointCloud(const t10utils::Frame &frame, rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &pub, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 cloud_msg{};
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = "unknown";
  cloud_msg.is_dense = true;
  cloud_msg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.resize(frame.height * frame.width);
  modifier.setPointCloud2Fields(
      7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "amplitude", 1, sensor_msgs::msg::PointField::UINT16,
      "ambient", 1, sensor_msgs::msg::PointField::INT16,
      "valid", 1, sensor_msgs::msg::PointField::UINT8,
      "phase", 1, sensor_msgs::msg::PointField::UINT16);

  // Note: For some reason setPointCloudFields doesn't set row_step
  //      and resets msg height and width so setup them here.
  cloud_msg.height = static_cast<uint32_t>(frame.height);
  cloud_msg.width = static_cast<uint32_t>(frame.width);
  cloud_msg.row_step = frame.width * 19; // 19 is the size in bytes of all the point cloud fields

  sensor_msgs::PointCloud2Iterator<float> it_x{cloud_msg, "x"};
  sensor_msgs::PointCloud2Iterator<float> it_y{cloud_msg, "y"};
  sensor_msgs::PointCloud2Iterator<float> it_z{cloud_msg, "z"};
  sensor_msgs::PointCloud2Iterator<uint16_t> it_amplitude{cloud_msg, "amplitude"};
  sensor_msgs::PointCloud2Iterator<int16_t> it_ambient{cloud_msg, "ambient"};
  sensor_msgs::PointCloud2Iterator<uint8_t> it_valid{cloud_msg, "valid"};
  sensor_msgs::PointCloud2Iterator<uint16_t> it_phase{cloud_msg, "phase"};

  auto it_d = frame.distData.begin();
  auto it_a = frame.amplData.begin();
  uint32_t count = 0;
  while (it_d != frame.distData.end())
  {
    auto distance = (*(it_d + 1) << 8) + (*it_d);
    auto y = count / frame.width;
    auto x = count % frame.width;
    int valid = 0;
    double px, py, pz;
    px = py = pz = 0.1; // std::numeric_limits<float>::quiet_NaN();
    if (distance > 0 && distance < 65000)
    {
      cartesianTransform_.transformPixel(x, y, distance, px, py, pz);
      px /= 1000.0; // mm -> m
      py /= 1000.0; // mm -> m
      pz /= 1000.0; // mm -> m
      valid = 1;
    }

    *it_x = px;
    *it_y = py;
    *it_z = pz;
    if (frame.dataType == t10utils::Frame::AMPLITUDE)
    {
      *it_amplitude = (*(it_a + 1) << 8) + (*it_a);
      it_a += 2;
    }
    else
    {
      *it_amplitude = pz;
    }
    *it_ambient = 0;
    *it_phase = distance;
    *it_valid = valid;

    ++it_x;
    ++it_y;
    ++it_z;
    ++it_amplitude;
    ++it_ambient;
    ++it_phase;
    ++it_valid;
    ++count;
    it_d += 2;
  }
  pub.publish(cloud_msg);

  // int x, y, k, l;
  // for(k=0, l=0, y=0; y< frame->height; y++){
  //     for(x=0; x< frame->width; x++, k++, l+=2){
  //         pcl::PointXYZI &p = cloud->points[k];
  //         distance = (frame->distData[l+1] << 8) + frame->distData[l];

  //         if(frame->dataType == Frame::AMPLITUDE)
  //             amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];

  //         if (distance > 0 && distance < 65000){

  //             if(cartesian){
  //                 cartesianTransform.transformPixel(x, y, distance, px, py, pz);
  //                 p.x = static_cast<float>(px / 1000.0); //mm -> m
  //                 p.y = static_cast<float>(py / 1000.0);
  //                 p.z = static_cast<float>(pz / 1000.0);

  //                 if(frame->dataType == Frame::AMPLITUDE) p.intensity = static_cast<float>(amplitude);
  //                 else p.intensity = static_cast<float>(pz / 1000.0);

  //             }else{
  //                 p.x = x / 100.0;
  //                 p.y = y / 100.0;
  //                 p.z = distance / 1000.0;
  //                 if(frame->dataType == Frame::AMPLITUDE) p.intensity =  static_cast<float>(amplitude);
  //                 else p.intensity = static_cast<float>(distance / 1000.0);
  //             }
  //         }else{
  //             p.x = std::numeric_limits<float>::quiet_NaN();
  //             p.y = std::numeric_limits<float>::quiet_NaN();
  //             p.z = std::numeric_limits<float>::quiet_NaN();
  //         }
  //     }
  // }
}


void T10Sensor::publish_DCSData(const t10utils::Frame &frame, rclcpp::Publisher<truesense_msgs::msg::Frame> &pub, const rclcpp::Time& stamp)
{
  if(frame.dataType == t10utils::Frame::DCS) {
    for(auto i = 0; i != 4; ++i) {
        truesense_msgs::msg::Frame img;
        img.header.stamp = stamp;
        img.header.frame_id = "unknown";
        img.height = static_cast<uint32_t>(frame.height);
        img.width = static_cast<uint32_t>(frame.width);
        img.encoding = "";
        img.step = frame.width*2;
        img.index = frame.frame_id;
        img.type = i;
        img.temperature = std::numeric_limits<float>::quiet_NaN(); //TODO Need to get the real temperature in degrees C
        img.modulation_frequency = 12; //TODO need to get mod freq in MHz
        img.integration_time = 200;  //TODO need to get the integration time in microseconds
        // # Binning refers here to any camera setting which combines rectangular
        // #  neighborhoods of pixels into larger "super-pixels." It reduces the
        // #  resolution of the output image to
        // #  (width / binning_h) x (height / binning_v).
        // # The default values binning_h = binning_v = 0 is considered the same
        // #  as binning_h = binning_v = 1 (no subsampling).
        img.binning_h = 0;
        img.binning_v = 0;
        img.config_index = 0; //This is the config index from Truesense sensors, really Not useful in this case.
        img.vled_mv = 0; //TODO Need to get the real value.
        img.chip_id = "deedbeef"; //TODO need to calcualte the chipid from teh sensor info (wafer id, etc)

        auto frame_size = img.step * img.height;
        img.data.resize(img.step * img.height);
        auto begin = frame.dcsData.begin() + (i*frame_size);
        auto end = begin + frame_size;
        std::copy(begin, end, img.data.begin());
        pub.publish(img);
    }

    //HACK: publish a fake ambient frame to test using this with the truesense::processor node.
    truesense_msgs::msg::Frame img;
    img.header.stamp = stamp;
    img.header.frame_id = "unknown";
    img.height = static_cast<uint32_t>(frame.height);
    img.width = static_cast<uint32_t>(frame.width);
    img.encoding = "";
    img.step = frame.width*2;
    img.index = frame.frame_id;
    img.type = truesense_msgs::msg::Frame::TYPE_AMBIENT;
    img.temperature = std::numeric_limits<float>::quiet_NaN(); //TODO Need to get the real temperature in degrees C
    img.modulation_frequency = 12; //TODO need to get mod freq in MHz
    img.integration_time = 200;  //TODO need to get the integration time in microseconds
// # Binning refers here to any camera setting which combines rectangular
// #  neighborhoods of pixels into larger "super-pixels." It reduces the
// #  resolution of the output image to
// #  (width / binning_h) x (height / binning_v).
// # The default values binning_h = binning_v = 0 is considered the same
// #  as binning_h = binning_v = 1 (no subsampling).
    img.binning_h = 0;
    img.binning_v = 0;
    img.config_index = 0; //This is the config index from Truesense sensors, really Not useful in this case.
    img.vled_mv = 0; //TODO Need to get the real value.
    img.chip_id = "deedbeef"; //TODO need to calcualte the chipid from the sensor info (wafer id, etc)

    img.data.resize(img.step * img.height, 0);
    pub.publish(img);
  }
}


void T10Sensor::updateFrame(const t10utils::Frame &frame)
{
  auto stamp = this->now();
  switch (frame.dataType)
  {
  case t10utils::Frame::GRAYSCALE:
  {
    publish_amplData(frame, *pub_ambient_, stamp);
    break;
  }
  case t10utils::Frame::AMPLITUDE:
  {
    publish_amplData(frame, *pub_amplitude_, stamp);
    publish_distData(frame, *pub_depth_, stamp);
    publish_pointCloud(frame, *pub_pcd_, stamp);
    break;
  }
  case t10utils::Frame::DISTANCE:
  {
    publish_distData(frame, *pub_depth_, stamp);
    publish_pointCloud(frame, *pub_pcd_, stamp);
    break;
  }
  case t10utils::Frame::DCS:
  {
    publish_DCSData(frame, *pub_dcs_, stamp);
    break;
  }
  }
}