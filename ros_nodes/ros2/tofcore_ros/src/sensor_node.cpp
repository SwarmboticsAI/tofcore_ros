#include "sensor_node.hpp"

using namespace std::chrono_literals;
using namespace std::string_literals;

constexpr double SENSOR_PIXEL_SIZE_MM = 0.02; // camera sensor pixel size 20x20 um
constexpr int m_width = 320;
constexpr int HEIGHT = 240;
constexpr int LENS_CENTER_OFFSET_X = 0;
constexpr int LENS_CENTER_OFFSET_Y = 0;
constexpr int32_t MIN_INTEGRATION_TIME = 0;
constexpr int32_t MAX_INTEGRATION_TIME = 4000;

// Read Only params
constexpr auto API_VERSION = "api_version";
constexpr auto CHIP_ID = "chip_id";
constexpr auto MODEL_NAME = "model_name";
constexpr auto SW_VERSION = "sw_version";
constexpr auto SENSOR_URI = "sensor_uri";
constexpr auto DESIRED_LOCATION = "desired_location";

// Configurable params
constexpr auto CAPTURE_MODE = "capture_mode";
constexpr auto INTEGRATION_TIME = "integration_time";
constexpr auto STREAMING_STATE = "streaming";
constexpr auto MODULATION_FREQUENCY = "modulation_frequency";
constexpr auto DISTANCE_OFFSET = "distance_offset";
constexpr auto MINIMUM_AMPLITUDE = "minimum_amplitude";
constexpr auto MAXIMUM_AMPLITUDE = "maximum_amplitude";
constexpr auto FLIP_HORIZONTAL = "flip_horizontal";
constexpr auto FLIP_VERITCAL = "flip_vertical";
constexpr auto BINNING = "binning";
constexpr auto SENSOR_NAME = "sensor_name";
constexpr auto SENSOR_LOCATION = "sensor_location";

// Filter parameters
constexpr auto MEDIAN_FILTER = "median_filter";
constexpr auto MEDIAN_KERNEL = "median_kernel";
constexpr auto BILATERAL_FILTER = "bilateral_filter";
constexpr auto BILATERAL_KERNEL = "bilateral_kernel";
constexpr auto BILATERAL_COLOR = "bilateral_color";
constexpr auto BILATERAL_SPACE = "bilateral_space";

constexpr auto GRADIENT_FILTER = "gradient_filter";
constexpr auto GRADIENT_KERNEL = "gradient_kernel";
constexpr auto GRADIENT_THRESHOLD = "gradient_threshold";
constexpr auto GRADIENT_FILTER_SUPPORT = "gradient_filter_support";

// hdr parameters
constexpr auto HDR_ENABLE = "hdr_enable";
constexpr auto HDR_INTEGRATIONS = "hdr_integrations";

/// Quick helper function that return true if the string haystack starts with the string needle
bool begins_with(const std::string &needle, const std::string &haystack)
{
  return haystack.rfind(needle, 0) == 0;
}

cv::Mat neighbour_mask(const cv::Mat &mask, int neighbour_support)
{
  // find pixels that have at least <neighbour_support> neighbours that are flagged as valid
  cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 1, 1, 1,
                    1, 0, 1,
                    1, 1, 1);
  cv::Mat neighbour_count;
  cv::filter2D(mask / 255, neighbour_count, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
  cv::Mat mask_new;
  cv::bitwise_and(mask, (neighbour_count >= neighbour_support), mask_new);

  return mask_new;
}

ToFSensor::ToFSensor()
    : Node("tof_sensor", "tof_sensor")
{
  rclcpp::QoS pub_qos(10);
  pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  pub_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  // Setup ROS parameters
  rcl_interfaces::msg::ParameterDescriptor readonly_descriptor;
  readonly_descriptor.read_only = true;

  std::vector<tofcore::device_info_t> devices = tofcore::find_all_devices(std::chrono::seconds(5), std::numeric_limits<int>::max());
  interface_.reset(new tofcore::Sensor(devices.begin()->connector_uri));

  // interface_->stopStream();

  // Get sensor info
  TofComm::versionData_t versionData{};
  interface_->getSensorInfo(versionData);

  // use generic lens transform always
  std::vector<double> rays_x, rays_y, rays_z;
  try
  {
    interface_->getLensInfo(rays_x, rays_y, rays_z);
    cartesianTransform_.initLensTransform(m_width, HEIGHT, rays_x, rays_y, rays_z);
  }
  catch(...)
  {
    RCLCPP_FATAL(this->get_logger(), "Error reading lens info from sensor.");
  }
  // Read Only params
  this->declare_parameter(API_VERSION, versionData.m_softwareSourceID, readonly_descriptor); // TODO: Update this when API version is availible
  this->declare_parameter(CHIP_ID, std::to_string(versionData.m_sensorChipId), readonly_descriptor);
  this->declare_parameter(MODEL_NAME, versionData.m_modelName, readonly_descriptor);
  this->declare_parameter(SW_VERSION, versionData.m_softwareVersion, readonly_descriptor);

  // Configurable params
  this->declare_parameter(CAPTURE_MODE, "distance_amplitude");
  this->declare_parameter(STREAMING_STATE, true);
  this->declare_parameter(MODULATION_FREQUENCY, 12000);
  this->declare_parameter(DISTANCE_OFFSET, 0);
  this->declare_parameter(MINIMUM_AMPLITUDE, 0);
  this->declare_parameter(MAXIMUM_AMPLITUDE, 2000);
  this->declare_parameter(FLIP_HORIZONTAL, false);
  this->declare_parameter(FLIP_VERITCAL, false);
  this->declare_parameter(BINNING, false);

  this->declare_parameter(MEDIAN_FILTER, false);
  this->declare_parameter(MEDIAN_KERNEL, 3);
  this->declare_parameter(BILATERAL_FILTER, false);
  this->declare_parameter(BILATERAL_KERNEL, 5);
  this->declare_parameter(BILATERAL_COLOR, 75);
  this->declare_parameter(BILATERAL_SPACE, 75);
  this->declare_parameter(GRADIENT_FILTER, false);
  this->declare_parameter(GRADIENT_KERNEL, 1);
  this->declare_parameter(GRADIENT_THRESHOLD, 50);
  this->declare_parameter(GRADIENT_FILTER_SUPPORT, 6);

  this->declare_parameter(HDR_ENABLE, false);
  this->declare_parameter(HDR_INTEGRATIONS, "");

  // Reading optional values from sensor
  std::optional<std::string> init_name = interface_->getSensorName();
  std::optional<std::string> init_location = interface_->getSensorLocation();
  std::optional<short unsigned int> init_integration = interface_->getIntegrationTime();

  if (init_name)
    this->declare_parameter(SENSOR_NAME, *init_name);
  else
    this->declare_parameter(SENSOR_NAME, "Mojave");

  if (init_location)
  {
    this->declare_parameter(SENSOR_LOCATION, *init_location);
    this->sensor_location_ = *init_location;
  }
  else
  {
    this->declare_parameter(SENSOR_LOCATION, "Unknown");
    this->sensor_location_ = "Unknown";
  }

  if (init_integration)
    this->declare_parameter(INTEGRATION_TIME, *init_integration);
  else
    this->declare_parameter(INTEGRATION_TIME, 500);

  // Setup a callback so that we can react to parameter changes from the outside world.

  parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ToFSensor::on_set_parameters_callback, this, std::placeholders::_1));

  // Setup topic pulbishers
  pub_ambient_ = this->create_publisher<sensor_msgs::msg::Image>("ambient", pub_qos);
  pub_distance_ = this->create_publisher<sensor_msgs::msg::Image>("depth", pub_qos); // renamed this from distance to depth to match tof_sensor node
  pub_amplitude_ = this->create_publisher<sensor_msgs::msg::Image>("amplitude", pub_qos);
  pub_pcd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", pub_qos);
  pub_integration_ = this->create_publisher<tofcore_msgs::msg::IntegrationTime>("frame_raw", pub_qos);

  for (size_t i = 0; i != pub_dcs_.size(); i++)
  {
    std::string topic{"dcs"};
    topic += std::to_string(i);
    pub_dcs_[i] = this->create_publisher<sensor_msgs::msg::Image>(topic, pub_qos);
  }

  sensor_temperature_tl = this->create_publisher<sensor_msgs::msg::Temperature>("sensor_temperature_tl_" + this->sensor_location_, pub_qos);
  sensor_temperature_tr = this->create_publisher<sensor_msgs::msg::Temperature>("sensor_temperature_tr_" + this->sensor_location_, pub_qos);
  sensor_temperature_bl = this->create_publisher<sensor_msgs::msg::Temperature>("sensor_temperature_bl_" + this->sensor_location_, pub_qos);
  sensor_temperature_br = this->create_publisher<sensor_msgs::msg::Temperature>("sensor_temperature_br_" + this->sensor_location_, pub_qos);

  // Update all parameters
  auto params = this->get_parameters(this->list_parameters({}, 1).names);
  this->on_set_parameters_callback(params);

  (void)interface_->subscribeMeasurement([&](std::shared_ptr<tofcore::Measurement_T> f) -> void
                                         { updateFrame(*f); });
}

rcl_interfaces::msg::SetParametersResult ToFSensor::on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  // assume success, if any parameter set below fails this will be changed
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &parameter : parameters)
  {
    auto name = parameter.get_name();
    if (name == CAPTURE_MODE)
    {
      bool streaming = true;
      this->get_parameter(STREAMING_STATE, streaming);
      if (streaming)
      {
        this->apply_stream_type_param(parameter, result);
      }
    }
    else if (begins_with("integration_time", name))
    {
      this->apply_integration_time_param(parameter, result);
    }
    else if (name == STREAMING_STATE)
    {
      this->apply_streaming_param(parameter, result);
    }
    else if (name == MODULATION_FREQUENCY)
    {
      this->apply_modulation_frequency_param(parameter, result);
    }
    else if (name == DISTANCE_OFFSET)
    {
      this->apply_distance_offset_param(parameter, result);
    }
    else if (name == MINIMUM_AMPLITUDE)
    {
      this->apply_minimum_amplitude_param(parameter, result);
    }
    else if (name == MAXIMUM_AMPLITUDE)
    {
      this->apply_param(maximum_amplitude_, parameter);
    }
    else if (name == FLIP_HORIZONTAL)
    {
      this->apply_flip_horizontal_param(parameter, result);
    }
    else if (name == FLIP_VERITCAL)
    {
      this->apply_flip_vertical_param(parameter, result);
    }
    else if (name == BINNING)
    {
      this->apply_binning_param(parameter, result);
    }
    else if (name == SENSOR_NAME)
    {
      this->apply_sensor_name_param(parameter, result);
    }
    else if (name == SENSOR_LOCATION)
    {
      this->apply_sensor_location_param(parameter, result);
    }
    else if (name == MEDIAN_FILTER)
    {
      this->apply_param(median_filter_, parameter);
    }
    else if (name == MEDIAN_KERNEL)
    {
      this->apply_param(median_kernel_, parameter);
    }
    else if (name == BILATERAL_FILTER)
    {
      this->apply_param(bilateral_filter_, parameter);
    }
    else if (name == BILATERAL_KERNEL)
    {
      this->apply_param(bilateral_kernel_, parameter);
    }
    else if (name == BILATERAL_COLOR)
    {
      this->apply_param(bilateral_color_, parameter);
    }
    else if (name == BILATERAL_SPACE)
    {
      this->apply_param(bilateral_space_, parameter);
    }
    else if (name == GRADIENT_FILTER)
    {
      this->apply_param(gradient_filter_, parameter);
    }
    else if (name == GRADIENT_KERNEL)
    {
      this->apply_param(gradient_kernel_, parameter);
    }
    else if (name == GRADIENT_THRESHOLD)
    {
      this->apply_param(gradient_threshold_, parameter);
    }
    else if (name == GRADIENT_FILTER_SUPPORT)
    {
      this->apply_param(gradient_filter_support_, parameter);
    }
    else if (name == HDR_ENABLE)
    {
      this->apply_param(hdr_enable_, parameter);
      this->apply_vsm_settings();
    }
    else if (name == HDR_INTEGRATIONS)
    {
      this->apply_param(hdr_integrations_, parameter);
      this->apply_vsm_settings();
    }
    else
    {
      result.successful = false;
      result.reason = "Unknown parameter: "s + name;
    }
  }
  return result;
}

void ToFSensor::apply_stream_type_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &result)
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : \"%s\"", parameter.get_name().c_str(), value.c_str());
  if (value == "distance")
  {
    interface_->streamDistance();
  }
  else if (value == "distance_amplitude")
  {
    interface_->streamDistanceAmplitude();
  }
  else if (value == "dcs")
  {
    interface_->streamDCS();
  }
  else if (value == "dcs_ambient")
  {
    interface_->streamDCSAmbient();
  }
  else
  {
    result.successful = false;
    result.reason = "Unknown stream type: "s + value;
    RCLCPP_ERROR(this->get_logger(), result.reason.c_str());
  }
}

void ToFSensor::apply_integration_time_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &result)
{
  auto value = parameter.as_int();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %li", parameter.get_name().c_str(), value);
  if (value < MIN_INTEGRATION_TIME || value > MAX_INTEGRATION_TIME)
  {
    result.successful = false;
    result.reason = parameter.get_name() + " value is out of range";
  }
  else
  {
    interface_->setIntegrationTime(value);
  }
}

void ToFSensor::apply_hdr_mode_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &result)
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str());
  RCLCPP_INFO(this->get_logger(), "HDR not yet supported in ROS2");

  result.successful = false;
  result.reason = "Not supported";

  // if (begins_with("s", value)) // spacital
  // {
  //   interface_->setHDRMode(1);
  // }
  // else if (begins_with("t", value)) // temporal
  // {
  //   interface_->setHDRMode(2);
  // }
  // else if (begins_with("o", value)) // off
  // {
  //   interface_->setHDRMode(0);
  // }
  // else
  // {
  //   result.successful = false;
  //   result.reason = parameter.get_name() + " value is out of range";
  // }
}

void ToFSensor::apply_modulation_frequency_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &result)
{
  auto value = parameter.as_int();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %ld", parameter.get_name().c_str(), value);

  interface_->setModulation(value);
  result.successful = true;
}

void ToFSensor::apply_streaming_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &result)
{
  rcl_interfaces::msg::SetParametersResult dummy_result;

  try
  {
    auto value = parameter.as_bool();
    RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ? "true" : "false"));
    if (value)
    {
      rclcpp::Parameter stream_type;
      (void)this->get_parameter(CAPTURE_MODE, stream_type);
      this->apply_stream_type_param(stream_type, dummy_result);
    }
    else
    {
      interface_->stopStream();
    }
  }
  catch (std::exception &e)
  {
    result.successful = false;
    result.reason = e.what();
  }
}

void ToFSensor::apply_lens_type_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &result)
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str());

  // 0 - wide field, 1 - standard field, 2 - narrow field
  if (begins_with("w", value)) // wide FOV
  {
    cartesianTransform_.initLensTransform(SENSOR_PIXEL_SIZE_MM, m_width, HEIGHT, LENS_CENTER_OFFSET_X, LENS_CENTER_OFFSET_Y, 0);
  }
  else if (begins_with("s", value)) // standard fov
  {
    cartesianTransform_.initLensTransform(SENSOR_PIXEL_SIZE_MM, m_width, HEIGHT, LENS_CENTER_OFFSET_X, LENS_CENTER_OFFSET_Y, 1);
  }
  else if (begins_with("n", value)) // narrow fov
  {
    cartesianTransform_.initLensTransform(SENSOR_PIXEL_SIZE_MM, m_width, HEIGHT, LENS_CENTER_OFFSET_X, LENS_CENTER_OFFSET_Y, 2);
  }
  else if (begins_with("r", value))
  {
    std::vector<double> rays_x, rays_y, rays_z;
    interface_->getLensInfo(rays_x, rays_y, rays_z);
    cartesianTransform_.initLensTransform(m_width, HEIGHT, rays_x, rays_y, rays_z);
  }
  else
  {
    result.successful = false;
    result.reason = parameter.get_name() + " value is out of range";
  }
}

void ToFSensor::apply_distance_offset_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &)
{
  auto value = parameter.as_int();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %ld", parameter.get_name().c_str(), value);

  interface_->setOffset(value);
}

void ToFSensor::apply_minimum_amplitude_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &)
{
  auto value = parameter.as_int();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %ld", parameter.get_name().c_str(), value);
  this->minimum_amplitude_ = value;
  interface_->setMinAmplitude(value);
}
void ToFSensor::apply_flip_horizontal_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &)
{
  auto value = parameter.as_bool();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ? "true" : "false"));
  // TODO:  It seems we need to stop and restart streaming to change these params
  interface_->stopStream();
  interface_->setFlipHorizontally(value);
  rclcpp::Parameter stream_type;
  rclcpp::Parameter is_streaming;
  rcl_interfaces::msg::SetParametersResult dummy_result;
  (void)this->get_parameter(STREAMING_STATE, is_streaming);
  (void)this->get_parameter(CAPTURE_MODE, stream_type);
  if (is_streaming.as_bool())
    this->apply_stream_type_param(stream_type, dummy_result);
}
void ToFSensor::apply_flip_vertical_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &)
{
  auto value = parameter.as_bool();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ? "true" : "false"));
  // TODO: It seems we need to stop and restart streaming to change these params
  interface_->stopStream();
  interface_->setFlipVertically(value);
  rclcpp::Parameter stream_type;
  rclcpp::Parameter is_streaming;
  rcl_interfaces::msg::SetParametersResult dummy_result;
  (void)this->get_parameter(STREAMING_STATE, is_streaming);
  (void)this->get_parameter(CAPTURE_MODE, stream_type);
  if (is_streaming.as_bool())
    this->apply_stream_type_param(stream_type, dummy_result);
}
void ToFSensor::apply_binning_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &)
{
  auto value = parameter.as_bool();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ? "true" : "false"));

  interface_->setBinning(value, value);
}
void ToFSensor::apply_sensor_name_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &)
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str());

  interface_->setSensorName(value);
  interface_->storeSettings();
}
void ToFSensor::apply_sensor_location_param(const rclcpp::Parameter &parameter, rcl_interfaces::msg::SetParametersResult &)
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str());

  interface_->setSensorLocation(value);
  interface_->storeSettings();
  this->sensor_location_ = value;
}

void ToFSensor::apply_param(bool& param, const rclcpp::Parameter& parameter)
{
  auto value = parameter.as_bool();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), (value ? "true" : "false"));
  param = value;
}

void ToFSensor::apply_param(int& param, const rclcpp::Parameter& parameter)
{
  auto value = parameter.as_int();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %ld", parameter.get_name().c_str(), value);
  param = value;
}

void ToFSensor::apply_param(std::string& param, const rclcpp::Parameter& parameter)
{
  auto value = parameter.as_string();
  RCLCPP_INFO(this->get_logger(), "Handling parameter \"%s\" : %s", parameter.get_name().c_str(), value.c_str());
  param = value;
}

void ToFSensor::publish_tempData(const tofcore::Measurement_T &frame, const rclcpp::Time &stamp)
{
  const std::array<float, 4> defaultTemps{0.0, 0.0, 0.0, 0.0};
  auto temperatures = frame.sensor_temperatures().value_or(defaultTemps);
  int count = 0;

  for (const auto &i : temperatures)
  {
    sensor_msgs::msg::Temperature tmp;
    tmp.header.stamp = stamp;
    tmp.header.frame_id = this->sensor_location_;
    tmp.temperature = i;
    tmp.variance = 0;
    switch (count)
    {
    case 0:
    {
      sensor_temperature_tl->publish(tmp);
      break;
    }
    case 1:
    {
      sensor_temperature_tr->publish(tmp);
      break;
    }
    case 2:
    {
      sensor_temperature_bl->publish(tmp);
      break;
    }
    case 3:
    {
      sensor_temperature_br->publish(tmp);
      break;
    }
    }
    count++;
  }
}

void ToFSensor::publish_amplData(const tofcore::Measurement_T &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time &stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = this->sensor_location_;
  img.height = static_cast<uint32_t>(frame.height());
  img.width = static_cast<uint32_t>(frame.width());
  img.encoding = sensor_msgs::image_encodings::MONO16;
  img.step = img.width * frame.pixel_size();
  img.is_bigendian = 0;
  auto amplitude_bv = frame.amplitude();
  img.data.resize(amplitude_bv.size() * sizeof(amplitude_bv.data()[0]));
  uint8_t *amplitude_begin = (uint8_t *)amplitude_bv.data();
  std::copy_n(amplitude_begin, img.data.size(), img.data.begin());
  pub.publish(img);
}

void ToFSensor::publish_ambientData(const tofcore::Measurement_T &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time &stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = this->sensor_location_;
  img.height = static_cast<uint32_t>(frame.height());
  img.width = static_cast<uint32_t>(frame.width());
  img.encoding = sensor_msgs::image_encodings::MONO16;
  img.step = img.width * frame.pixel_size();
  img.is_bigendian = 0;
  auto amplitude_bv = frame.ambient();
  img.data.resize(amplitude_bv.size() * sizeof(amplitude_bv.data()[0]));
  uint8_t *amplitude_begin = (uint8_t *)amplitude_bv.data();
  std::copy_n(amplitude_begin, img.data.size(), img.data.begin());
  pub.publish(img);
}

void ToFSensor::publish_distData(const tofcore::Measurement_T &frame, rclcpp::Publisher<sensor_msgs::msg::Image> &pub, const rclcpp::Time &stamp)
{
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = this->sensor_location_;
  img.height = static_cast<uint32_t>(frame.height());
  img.width = static_cast<uint32_t>(frame.width());
  img.encoding = sensor_msgs::image_encodings::MONO16;
  img.step = img.width * frame.pixel_size();
  img.is_bigendian = 1;
  auto distance_bv = frame.distance();
  img.data.resize(distance_bv.size() * sizeof(distance_bv.data()[0]));
  uint8_t *dist_begin = (uint8_t *)distance_bv.data();
  std::copy_n(dist_begin, img.data.size(), img.data.begin());
  pub.publish(img);
}

void ToFSensor::update_hdr_frame(const tofcore::Measurement_T& frame)
{
  // verify hdr dist and amp frames are initialized same size as frame
  if (hdr_dist_frame_.empty() || hdr_dist_frame_.cols != frame.width() || hdr_dist_frame_.rows != frame.height() || hdr_integration_counter_ == 0)
  {
    hdr_dist_frame_ = cv::Mat(frame.height(), frame.width(), CV_16UC1, (void *)frame.distance().begin()).clone();
    hdr_amp_frame_ = cv::Mat(frame.height(), frame.width(), CV_16UC1, (void *)frame.amplitude().begin()).clone();

    cv::Mat amplitude_threshold_mask = hdr_amp_frame_ >= maximum_amplitude_;
    hdr_dist_frame_.setTo(cv::Scalar(0), amplitude_threshold_mask);
    hdr_amp_frame_.setTo(cv::Scalar(0), amplitude_threshold_mask);

    hdr_integration_counter_ = 1;

    return;
  }

  auto it_amp_dst = (unsigned short *) hdr_amp_frame_.datastart;
  auto it_amp_src = frame.amplitude().begin();
  auto it_dist_dst = (unsigned short *) hdr_dist_frame_.datastart;
  auto it_dist_src = frame.distance().begin();

  while (it_dist_src != frame.distance().end())
  {
    if (*it_amp_src > *it_amp_dst && *it_amp_src < maximum_amplitude_ && *it_dist_src > 0 && *it_dist_src < 64000)
    {
      *it_amp_dst = *it_amp_src;
      *it_dist_dst = *it_dist_src;
    }

    it_amp_dst++;
    it_amp_src++;
    it_dist_dst++;
    it_dist_src++;
  }
}

void ToFSensor::publish_pointCloud(const tofcore::Measurement_T &frame, rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &pub, rclcpp::Publisher<tofcore_msgs::msg::IntegrationTime> &cust_pub, const rclcpp::Time &stamp)
{
  cv::Mat dist_frame;
  cv::Mat amp_frame;

  if (hdr_enable_ && hdr_count_ > 1)
  {
    update_hdr_frame(frame);

    if (hdr_integration_counter_ < hdr_count_)
    {
      hdr_integration_counter_++;
      return;
    }

    hdr_integration_counter_ = 0;

    amp_frame = hdr_amp_frame_;
    dist_frame = hdr_dist_frame_;
  }
  else
  {
    dist_frame = cv::Mat(frame.height(), frame.width(), CV_16UC1, (void *)frame.distance().begin());
    amp_frame = cv::Mat(frame.height(), frame.width(), CV_16UC1, (void *)frame.amplitude().begin());
  }

  sensor_msgs::msg::PointCloud2 cloud_msg{};
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = this->sensor_location_;
  cloud_msg.is_dense = true;
  cloud_msg.is_bigendian = false;
  tofcore_msgs::msg::IntegrationTime integration_msg{};
  integration_msg.header.stamp = stamp;
  integration_msg.header.frame_id = this->sensor_location_;
  // Adding this to the message for the auto exposure node
  // Need to check if it exists because this is optional value
  if (frame.integration_time())
  {
    auto integration_times = *std::move(frame.integration_time());
    integration_msg.integration_time = integration_times;
  }

  if (this->median_filter_)
  {
    cv::medianBlur(dist_frame, dist_frame, this->median_kernel_);
  }
  if (this->bilateral_filter_)
  {
    cv::Mat src = cv::Mat::zeros(dist_frame.size(), CV_32FC1);
    cv::Mat dst = cv::Mat::zeros(dist_frame.size(), CV_32FC1);
    dist_frame.convertTo(src, CV_32FC1);
    cv::bilateralFilter(src, dst, this->bilateral_kernel_, this->bilateral_color_, this->bilateral_space_);
    dst.convertTo(dist_frame, CV_16UC1);
  }
  if (this->gradient_filter_)
  {
    // apply gradient filtering to cloud
    cv::Mat grad_x, grad_y;
    cv::Mat dst = cv::Mat::zeros(dist_frame.size(), CV_32FC1);
    dist_frame.convertTo(dst, CV_32FC1);

    // Compute the Laplacian
    cv::Mat laplacian;
    cv::Laplacian(dist_frame, laplacian, CV_64F);

    // Calculate the magnitude of the gradient
    cv::Mat laplacian_abs;
    cv::Mat grad_mask = cv::abs(laplacian) > this->gradient_threshold_;

    cv::Mat mask_valid;
    cv::bitwise_not(grad_mask, mask_valid);

    // ensure enough neighbours of each pixel satisfy the gradient condition
    mask_valid = neighbour_mask(mask_valid, this->gradient_filter_support_);
    cv::Mat mask;
    cv::bitwise_not(mask_valid, mask);
    dist_frame.setTo(cv::Scalar(0), mask);
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.resize(frame.height() * frame.width());
  modifier.setPointCloud2Fields(
      7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "amplitude", 1, sensor_msgs::msg::PointField::UINT16,
      "ambient", 1, sensor_msgs::msg::PointField::INT16,
      "valid", 1, sensor_msgs::msg::PointField::UINT8,
      "distance", 1, sensor_msgs::msg::PointField::UINT16);

  // Note: For some reason setPointCloudFields doesn't set row_step
  //      and resets msg height and m_width so setup them here.
  cloud_msg.height = static_cast<uint32_t>(frame.height());
  cloud_msg.width = static_cast<uint32_t>(frame.width());
  cloud_msg.row_step = frame.width() * 19; // 19 is the size in bytes of all the point cloud fields

  sensor_msgs::PointCloud2Iterator<float> it_x{cloud_msg, "x"};
  sensor_msgs::PointCloud2Iterator<float> it_y{cloud_msg, "y"};
  sensor_msgs::PointCloud2Iterator<float> it_z{cloud_msg, "z"};
  sensor_msgs::PointCloud2Iterator<uint16_t> it_amplitude{cloud_msg, "amplitude"};
  sensor_msgs::PointCloud2Iterator<int16_t> it_ambient{cloud_msg, "ambient"};
  sensor_msgs::PointCloud2Iterator<uint8_t> it_valid{cloud_msg, "valid"};
  sensor_msgs::PointCloud2Iterator<uint16_t> it_phase{cloud_msg, "distance"};

  auto it_d = (const unsigned short *)dist_frame.datastart;
  auto it_a = (const unsigned short *)amp_frame.datastart;
  uint32_t count = 0;
  while (it_d != (const unsigned short *)dist_frame.dataend)
  {
    auto distance = *it_d;
    auto y = count / frame.width();
    auto x = count % frame.width();
    int valid = 0;
    double px, py, pz;
    px = py = pz = 0.1;

    bool invalid = *it_a < this->minimum_amplitude_ || *it_a >= this->maximum_amplitude_;

    if (distance > 0 && distance < 64000 && !invalid)
    {
      if (frame.width() == 160)
        cartesianTransform_.transformPixel(2 * x, 2 * y, distance, px, py, pz);
      else
        cartesianTransform_.transformPixel(x, y, distance, px, py, pz);
      px /= 1000.0; // mm -> m
      py /= 1000.0; // mm -> m
      pz /= 1000.0; // mm -> m
      valid = 1;
    }

    *it_x = px;
    *it_y = py;
    *it_z = pz;
    if (frame.type() == tofcore::Measurement_T::DataType::DISTANCE_AMPLITUDE)
    {
      *it_amplitude = *it_a;
      it_a += 1;
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
    it_d += 1;
  }
  // This is dumb and redundant but I don't want to break rviz viewer
  pub.publish(cloud_msg);
  cust_pub.publish(integration_msg);
}

void ToFSensor::publish_DCSData(const tofcore::Measurement_T &frame, const rclcpp::Time &stamp)
{

  // TODO Need to figure out the best way to publish image meta-data including:
  //   modulation_frequency
  //   integration_time
  //   binning
  //   vled_mv
  //   chip_id
  //
  //  Also need to figure out how to publish an ambient frame which will be required for use with the calibration app.
  //
  // Does feature/add-meta-data-publishers branch have work that should be used for this?

  if (frame.type() == tofcore::Measurement_T::DataType::DCS)
  {
    for (auto i = 0; i != 4; ++i)
    {
      sensor_msgs::msg::Image img;
      img.header.stamp = stamp;
      img.header.frame_id = this->sensor_location_;
      img.height = static_cast<uint32_t>(frame.height());
      // RCLCPP_INFO(this->get_logger(), "Frame Height: %d", frame.height());

      img.width = static_cast<uint32_t>(frame.width());
      img.encoding = sensor_msgs::image_encodings::MONO16;
      img.step = img.width * frame.pixel_size();
      img.is_bigendian = 0;
      auto frame_size = img.step * img.height;
      img.data.resize((frame_size));
      auto begin = reinterpret_cast<const uint8_t *>(frame.dcs(i).begin());
      auto end = begin + (frame_size);
      std::copy(begin, end, img.data.begin());
      pub_dcs_[i]->publish(img);
    }
  }
}

void ToFSensor::updateFrame(const tofcore::Measurement_T &frame)
{
  auto stamp = this->now();
  switch (frame.type())
  {
  case tofcore::Measurement_T::DataType::AMBIENT:
  {
    publish_ambientData(frame, *pub_ambient_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::GRAYSCALE:
  {
    publish_ambientData(frame, *pub_ambient_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::DISTANCE_AMPLITUDE:
  {
    publish_amplData(frame, *pub_amplitude_, stamp);
    publish_distData(frame, *pub_distance_, stamp);
    publish_pointCloud(frame, *pub_pcd_, *pub_integration_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::AMPLITUDE:
  {
    // Probably not the case we just stream amplitude, but its here
    publish_amplData(frame, *pub_amplitude_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::DISTANCE:
  {
    publish_distData(frame, *pub_distance_, stamp);
    publish_pointCloud(frame, *pub_pcd_, *pub_integration_, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::DCS:
  {
    publish_DCSData(frame, stamp);
    publish_tempData(frame, stamp);
    break;
  }
  case tofcore::Measurement_T::DataType::UNKNOWN:
  {
    break;
  }
  }
}

// TODO ability to set different modulation frequencies
void ToFSensor::apply_vsm_settings()
{
  TofComm::VsmControl_T vsmControl{};

  if (hdr_enable_)
  {
    std::vector<std::string> integration_times;
    boost::split(integration_times, hdr_integrations_, boost::is_any_of(", "), boost::token_compress_on);

    RCLCPP_INFO(this->get_logger(), "HDR enabled, %li integrations", integration_times.size());
    RCLCPP_INFO(this->get_logger(), "HDR integrations: %s", hdr_integrations_.c_str());

    vsmControl.m_numberOfElements = integration_times.size();

    hdr_count_ = integration_times.size();
    uint16_t modulationFreqKhz = interface_->getModulation().value_or(12000);

    for (long unsigned int n = 0; n < hdr_count_; ++n)
    {
      try
      {
        vsmControl.m_elements[n].m_integrationTimeUs = std::stoi(integration_times[n]);
        RCLCPP_INFO(this->get_logger(), "VSM Element %ld : %d", n, vsmControl.m_elements[n].m_integrationTimeUs);
      }
      catch (std::invalid_argument &e)
      {
        // if no conversion could be performed
        RCLCPP_INFO(this->get_logger(), "Invalid integration time argument, defaulting to 500us");
        vsmControl.m_elements[n].m_integrationTimeUs = 500;
      }
      catch (std::out_of_range &e)
      {
        // if the converted value would fall out of the range of the result type
        // or if the underlying function (std::strtol or std::strtoull) sets errno
        // to ERANGE.
        RCLCPP_INFO(this->get_logger(), "Out of range, defaulting to 500us");
        vsmControl.m_elements[n].m_integrationTimeUs = 500;
      }
      catch (...)
      {
        // everything else
        RCLCPP_INFO(this->get_logger(), "Some other error, defaulting to 500us");
        vsmControl.m_elements[n].m_integrationTimeUs = 500;
      }
      vsmControl.m_elements[n].m_modulationFreqKhz = modulationFreqKhz;
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "HDR disabled");
    vsmControl.m_numberOfElements = 0;
  }

  interface_->setVsm(vsmControl);

  std::optional<TofComm::VsmControl_T> vsmControlOut = interface_->getVsmSettings();
  if (vsmControlOut)
  {
    RCLCPP_INFO(this->get_logger(), "VSM settings applied, %i elements", vsmControlOut->m_numberOfElements);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to apply VSM settings");
  }
}
