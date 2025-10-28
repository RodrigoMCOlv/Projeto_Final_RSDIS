#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>

using namespace cv;

ros::Publisher pub_image;
ros::Publisher pub_rgb;

struct Params {
  // Crop rectangle (defaults match your code: Rect(0,220,410,154))
  int crop_x{0};
  int crop_y{220};
  int crop_w{410};
  int crop_h{154};

  // Scanline positions as fractions of cropped image height
  float row_far_ratio{0.5f};  // "far" row (upper)
  //float row_near_ratio{0.80f}; // "near" row (lower)

  // Edge finding
  int margin_px{40};           // avoid side clutter
  int min_gap_px{40};          // min gap between two edges on a row
  int canny_low{30};
  int canny_high{150};

  // Blur
  int blur_ksize{3};           // odd
  double blur_sigma{1.0};

  // HSV thresholds for color detection
  int sat_min{80};             // saturation floor
  int val_min{80};             // value (brightness) floor
  int red_hue_min{0};
  int red_hue_max{10};
  int red_hue2_min{170};
  int red_hue2_max{180};
  int green_hue_min{35};
  int green_hue_max{85};
  int blue_hue_min{90};
  int blue_hue_max{130};

  // EMA smoothing
  float ema_alpha{0.25f};

  // Debug
  bool draw_debug{true};
} g_params;

namespace {
  float line_error = 0;
  // EMA state
  //float e_y_f = 0.0f, e_psi_f = 0.0f;
  //bool have_filter = false;
}

static bool findTwoEdgesInRow(const cv::Mat& edges, int rowIndex,
                              int margin, int min_gap,
                              unsigned int cols_out[2], int& count_out)
{
  count_out = 0;
  const int w = edges.cols;
  const int start = std::max(margin, 0);
  const int end   = std::min(w - margin, w);

  for (int c = start; c < end; ++c) {
    if (edges.at<uchar>(rowIndex, c) != 0) {
      if (count_out == 0) {
        cols_out[count_out++] = c;
      } else if (count_out == 1 && c > int(cols_out[0]) + min_gap) {
        cols_out[count_out++] = c;
        break;
      }
    }
  }
  return (count_out == 2);
}

void image_callback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  // Decode compressed JPEG
  cv::Mat decoded = cv::imdecode(msg->data, cv::IMREAD_COLOR);
  if (decoded.empty()) {
    ROS_WARN_THROTTLE(1.0, "image_processing: empty decode");
    return;
  }

  // Clamp crop to image bounds
  int x = std::max(0, g_params.crop_x);
  int y = std::max(0, g_params.crop_y);
  int w = std::min(g_params.crop_w, decoded.cols - x);
  int h = std::min(g_params.crop_h, decoded.rows - y);
  if (w <= 0 || h <= 0) {
    ROS_WARN_THROTTLE(1.0, "image_processing: invalid crop");
    return;
  }

  cv::Rect crop(x, y, w, h);
  cv::Mat roi = decoded(crop).clone();

//-----------------------------RED-----------------------------

  // Blur
  cv::Mat blurred;
  int k = std::max(1, g_params.blur_ksize);
  if (k % 2 == 0) k += 1;
  cv::GaussianBlur(roi, blurred, cv::Size(k, k), g_params.blur_sigma);

  // Convert once to HSV for all colour thresholds
  cv::Mat hsv;
  cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

  auto clamp = [](int value, int low, int high) {
    return std::max(low, std::min(high, value));
  };

  const int sat_min = clamp(g_params.sat_min, 0, 255);
  const int val_min = clamp(g_params.val_min, 0, 255);
  const int red_hue_min = clamp(g_params.red_hue_min, 0, 180);
  const int red_hue_max = clamp(g_params.red_hue_max, 0, 180);
  const int red_hue2_min = clamp(g_params.red_hue2_min, 0, 180);
  const int red_hue2_max = clamp(g_params.red_hue2_max, 0, 180);
  const int green_hue_min = clamp(g_params.green_hue_min, 0, 180);
  const int green_hue_max = clamp(g_params.green_hue_max, 0, 180);
  const int blue_hue_min = clamp(g_params.blue_hue_min, 0, 180);
  const int blue_hue_max = clamp(g_params.blue_hue_max, 0, 180);

  // Threshold (Red detection - hue wraps at 180, so use two ranges)
  cv::Mat red_mask_low = cv::Mat::zeros(hsv.size(), CV_8U);
  if (red_hue_min <= red_hue_max) {
    cv::inRange(hsv,
                cv::Scalar(red_hue_min, sat_min, val_min),
                cv::Scalar(red_hue_max, 255, 255),
                red_mask_low);
  }
  cv::Mat red_mask_high = cv::Mat::zeros(hsv.size(), CV_8U);
  if (red_hue2_min <= red_hue2_max) {
    cv::inRange(hsv,
                cv::Scalar(red_hue2_min, sat_min, val_min),
                cv::Scalar(red_hue2_max, 255, 255),
                red_mask_high);
  }
  cv::Mat red_mask = cv::Mat::zeros(hsv.size(), CV_8U);
  cv::bitwise_or(red_mask_low, red_mask_high, red_mask);

  // Threshold (Green detection)
  cv::Mat green_mask = cv::Mat::zeros(hsv.size(), CV_8U);
  if (green_hue_min <= green_hue_max) {
    cv::inRange(hsv,
                cv::Scalar(green_hue_min, sat_min, val_min),
                cv::Scalar(green_hue_max, 255, 255),
                green_mask);
  }

  // Threshold (Blue detection)
  cv::Mat blue_mask = cv::Mat::zeros(hsv.size(), CV_8U);
  if (blue_hue_min <= blue_hue_max) {
    cv::inRange(hsv,
                cv::Scalar(blue_hue_min, sat_min, val_min),
                cv::Scalar(blue_hue_max, 255, 255),
                blue_mask);
  }

  // Edges
  cv::Mat edges_red;
  cv::Mat edges_green;
  cv::Mat edges_blue;
  cv::Canny(red_mask, edges_red, g_params.canny_low, g_params.canny_high);
  cv::Canny(green_mask, edges_green, g_params.canny_low, g_params.canny_high);
  cv::Canny(blue_mask, edges_blue, g_params.canny_low, g_params.canny_high);


  // Row indices from ratios
  const int rows_red = edges_red.rows;
  const int cols_red = edges_red.cols;

  const int rows_green = edges_green.rows;
  const int cols_green = edges_green.cols;

  const int rows_blue = edges_blue.rows;
  const int cols_blue = edges_blue.cols;

  int row_far  = std::max(0, std::min(rows_red - 1, int(std::round(g_params.row_far_ratio  * rows_red))));

  // Find two edges on each row
  unsigned int rowFarColsRed[2]  = {0, 0};
  unsigned int rowFarColsGreen[2]  = {0, 0};
  unsigned int rowFarColsBlue[2]  = {0, 0};

  int farCountRed = 0; //, nearCount = 0;
  int farCountGreen = 0;
  int farCountBlue = 0;

  bool detected_red  = findTwoEdgesInRow(edges_red, row_far,  g_params.margin_px, g_params.min_gap_px, rowFarColsRed,  farCountRed);
  bool detected_green  = findTwoEdgesInRow(edges_green, row_far,  g_params.margin_px, g_params.min_gap_px, rowFarColsGreen,  farCountGreen);
  bool detected_blue  = findTwoEdgesInRow(edges_blue, row_far,  g_params.margin_px, g_params.min_gap_px, rowFarColsBlue,  farCountBlue);

  std_msgs::String colour_msg;
  colour_msg.data = "N";

  cv::Mat debug_edges = edges_red.clone();
  cv::bitwise_or(debug_edges, edges_green, debug_edges);
  cv::bitwise_or(debug_edges, edges_blue, debug_edges);

  if (detected_red) {

    colour_msg.data = "R";


    // Debug draw
    if (g_params.draw_debug) {
      cv::circle(debug_edges, cv::Point(int(rowFarColsRed[0]),  row_far),  3, cv::Scalar(255), -1);
      cv::circle(debug_edges, cv::Point(int(rowFarColsRed[1]),  row_far),  3, cv::Scalar(255), -1);
    }
  } else if (detected_green) {
    colour_msg.data = "G";

    if (g_params.draw_debug) {
      cv::circle(debug_edges, cv::Point(int(rowFarColsGreen[0]),  row_far),  3, cv::Scalar(255), -1);
      cv::circle(debug_edges, cv::Point(int(rowFarColsGreen[1]),  row_far),  3, cv::Scalar(255), -1);
    }
  } else if (detected_blue) {
    colour_msg.data = "B";

    if (g_params.draw_debug) {
      cv::circle(debug_edges, cv::Point(int(rowFarColsBlue[0]),  row_far),  3, cv::Scalar(255), -1);
      cv::circle(debug_edges, cv::Point(int(rowFarColsBlue[1]),  row_far),  3, cv::Scalar(255), -1);
    }
  }

  pub_rgb.publish(colour_msg);

  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, debug_edges);
  img_bridge.toImageMsg(img_msg);
  pub_image.publish(img_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgb_detector");
  ros::NodeHandle nh("~");      // private namespace for params
  ros::NodeHandle n;            // public

  // Load parameters (with current defaults)
  nh.param("crop_x_rgb", g_params.crop_x, g_params.crop_x);
  nh.param("crop_y_rgb", g_params.crop_y, g_params.crop_y);
  nh.param("crop_w_rgb", g_params.crop_w, g_params.crop_w);
  nh.param("crop_h_rgb", g_params.crop_h, g_params.crop_h);

  nh.param("row_far_ratio_rgb",  g_params.row_far_ratio,  g_params.row_far_ratio);
  //nh.param("row_near_ratio", g_params.row_near_ratio, g_params.row_near_ratio);

  nh.param("margin_px_rgb",  g_params.margin_px,  g_params.margin_px);
  nh.param("min_gap_px_rgb", g_params.min_gap_px, g_params.min_gap_px);
  nh.param("canny_low_rgb",  g_params.canny_low,  g_params.canny_low);
  nh.param("canny_high_rgb", g_params.canny_high, g_params.canny_high);

  nh.param("blur_ksize_rgb",  g_params.blur_ksize,  g_params.blur_ksize);
  nh.param("blur_sigma_rgb",  g_params.blur_sigma,  g_params.blur_sigma);

  nh.param("sat_min", g_params.sat_min, g_params.sat_min);
  nh.param("val_min", g_params.val_min, g_params.val_min);
  nh.param("red_hue_min", g_params.red_hue_min, g_params.red_hue_min);
  nh.param("red_hue_max", g_params.red_hue_max, g_params.red_hue_max);
  nh.param("red_hue2_min", g_params.red_hue2_min, g_params.red_hue2_min);
  nh.param("red_hue2_max", g_params.red_hue2_max, g_params.red_hue2_max);
  nh.param("green_hue_min", g_params.green_hue_min, g_params.green_hue_min);
  nh.param("green_hue_max", g_params.green_hue_max, g_params.green_hue_max);
  nh.param("blue_hue_min", g_params.blue_hue_min, g_params.blue_hue_min);
  nh.param("blue_hue_max", g_params.blue_hue_max, g_params.blue_hue_max);

  nh.param("ema_alpha_rgb", g_params.ema_alpha, g_params.ema_alpha);
  nh.param("draw_debug_rgb", g_params.draw_debug, g_params.draw_debug);

  // I/O
  ros::Subscriber sub_cam =
      n.subscribe<sensor_msgs::CompressedImage>("/raspicam_node/image/compressed", 1, image_callback);

  pub_image = n.advertise<sensor_msgs::Image>("/processed_image_rgb", 1);
 
  pub_rgb = n.advertise<std_msgs::String>("/colour", 1);


  ros::spin();
  return 0;
}
