#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>

using namespace cv;

ros::Publisher pub_image;
//ros::Publisher pub_error_legacy;
//ros::Publisher pub_error_lat;
//ros::Publisher pub_error_head;
ros::Publisher pub_error;

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

  // Threshold (RGB inRange)
  int thr_low{0};              // 0..255
  int thr_high{100};           // 0..255

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

  // Blur
  cv::Mat blurred;
  int k = std::max(1, g_params.blur_ksize);
  if (k % 2 == 0) k += 1;
  cv::GaussianBlur(roi, blurred, cv::Size(k, k), g_params.blur_sigma);

  // Threshold (RGB inRange as in your code)
  cv::Mat rgb, mask;
  cv::cvtColor(blurred, rgb, cv::COLOR_BGR2RGB);
  cv::inRange(rgb,
              cv::Scalar(g_params.thr_low, g_params.thr_low, g_params.thr_low),
              cv::Scalar(g_params.thr_high, g_params.thr_high, g_params.thr_high),
              mask);

  // Edges
  cv::Mat edges;
  cv::Canny(mask, edges, g_params.canny_low, g_params.canny_high);

  // Row indices from ratios
  const int rows = edges.rows;
  const int cols = edges.cols;
  int row_far  = std::max(0, std::min(rows - 1, int(std::round(g_params.row_far_ratio  * rows))));
  //int row_near = std::max(0, std::min(rows - 1, int(std::round(g_params.row_near_ratio * rows))));
  // Ensure far is above near (smaller y)
  //if (row_far > row_near) std::swap(row_far, row_near);

  // Find two edges on each row
  unsigned int rowFarCols[2]  = {0, 0};
  //unsigned int rowNearCols[2] = {0, 0};
  int farCount = 0; //, nearCount = 0;

  bool farOK  = findTwoEdgesInRow(edges, row_far,  g_params.margin_px, g_params.min_gap_px, rowFarCols,  farCount);
  //bool nearOK = findTwoEdgesInRow(edges, row_near, g_params.margin_px, g_params.min_gap_px, rowNearCols, nearCount);

  //float e_y = 0.0f;
  //float e_psi = 0.0f;
  //bool have_measure = false;

  if (farOK) {
    // centers
    float center_far      = 0.5f * (rowFarCols[0]  + rowFarCols[1]);
    float bottom_center_x = 0.5f * static_cast<float>(cols);

    line_error = (bottom_center_x - center_far) / static_cast<float>(cols);


    // Debug draw
    if (g_params.draw_debug) {
      cv::circle(edges, cv::Point(int(rowFarCols[0]),  row_far),  3, cv::Scalar(255), -1);
      cv::circle(edges, cv::Point(int(rowFarCols[1]),  row_far),  3, cv::Scalar(255), -1);

      const cv::Point far_point(static_cast<int>(center_far), row_far);
      const cv::Point bottom_center(static_cast<int>(bottom_center_x), rows - 1);

      cv::circle(edges, bottom_center, 3, cv::Scalar(255), -1);

      cv::line(edges, far_point, bottom_center, cv::Scalar(255), 1);
    }
  } else {
    // No measurement: keep last EMA (do nothing) or slowly decay towards 0
    // e_y_f *= 0.98f; e_psi_f *= 0.98f; // optional
  }

  // Publish errors
  std_msgs::Float32 error_msg;//msg_lat, msg_head, msg_legacy;
  error_msg.data = line_error;


  pub_error.publish(error_msg);


  // Publish debug image
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, edges);
  img_bridge.toImageMsg(img_msg);
  pub_image.publish(img_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_processing");
  ros::NodeHandle nh("~");      // private namespace for params
  ros::NodeHandle n;            // public

  // Load parameters (with current defaults)
  nh.param("crop_x", g_params.crop_x, g_params.crop_x);
  nh.param("crop_y", g_params.crop_y, g_params.crop_y);
  nh.param("crop_w", g_params.crop_w, g_params.crop_w);
  nh.param("crop_h", g_params.crop_h, g_params.crop_h);

  nh.param("row_far_ratio",  g_params.row_far_ratio,  g_params.row_far_ratio);
  //nh.param("row_near_ratio", g_params.row_near_ratio, g_params.row_near_ratio);

  nh.param("margin_px",  g_params.margin_px,  g_params.margin_px);
  nh.param("min_gap_px", g_params.min_gap_px, g_params.min_gap_px);
  nh.param("canny_low",  g_params.canny_low,  g_params.canny_low);
  nh.param("canny_high", g_params.canny_high, g_params.canny_high);

  nh.param("blur_ksize",  g_params.blur_ksize,  g_params.blur_ksize);
  nh.param("blur_sigma",  g_params.blur_sigma,  g_params.blur_sigma);

  nh.param("thr_low",   g_params.thr_low,   g_params.thr_low);
  nh.param("thr_high",  g_params.thr_high,  g_params.thr_high);

  nh.param("ema_alpha", g_params.ema_alpha, g_params.ema_alpha);
  nh.param("draw_debug", g_params.draw_debug, g_params.draw_debug);

  // I/O
  ros::Subscriber sub_cam =
      n.subscribe<sensor_msgs::CompressedImage>("/raspicam_node/image/compressed", 1, image_callback);

  pub_image        = n.advertise<sensor_msgs::Image>("/Processed_Image", 1);
  //pub_error_legacy = n.advertise<std_msgs::Float32>("/error", 1);
  //pub_error_lat    = n.advertise<std_msgs::Float32>("/line_lateral_error", 1);
  //pub_error_head   = n.advertise<std_msgs::Float32>("/line_heading_error", 1);

  pub_error = n.advertise<std_msgs::Float32>("/error", 1);

  ros::spin();
  return 0;
}
