// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <frontend/ImageAndExposure.h>
#include <frontend/FullSystem.h>

std::string vignette = "/media/gaoxiang/Data1/Dataset/TUM-MONO/sequence_31/vignette.png";
std::string gammaCalib = "/media/gaoxiang/Data1/Dataset/TUM-MONO/sequence_31/pcalib.txt";
std::string source = "/media/gaoxiang/Data1/Dataset/TUM-MONO/sequence_31/";
std::string calib = "/media/gaoxiang/Data1/Dataset/TUM-MONO/sequence_31/camera.txt";
std::string output_file = "./results.txt";
std::string vocPath = "/home/smartrollerz/SmartRollerz-SLAM/slam/ldso_slam/vocab/orbvoc.dbow3";

class MinimalSubscriber : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  std::shared_ptr<FullSystem> fullSystem;
  int counter;

public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    counter = 0;

    int playbackSpeed = 0;    // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
    setting_desiredImmatureDensity = 1500;
    setting_desiredPointDensity = 2000;
    setting_minFrames = 5;
    setting_maxFrames = 7;
    setting_maxOptIterations = 6;
    setting_minOptIterations = 1;
    setting_logStuff = false;

    setting_enableLoopClosing = true;

    printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
    setting_photometricCalibration = 0;
    setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
    setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).

    std::shared_ptr<ORBVocabulary> voc(new ORBVocabulary());
    voc->load(vocPath);

    fullSystem = std::make_shared<FullSystem>(voc);
    fullSystem->setGammaFunction(0);
    fullSystem->linearizeOperation = (playbackSpeed == 0);

    std::shared_ptr<PangolinDSOViewer> viewer = nullptr;
    viewer = std::shared_ptr<PangolinDSOViewer>(new PangolinDSOViewer(wG[0], hG[0], false));
    fullSystem->setViewer(viewer);

    auto topic_callback =
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
        // Konvertiere ROS Image -> OpenCV
        try {
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
          cv::Mat img = cv_ptr->image;

          // L-DSO erwartet Graustufen float image (häufig). Beispielkonversion:
          cv::Mat gray;
          if (img.channels() == 3) cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
          else gray = img;

          // convert to float
          cv::Mat img_f;
          gray.convertTo(img_f, CV_32F, 1.0/255.0);
          
          if (img_f.type() != CV_32F) img_f.convertTo(img_f, CV_32F);
          if (!img_f.isContinuous()) img_f = img_f.clone();

          int width = img_f.cols;   // w
          int height = img_f.rows;  // h
          double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

          ldso::ImageAndExposure *frame = new ldso::ImageAndExposure(width, height, timestamp);
          // frame->image was allocated in constructor
          std::memcpy(frame->image, img_f.ptr<float>(), static_cast<size_t>(width) * height * sizeof(float));

          // evtl. exposure_time setzen
          frame->exposure_time = 1.0f; // oder aus msg, falls vorhanden

          fullSystem->addActiveFrame(frame, counter);
          counter++;

          delete frame;

          RCLCPP_INFO(this->get_logger(), "Image " + std::to_string(counter) + " received and forwarded to LDSO");
        } catch (const cv_bridge::Exception & e) {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
      };
    subscription_ =
      this->create_subscription<sensor_msgs::msg::Image>("/camera/image/raw", 10, topic_callback);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
