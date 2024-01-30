#pragma once
#include "common.hpp"

#include <opencv2/opencv.hpp>
#include <paddle_api.h>
#include <paddle_image_preprocess.h>

using namespace cv;
using namespace paddle::lite_api;
typedef paddle::lite_api::Tensor Tensor;
typedef paddle::lite_api::DataLayoutType LayoutType;
typedef paddle::lite::utils::cv::FlipParam FlipParam;
typedef paddle::lite::utils::cv::TransParam TransParam;
typedef paddle::lite::utils::cv::ImageFormat ImageFormat;
typedef paddle::lite::utils::cv::ImagePreprocess ImagePreprocess;

inline void fpga_preprocess(cv::Mat img, ModelConfig &config,
                                       std::unique_ptr<Tensor> &tensor) {

  int width = img.cols;
  int height = img.rows;

  if (height > 1080) {
    float fx = img.cols / config.input_width;
    float fy = img.rows / config.input_height;
    Mat resize_mat;
    resize(img, resize_mat, Size(config.input_width, config.input_height), fx,
           fy);
    img = resize_mat;
    height = img.rows;
    width = img.cols;
  }

  int index = 0;

  uint8_t *src = (uint8_t *)malloc(3 * width * height);
  if (img.isContinuous()) {
    memcpy(src, img.data, 3 * width * height * sizeof(uint8_t));
  } else {
    uint8_t *img_data = img.data;
    for (int i = 0; i < img.rows; ++i) {
      src = src + i * (width * 3);
      img_data = img_data + i * (width * 3);
      memcpy(src, img_data, width * 3 * sizeof(uint8_t));
    }
  }
  TransParam tparam;
  tparam.ih = img.rows;
  tparam.iw = img.cols;
  tparam.oh = config.input_height;
  tparam.ow = config.input_width;

  ImagePreprocess image_preprocess(
      ImageFormat::BGR,
      config.format == "RGB" ? ImageFormat::RGB : ImageFormat::BGR, tparam);
  image_preprocess.image_to_tensor(src, tensor.get(), LayoutType::kNHWC,
                                   config.means, config.scales);
  free(src);
}

inline void cpu_preprocess(cv::Mat img, ModelConfig &config,
                                      float *data) {

  Mat resize_img;
  resize(img, resize_img, Size(config.input_width, config.input_height));
  Mat preprocessMat;
  resize_img.convertTo(preprocessMat, CV_32FC3);

  int index = 0;

  for (int row = 0; row < preprocessMat.rows; ++row) {
    float *ptr = (float *)preprocessMat.ptr(row);
    for (int col = 0; col < preprocessMat.cols; col++) {
      float *uc_pixel = ptr;
      float b = uc_pixel[0];
      float g = uc_pixel[1];
      float r = uc_pixel[2];

      if (config.format == "RGB") {
        data[index] = (r - config.means[0]) * config.scales[0];
        data[index + 1] = (g - config.means[1]) * config.scales[1];
        data[index + 2] = (b - config.means[2]) * config.scales[2];
      } else {
        data[index] = (b - config.means[0]) * config.scales[0];
        data[index + 1] = (g - config.means[1]) * config.scales[1];
        data[index + 2] = (r - config.means[2]) * config.scales[2];
      }
      ptr += 3;
      index += 3;
    }
  }
}
