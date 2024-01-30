#pragma once
#include "../utils/blocking_queue.h"
#include "common.hpp"
#include <opencv2/opencv.hpp>
#include <thread>

class CaptureInterface {
public:
  virtual int stop() = 0;
  virtual int start() = 0;
  virtual std::string getType() = 0;
  virtual FrameWrapper getFrame() = 0;
  virtual ~CaptureInterface() = default;
};

std::shared_ptr<CaptureInterface> createCapture(std::string type, std::string path);