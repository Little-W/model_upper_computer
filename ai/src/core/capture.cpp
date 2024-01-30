#include "capture.hpp"
#include "multi_channel_camera.hpp"
#include <opencv2/opencv.hpp>
#include <unistd.h>

class USBCamera : public CaptureInterface {
private:
  std::string dev_name;
  std::string dev_type;
  BlockingQueue<FrameWrapper> queue;
  std::shared_ptr<cv::VideoCapture> cap;
  std::thread worker_thread;
  bool loop;

  void run();

public:
  int stop();
  int start();
  FrameWrapper getFrame();
  std::string getType() { return dev_type; }

  USBCamera(std::string type, std::string dev);
  ~USBCamera(){};
};

USBCamera::USBCamera(std::string type, std::string dev)
    : dev_name(dev), dev_type(type) {
  loop = false;
}

int USBCamera::start() {
  cap = std::make_shared<cv::VideoCapture>(dev_name);
  if (!cap->isOpened()) {
    return -1;
  }
  loop = true;
  worker_thread = std::thread(&USBCamera::run, this);
  return 0;
}

int USBCamera::stop() {
  loop = false;
  worker_thread.join();
  if (cap->isOpened()) {
    cap->release();
  }
  return 0;
}

void USBCamera::run() {
  while (loop) {
    cv::Mat frame;
    if (!cap->read(frame)) {
      std::cout << "Warninig:UsbCamera read failed, wait 2s and continue.\n";
      usleep(2000);
      continue;
    }
    if (queue.Size() >= 5) {
      queue.Take();
    }
    queue.Put({frame, 0});
  }
}

FrameWrapper USBCamera::getFrame() { return queue.Take(); }

class RTSPCamera : public CaptureInterface {
private:
  std::string dev_name;
  std::string dev_type;
  BlockingQueue<FrameWrapper> queue;
  std::shared_ptr<cv::VideoCapture> cap;
  std::thread worker_thread;
  int loop;

  void run();

public:
  int stop();
  int start();
  FrameWrapper getFrame();
  std::string getType() { return dev_type; }

  RTSPCamera(std::string type, std::string dev);
  ~RTSPCamera(){};
};

RTSPCamera::RTSPCamera(std::string type, std::string dev)
    : dev_name(dev), dev_type(type) {
  loop = false;
}

int RTSPCamera::start() {
  cap = std::make_shared<cv::VideoCapture>(dev_name);
  if (!cap->isOpened()) {
    return -1;
  }
  loop = true;
  worker_thread = std::thread(&RTSPCamera::run, this);
  return 0;
}

int RTSPCamera::stop() {
  loop = false;
  worker_thread.join();
  if (cap->isOpened()) {
    cap->release();
  }
  return 0;
}

void RTSPCamera::run() {
  while (loop) {
    cv::Mat frame;
    if (!cap->read(frame)) {
      std::cout << "Warninig:RTSPCamera read failed, wait 2s and continue.\n";
      usleep(2000);
      continue;
    }
    if (queue.Size() >= 5) {
      queue.Take();
    }
    queue.Put({frame, 0});
  }
}

FrameWrapper RTSPCamera::getFrame() { return queue.Take(); }

class ImageReader : public CaptureInterface {
private:
  std::string image_dir;
  std::string dev_type;
  cv::Mat image;

public:
  int stop();
  int start();
  FrameWrapper getFrame();
  std::string getType() { return dev_type; }

  ImageReader(std::string type, std::string dir);
  ~ImageReader(){};
};

ImageReader::ImageReader(std::string type, std::string dir)
    : image_dir(dir), dev_type(type) {}

int ImageReader::start() {
  image = cv::imread(image_dir);
  if (image.empty()) {
    return -1;
  }
  return 0;
}

int ImageReader::stop() {
  image.release();
  return 0;
}

FrameWrapper ImageReader::getFrame() { return {image, 0}; }

std::shared_ptr<CaptureInterface> createCapture(std::string type,
                                                std::string path) {
  std::shared_ptr<CaptureInterface> capture = nullptr;

  if (type == "usb_camera") {
    capture = make_shared<USBCamera>(type, path);
  } else if (type == "image") {
    capture = make_shared<ImageReader>(type, path);
  } else if (type == "rtsp_camera") {
    capture = make_shared<RTSPCamera>(type, path);
  } else if (type == "multichannel_camera") {
    capture = make_shared<MultiChannelCamera>(type, path);
  } else {
    std::cout << "Error !!!! createCapture: unsupport capture tyep:" << type
              << ", support type is: image or usb_camera or rtsp_camera or "
                 "multichannel_camera \n";
    return nullptr;
  }
  if (capture->start()) {
    std::cout << "Error !!!! createCapture: start caputure failed. type: "
              << type << " path: " << path << "\n";
    return nullptr;
  } else {
    std::cout << "CreateCapture Success !!! type: [" << type << "] path: ["
              << path << "]\n";
  }
  return capture;
}
