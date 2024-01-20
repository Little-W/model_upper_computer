#pragma once
#include <asm/types.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <iostream>
#include <linux/videodev2.h>
#include <malloc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "common.hpp"
#include "capture.hpp"

struct buffer {
  void *start;
  size_t length;
};

class MultiChannelCamera : public CaptureInterface {
private:
  int fd;
  int width;
  int height;
  std::string dev_name;
  std::string dev_type;
  BlockingQueue<FrameWrapper> queue;
  std::thread worker_thread;

  v4l2_buffer last_buf;
  buffer *buffers;
  int n_buffers;
  bool _loop;
  int log_en;

  void run();
  int read_frame();
  int process_image(void *src, int size);

  int init_mmap(void);
  int init_userp(unsigned int buffer_size);

  int open_device(void);
  int init_device(void);
  int start_capturing(void);

  int stop_capturing(void);
  int release_device(void);
  void close_device(void);

public:
  int stop();
  int start();
  FrameWrapper getFrame();
  std::string getType() { return dev_type; }
  MultiChannelCamera(std::string type, std::string dev)
      : dev_name(dev), dev_type(type) {
    last_buf.m.userptr = 0;
    fd = -1;
    _loop = false;
    log_en = 0;
    n_buffers = 0;
    buffers = nullptr;
    width = 1920;
    height = 1080;
  };
  ~MultiChannelCamera(){};
};
