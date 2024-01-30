#pragma once
#include "../utils/blocking_queue.h"
#include "common.hpp"
#include <thread>

class Display {
private:
  BlockingQueue<FrameWrapper> queue;
  std::string name;
  std::thread worker_thread;

  int channel_selected;
  bool loop;
  void run();
  static void OnMouseCallBack(int event, int x, int y, int flags, void *param);
  void OnKeyBoardCallBack(int key_value);

public:
  int start();
  int stop();
  void putFrame(FrameWrapper frame_wraper);
  Display(std::string name) : name(name), channel_selected(0), loop(false) {}

  ~Display() {}
};
