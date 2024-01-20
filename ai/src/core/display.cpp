#include "display.hpp"
#define IPC_CAMERA_CHANNEL_COUNT (16)
#define GUI_WAITING_JPG_PATH "../../res/pictures/waiting.jpg"
#define GUI_STARTING_JPG_PATH "../../res/pictures/starting.jpg"

void Display::OnMouseCallBack(int event, int x, int y, int flags, void *param) {
  Display *display = (Display *)param;
  switch (event) {
  case CV_EVENT_LBUTTONDOWN:
    if (display->channel_selected <= 0) {
      display->channel_selected = (IPC_CAMERA_CHANNEL_COUNT - 1);
    } else {
      display->channel_selected = display->channel_selected - 1;
    }
    printf("[Left Mouse Push],change ch to -> %d\n", display->channel_selected);
    break;

  case CV_EVENT_RBUTTONDOWN:
    if (display->channel_selected >= (IPC_CAMERA_CHANNEL_COUNT - 1)) {
      display->channel_selected = 0;
    } else {
      display->channel_selected = display->channel_selected + 1;
    }
    printf("[Right Mouse Push],change ch to -> %d\n",
           display->channel_selected);
    break;
  case CV_EVENT_LBUTTONUP:
  case CV_EVENT_MOUSEMOVE:
  default:
    break;
  }
}

void Display::OnKeyBoardCallBack(int key_value) {
#define KEY_BOARD_ARROW_LEFT (65361)
#define KEY_BOARD_ARROW_RIGHT (65363)

  switch (key_value) {
  case KEY_BOARD_ARROW_LEFT:
    if (channel_selected <= 0) {
      channel_selected = (IPC_CAMERA_CHANNEL_COUNT - 1);
    } else {
      channel_selected--;
    }
    printf("[Left KeyBoard Push],change ch to -> %d\n", channel_selected);
    break;

  case KEY_BOARD_ARROW_RIGHT:
    if (channel_selected >= (IPC_CAMERA_CHANNEL_COUNT - 1)) {
      channel_selected = 0;
    } else {
      channel_selected++;
    }
    printf("[Right KeyBoard Push],change ch to -> %d\n", channel_selected);
    break;
  default:
    break;
  }
}

void Display::run() {
#define EACH_WAIT_TIME_MS (20)

  bool recv_data = false;
  char channel_selected_buf[10];
  int channel_selected_last = -1;
  cv::Mat display_mat;
  

  while (loop) {
    FrameWrapper frame_wrapper;
    OnKeyBoardCallBack(cv::waitKeyEx(EACH_WAIT_TIME_MS));
    if (queue.Size() > 1) {
      recv_data = true;
      frame_wrapper = queue.Take();
    } else {
      recv_data = false;
    }

    snprintf(channel_selected_buf, sizeof(channel_selected_buf), "Ch%d",
             channel_selected);

    if ((recv_data) && (frame_wrapper.channel == channel_selected)) {
      cv::Point ch_display_point(frame_wrapper.frame.cols / 2, 40);
      cv::putText(frame_wrapper.frame, channel_selected_buf, ch_display_point,
                  cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(180, 180, 180), 4);
      if (frame_wrapper.frame.cols > 1280) {
        cv::resize(frame_wrapper.frame, display_mat, cv::Size(1280, 720));
        cv::imshow(name, display_mat);
      } else {
        cv::imshow(name, frame_wrapper.frame);
      }
    } else {
      if (channel_selected_last != channel_selected) {
        cv::Mat waiting_mat = cv::imread(GUI_WAITING_JPG_PATH);
        cv::Point point(waiting_mat.cols / 2, 40);
        cv::putText(waiting_mat, channel_selected_buf, point,
                    cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 0), 4);

        cv::resize(waiting_mat, display_mat, cv::Size(1280, 720));
        cv::imshow(name, display_mat);
      }
    }

    channel_selected_last = channel_selected;
  }
}

int Display::start() {

  cv::namedWindow(name, CV_WINDOW_NORMAL);
  cv::setWindowProperty(name, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  cv::setMouseCallback(name, OnMouseCallBack, this);
  std::ifstream is(GUI_STARTING_JPG_PATH);

  if (is.good()) {
    cv::Mat display_mat;
    cv::Mat initial_mat = cv::imread(GUI_STARTING_JPG_PATH);
    cv::resize(initial_mat, display_mat, cv::Size(1280, 720));

    cv::imshow(name, display_mat);
    cv::waitKeyEx(1);
  } else {
    printf("Display::start not find pictures: %s,for debug.\n",
           GUI_STARTING_JPG_PATH);
  }
  loop = true;
  worker_thread = std::thread(&Display::run, this);
  std::cout << "Display Init Success !!!" << std::endl;
  return 0;
}

int Display::stop() {
  loop = false;
  worker_thread.join();
  return 0;
}

void Display::putFrame(FrameWrapper frame_wraper) {
  if (queue.Size() > 5) {
    queue.Take();
  }
  queue.Put(frame_wraper);
}