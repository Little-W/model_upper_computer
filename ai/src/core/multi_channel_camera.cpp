
#include "multi_channel_camera.hpp"

#include "../utils/multi_channel_camera_util.hpp"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

int errno_exit(const char *s) {
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  return -1;
}

int xioctl(int fh, int request, void *arg) {
  int r = 0;
  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);
  return r;
}

int MultiChannelCamera::process_image(void *src, int size) {
  int ret = 0;
  struct frame_info_t frame_info;
  ret = decode_frame_header((const uint8_t *)src, size, &frame_info);
  if (ret != 0) {
    printf("decode_frame_info failed. [%d]\n", ret);
    return -1;
  }
  // For debug
  if (log_en)
    printf("[MultiChannelCameraLog]:ch_num [%d] frame_num[%d].\n",
           frame_info.ch_num, frame_info.frame_num);
  // For debug
  {
    static uint32_t frame_num[16] = {0};
    static uint8_t first_flag[16] = {0};
    if (first_flag[frame_info.ch_num] == 1) {
      if (frame_num[frame_info.ch_num] + 1 != frame_info.frame_num) {
        printf(
            "[MultiChannelCameraLog] Frame Num Nonsequence: ch_num [%d] last "
            "frame_num[%d]-frame_num[%d].\n",
            frame_info.ch_num, frame_num[frame_info.ch_num],
            frame_info.frame_num);
      }
      frame_num[frame_info.ch_num] = frame_info.frame_num;
    } else {
      first_flag[frame_info.ch_num] = 1;
      frame_num[frame_info.ch_num] = frame_info.frame_num;
    }
  }
  cv::Mat image(height, width, CV_8UC3, (unsigned char *)src);
  cv::Mat image_clone = image.clone();

  if (frame_info.type == VIDEO_FRAME_NORMAL) {
    if (queue.Size() >= 5) {
      queue.Take();
    }
    queue.Put({image_clone, frame_info.ch_num});
  } else {
    /*  注意：因为 cache的原因，对于空帧 也需要内存拷贝
        否则回影响后面空帧的数据接收导致 空帧的帧头的解析。
    */
    image_clone.release();
    printf("[MultiChannelCameraLog]:Recv Null Frame : ch_num [%d] "
           "frame_num[%d].\n",
           frame_info.ch_num, frame_info.frame_num);
  }
  return 0;
}

int MultiChannelCamera::read_frame() {
  struct v4l2_buffer buf;
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;

  if (0 != xioctl(fd, VIDIOC_DQBUF, &buf)) {
    switch (errno) {
    case EAGAIN:
      return 0;
    case EIO:
      /* Could ignore EIO, see spec. */
      /* fall through */
    default:
      return errno_exit("VIDIOC_DQBUF");
    }
  }
  /*
   * https://www.kernel.org/doc/html/v4.9/media/uapi/v4l/vidioc-qbuf.html
   * 注意：
   *   因为 VIDIOC_QBUF 接口返回的数据是 正在拷贝的bufer,
   * 所以此处使用上次的获得的bufer 或者延迟 一帧的时间再拷贝数据
   *
   *   其他摄像头程序是不需要这些操作的：上一帧和本帧基本无差别
   *   本程序的适用情况： 多个通道的视频公用同一个 通道
   */
  if (last_buf.m.userptr != 0) {
    // invalidate_device_memeory(buffers[last_buf.index].start,last_buf.bytesused);
    process_image(buffers[last_buf.index].start, last_buf.bytesused);
    // memset(buffers[last_buf.index].start,0,last_buf.bytesused);
    if (-1 == xioctl(fd, VIDIOC_QBUF, &last_buf)) {
      return errno_exit("VIDIOC_QBUF");
    }
  }

  last_buf = buf;
  return 0;
}

void MultiChannelCamera::run() {
  int r = 0;
  while (_loop) {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(fd + 1, &fds, NULL, NULL, &tv);
    if (-1 == r) {
      if (EINTR == errno)
        continue;
      errno_exit("select");
    }
    if (0 == r) {
      fprintf(stderr, "select timeout,wait %lu s\n", tv.tv_sec);
      continue;
    }
    if (read_frame())
      break;
  }
  std::cout << "MultiChannelCamera run over .\n";
}

int MultiChannelCamera::stop_capturing(void) {
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)) {
    return errno_exit("VIDIOC_STREAMOFF");
  }
  return 0;
}

int MultiChannelCamera::start_capturing(void) {
  unsigned int i = 0;
  enum v4l2_buf_type type;

  for (i = 0; i < n_buffers; ++i) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_USERPTR;
    buf.index = i;
    buf.m.userptr = (unsigned long)buffers[i].start;
    buf.length = buffers[i].length;

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
      fprintf(stderr, "%s ioctl error: VIDIOC_QBUF\\n", dev_name.c_str());
      return -1;
    }
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)) {
    fprintf(stderr, "%s ioctl error: VIDIOC_STREAMON\\n", dev_name.c_str());
    return -1;
  }
  return 0;
}

int MultiChannelCamera::release_device(void) {
  unsigned int i = 0;
  for (i = 0; i < n_buffers; ++i) {
    if (-1 == release_device_memory(buffers[i].start, buffers[i].length)) {
      fprintf(stderr, "%s release_device_memory ...\\n", dev_name.c_str());
      return -1;
    }
  }
  deint_device_memory();
  free(buffers);
  return 0;
}

int MultiChannelCamera::init_mmap(void) {
  struct v4l2_requestbuffers req;
  CLEAR(req);

  req.count = 3;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr,
              "%s does not support "
              "memory mappingn",
              dev_name.c_str());
      return -1;
    } else {
      fprintf(stderr, "%s ioctl error: VIDIOC_REQBUFS\\n", dev_name.c_str());
      return -1;
    }
  }

  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name.c_str());
    return -1;
  }
  buffers = (struct buffer *)calloc(req.count, sizeof(*buffers));
  if (!buffers) {
    fprintf(stderr, "Out of memory\\n");
    return -1;
  }

  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)) {
      fprintf(stderr, "%s ioctl error: VIDIOC_QUERYBUF\\n", dev_name.c_str());
      return -1;
    }

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start =
        mmap(NULL /* start anywhere */, buf.length,
             PROT_READ | PROT_WRITE /* required */,
             MAP_SHARED /* recommended */, fd, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start) {
      fprintf(stderr, "%s mmap error\\n", dev_name.c_str());
      return -1;
    }
  }
  return 0;
}

int MultiChannelCamera::init_userp(unsigned int buffer_size) {
#define CACHE_BUF_SIZE (4)
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.count = CACHE_BUF_SIZE;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support user pointer i/o/n", dev_name.c_str());
      return -1;
    } else {
      fprintf(stderr, "%s ioctl error: VIDIOC_REQBUFS\\n", dev_name.c_str());
      return -1;
    }
  }

  int ret = init_device_memory();
  if (ret < 0) {
    fprintf(stderr, "%s init_device_memory ...\\n", dev_name.c_str());
    return -1;
  }

  buffer_size = width * height * 3;

  buffers = (struct buffer *)calloc(CACHE_BUF_SIZE, sizeof(*buffers));
  if (!buffers) {
    fprintf(stderr, "Out of memory/n");
    return -1;
  }

  for (n_buffers = 0; n_buffers < CACHE_BUF_SIZE; ++n_buffers) {
    buffers[n_buffers].length = buffer_size;
    buffers[n_buffers].start = (void *)malloc_device_memory(buffer_size);
    if (!buffers[n_buffers].start) {
      fprintf(stderr, "Out of memory/n");
      return -1;
    }
  }
  return 0;
}

int MultiChannelCamera::init_device(void) {
  struct v4l2_capability cap;
  struct v4l2_format fmt;
  unsigned int min = 0;

  if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s is no V4L2 device\\n", dev_name.c_str());
      return -1;
    } else {
      fprintf(stderr, "%s ioctl error: VIDIOC_QUERYCAP\\n", dev_name.c_str());
      return -1;
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device\\n", dev_name.c_str());
    return -1;
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    fprintf(stderr, "%s does not support streaming i/o\\n", dev_name.c_str());
    return -1;
  }
  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
    fprintf(stderr, "%s ioctl error: VIDIOC_S_FMT\\n", dev_name.c_str());
    return -1;
  }

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 3;
  if (fmt.fmt.pix.bytesperline < min) {
    fmt.fmt.pix.bytesperline = min;
  }
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min) {
    fmt.fmt.pix.sizeimage = min;
  }

  width = fmt.fmt.pix.width;
  height = fmt.fmt.pix.height;

  return init_userp(fmt.fmt.pix.sizeimage);
}

void MultiChannelCamera::close_device(void) {
  if (fd > 0) {
    close(fd);
  }
  fd = -1;
}

int MultiChannelCamera::open_device(void) {
  struct stat st;
  if (-1 == stat(dev_name.c_str(), &st)) {
    fprintf(stderr, "Cannot identify '%s': %d\n", dev_name.c_str(), errno);
    return -1;
  }

  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "%s is no devicen", dev_name.c_str());
    return -1;
  }

  fd = open(dev_name.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == fd) {
    fprintf(stderr, "Cannot open '%s': %d\n", dev_name.c_str(), errno);
    return -1;
  }
  return 0;
}

int MultiChannelCamera::start() {
  int ret = 0;
  ret = open_device();
  if (ret != 0) {
    return ret;
  }
  ret = init_device();
  if (ret != 0) {
    return ret;
  }
  start_capturing();
  if (ret != 0) {
    return ret;
  }
  _loop = true;
  worker_thread = std::thread(&MultiChannelCamera::run, this);
  return 0;
}

int MultiChannelCamera::stop(void) {
  _loop = false;
  worker_thread.join();
  stop_capturing();
  release_device();
  close_device();
  return 0;
}

FrameWrapper MultiChannelCamera::getFrame() { return queue.Take(); }