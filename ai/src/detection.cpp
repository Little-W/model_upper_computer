/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file icar.cpp
 * @author Leo
 * @brief 智能汽车-顶层框架（TOP）
 * @version 0.1
 * @date 2023-12-25
 * @copyright Copyright (c) 2024
 *
 */
#include <shared_mem.h>
#include <unistd.h>
#include <chrono>
#include <csignal>
#include <common.hpp>     //公共类方法文件
#include <detection.hpp>  //百度Paddle框架移动端部署
// #include "../include/uart.hpp"       //串口通信驱动
// #include "controlcenter.cpp"         //控制中心计算类
// #include "detection/bridge.cpp"      //AI检测：坡道区
// #include "detection/danger.cpp"      //AI检测：危险区
// #include "detection/parking.cpp"     //AI检测：停车区
// #include "detection/racing.cpp"      //AI检测：追逐区
// #include "detection/rescue.cpp"      //AI检测：救援区
// #include "motion.cpp"                //智能车运动控制类
// #include "preprocess.cpp"            //图像预处理类
// #include "recognition/crossroad.cpp" //十字道路识别与路径规划类
// #include "recognition/ring.cpp"      //环岛道路识别与路径规划类
// #include "recognition/tracking.cpp"  //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace cv;

#define THRESH 1

bool stop = false;
void callback(int signum){
    cerr << "AI received signal, quit!" << endl;
    stop = true;
}
cv::VideoWriter wri;
void boundaryCorrection(PredictResult &r, int width_range, int height_range) {
#define MARGIN_PIXELS (2)
  r.width = (r.width > (width_range - r.x - MARGIN_PIXELS))
                ? (width_range - r.x - MARGIN_PIXELS)
                : r.width;
  r.height = (r.height > (height_range - r.y - MARGIN_PIXELS))
                 ? (height_range - r.y - MARGIN_PIXELS)
                 : r.height;
                 
  r.x = (r.x < MARGIN_PIXELS) ? MARGIN_PIXELS : r.x;
  r.y = (r.y < MARGIN_PIXELS) ? MARGIN_PIXELS : r.y;
}
void drawResults(cv::Mat &inputFrame, std::vector<PredictResult> &results) {
  for (int i = 0; i < results.size(); ++i) {
    PredictResult r = results[i];
    // boundaryCorrection(r, inputFrame.cols, inputFrame.rows);
    if (r.type >= 0 && r.type < 6) {//6 attention
      cv::Point origin(r.x, r.y);
      // std::string label_name = g_model_config->labels[r.type];
      // cv::putText(inputFrame, label_name, origin, cv::FONT_HERSHEY_PLAIN, 2,
      //             cv::Scalar(255, 0, 0, 255), 2);
    }
    cv::Rect rect(r.x, r.y, r.width, r.height);
    cv::rectangle(inputFrame, rect, Scalar(0, 0, 224), 2);

  }
    // cerr<<"img_clos" << inputFrame.cols << "img rows" << inputFrame.rows << endl;
    // cerr<<"draw done!"<<endl;
}
string model = "../res/model/yolov3_mobilenet_v1"; // 模型路径
string video = "../res/samples/demo.mp4";          // 视频路径
float score = 0.5;
int main(int argc, char const *argv[]) {
  // Preprocess preprocess;    // 图像预处理类
  // Motion motion;            // 运动控制类
  // Tracking tracking;        // 赛道识别类
  // Crossroad crossroad;      // 十字道路识别类
  // Ring ring;                // 环岛识别类
  // Bridge bridge;            // 坡道区检测类
  // Parking parking;          // 停车区检测类
  // Danger danger;            // 危险区检测类
  // Rescue rescue;            // 救援区检测类
  // Racing racing;            // 追逐区检测类
  // ControlCenter ctrlCenter; // 控制中心计算类
  // Display display(4);       // 初始化UI显示窗口
  // VideoCapture capture;     // Opencv相机类

  // 目标检测类(AI模型文件)
  cerr<< "problem 1"<<endl;
  shared_ptr<Detection> detection = make_shared<Detection>(model);
  detection->score = score; // AI检测置信度
  cerr<< "problem 2"<<endl;
  // // USB转串口初始化： /dev/ttyUSB0
  // shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
  // int ret = uart->open();
  // if (ret != 0) {
  //   printf("[Error] Uart Open failed!\n");
  //   return -1;
  // }
  // uart->startReceive(); // 启动数据接收子线程

  // USB摄像头初始化
  // if (motion.params.debug)
  //   capture = VideoCapture(motion.params.video); // 打开本地视频
  // else
  //   capture = VideoCapture("/dev/video0"); // 打开摄像头
  // if (!capture.isOpened()) {
  //   printf("can not open video device!!!\n");
  //   return 0;
  // }
  // capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像分辨率
  // capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像分辨率

  // // 等待按键发车
  // if (!motion.params.debug) {
  //   printf("--------------[等待按键发车!]-------------------\n");
  //   uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
  //   while (!uart->keypress)
  //     waitKey(300);
  //   while (ret < 10) // 延时3s
  //   {
  //     uart->carControl(0, PWMSERVOMID); // 通信控制车辆停止运动
  //     waitKey(300);
  //     ret++;
  //   }
  //   uart->keypress = false;
  //   uart->buzzerSound(uart->BUZZER_START); // 祖传提示音效
  // }

  // // 初始化参数
  // Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
  // Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
  long preTime;
  // Mat img;
  wri.open("/home/edgeboard/ftp_share/code/405/detection.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(300, 200));
  if (!wri.isOpened()) {
      std::cerr << "Error: VideoWriter not opened!" << std::endl;
      return -1;
  }else{
      cerr<<"video ready"<<endl;
  }

  signal(SIGTSTP, callback);
  int ret = 0;
  int image_sem = GetSem(IMAGE_ID);
  int result_sem = GetSem(RESULT_ID);
  int result_shm = GetShm(RESULT_SHM, 4096);
  int image_shm = GetShm(IMAGE_SHM, 44*4096);
  usleep(1000);
  int* addr = (int*)shmat(result_shm, NULL, 0);
  void* image_addr = shmat(image_shm, NULL, 0);

  Mat frame;

  int bomb_count, bridge_count, right_garage_count, left_garage_count;
  bomb_count, bridge_count, right_garage_count, left_garage_count = 0;
  int cat_x, cat_y;
  bool found;
  unsigned int count = 0, count_last = 0;

  cerr << "AI Begin!" << endl;
  int state = 0;
  int cat_id = 0;
  int cat_max_y = 0, num_max_y = 0; 
  chrono::high_resolution_clock::time_point t1, t2;

  int dur;
  while (!stop) {
    semP(result_sem);
    state = addr[1];
    count = addr[2];
    semV(result_sem);

    if (state == 0 || count == count_last) {

      usleep(20 * 1000);
      cout<< "sleeping"<<endl;
      continue;
    }
  
    count_last = count;
    cout << "--------------------------------" << endl;
    cout << "Predict frame " << count << "..." << endl;
    semP(image_sem);
    matRead(image_addr, frame);
    semV(image_sem);
    usleep(1000);
    if (frame.empty()){
      usleep(20 * 1000);
      continue;
    }

    t1 = chrono::high_resolution_clock::now();
    detection->inference(frame);
    t2 = chrono::high_resolution_clock::now();
    dur = chrono::duration_cast<chrono::microseconds>(t2-t1).count();
    cout << "FPS: " << 1000000.0 / dur << endl;

    if (!detection->predict_results.empty()){
      cout << "Found " << detection->predict_results.size() << "results:" << endl;
      for (auto& pr : detection->predict_results){
        pr.x = round(pr.x * (160.0/300));
        pr.width = round(pr.width * (160.0/300));
        pr.y = round(pr.y * (120.0/200));
        pr.height = round(pr.x * (120.0/200));
        cerr << pr.type << ",(" << pr.x << "," << pr.y << ")," << pr.width << "," << pr.height << endl;
      }
      drawResults(frame, detection->predict_results);
      cout << endl;
      found = true;
      cat_id = -1;
      cat_max_y = 5;
      for (int i = 0; i < detection->predict_results.size(); i++){
        if (detection->predict_results[i].y > cat_max_y) {
          cat_id = i;
          cat_max_y = detection->predict_results[i].y;
        }
      }
      if (cat_id != -1){
        if (detection->predict_results[cat_id].type == 0){ //work
            if (detection->predict_results[cat_id].y + detection->predict_results[cat_id].width / 2 > 40) {
                bomb_count++;
                cout << "Bomb found!" << endl;
            }
        }
        else if (detection->predict_results[cat_id].type == 1){ //fork
            if (detection->predict_results[cat_id].width>20){
                bridge_count++;
                cout << "Bridge found!" << endl;
            }
        }
        else if (detection->predict_results[cat_id].type == 2 || detection->predict_results[cat_id].type == 4){ //station
            right_garage_count++;
            cout << "Right garage found!" << endl;
        }
        else if (detection->predict_results[cat_id].type == 3 || detection->predict_results[cat_id].type == 5){ //hill
            left_garage_count++;
            cout << "Left garage found!" << endl;
        }
      }
    }
    else found = false;
    wri << frame;

    semP(result_sem);
    if (!found){
      bridge_count = bomb_count = right_garage_count = left_garage_count = 0;
      addr[0] = 'N';
    }
    else {
      if (bomb_count >= THRESH){
        bomb_count = 0;
        cout<<"sent bomb_count ai signal"<<endl;
        addr[0] = 'O';
      }
      else if (bridge_count >= THRESH){
        bridge_count = 0;
        cout<<"sent bridge_count ai signal"<<endl;
        addr[0] = 'B';
      }
      else if (right_garage_count >= THRESH){
        right_garage_count = 0;
        cout<<"sent right_garage_count ai signal"<<endl;
        addr[0] = 'R';
      }
      else if (left_garage_count >= THRESH){
        left_garage_count = 0;
        cout<<"sent left_garage_count ai signal"<<endl;
        addr[0] = 'L';
      } 
    }
    semV(result_sem);
    //cerr<<"done"<<endl;
    //[01] 视频源读取
    // if (motion.params.debug) // 综合显示调试UI窗口
    //   preTime = chrono::duration_cast<chrono::milliseconds>(
    //                 chrono::system_clock::now().time_since_epoch())
    //                 .count();
    // if (!capture.read(img))
    //   continue;
    // if (motion.params.saveImg && !motion.params.debug) // 存储原始图像
    //   savePicture(img);

    //[02] 图像预处理
    // Mat imgCorrect = preprocess.correction(img);         // 图像矫正
    // Mat imgBinary = preprocess.binaryzation(imgCorrect); // 图像二值化

    //[03] 启动AI推理
    //[04] 赛道识别
    // tracking.rowCutUp = motion.params.rowCutUp; // 图像顶部切行（前瞻距离）
    // tracking.rowCutBottom =
    //     motion.params.rowCutBottom; // 图像底部切行（盲区距离）
    // tracking.trackRecognition(imgBinary);
    // if (motion.params.debug) // 综合显示调试UI窗口
    // {
    //   Mat imgTrack = imgCorrect.clone();
    //   tracking.drawImage(imgTrack); // 图像绘制赛道识别结果
    //   display.setNewWindow(2, "Track", imgTrack);
    // }

    // //[05] 停车区检测
    // if (motion.params.parking) {
    //   if (parking.process(detection->results)) {
    //     scene = Scene::ParkingScene;
    //     if (parking.countExit > 20) {
    //       uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
    //       sleep(1);
    //       printf("-----> System Exit!!! <-----\n");
    //       exit(0); // 程序退出
    //     }
    //   }
    // }

  //   //[06] 救援区检测
  //   if ((scene == Scene::NormalScene || scene == Scene::RescueScene) &&
  //       motion.params.rescue) {
  //     if (rescue.process(tracking, detection->results))
  //       scene = Scene::RescueScene;
  //     else
  //       scene = Scene::NormalScene;
  //   }

  //   //[07] 追逐区检测
  //   if ((scene == Scene::NormalScene || scene == Scene::RacingScene) &&
  //       motion.params.racing) {
  //     if (racing.process(tracking, detection->results))
  //       scene = Scene::RacingScene;
  //     else
  //       scene = Scene::NormalScene;
  //   }

  //   //[08] 坡道区检测
  //   if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
  //       motion.params.bridge) {
  //     if (bridge.process(tracking, detection->results))
  //       scene = Scene::BridgeScene;
  //     else
  //       scene = Scene::NormalScene;
  //   }

  //   // [09] 危险区检测
  //   if ((scene == Scene::NormalScene || scene == Scene::DangerScene) &&
  //       motion.params.danger) {
  //     if (danger.process(tracking, detection->results)) {
  //       uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
  //       scene = Scene::DangerScene;
  //     } else
  //       scene = Scene::NormalScene;
  //   }

  //   //[10] 十字道路识别与路径规划
  //   if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
  //       motion.params.cross) {
  //     if (crossroad.crossRecognition(tracking))
  //       scene = Scene::CrossScene;
  //     else
  //       scene = Scene::NormalScene;
  //   }

  //   //[11] 环岛识别与路径规划
  //   if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
  //       motion.params.ring) {
  //     if (ring.process(tracking, imgBinary))
  //       scene = Scene::RingScene;
  //     else
  //       scene = Scene::NormalScene;
  //   }

  //   //[12] 车辆控制中心拟合
  //   ctrlCenter.fitting(tracking);
  //   if (scene != Scene::RescueScene) {
  //     if (ctrlCenter.derailmentCheck(tracking)) // 车辆冲出赛道检测（保护车辆）
  //     {
  //       uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
  //       sleep(1);
  //       printf("-----> System Exit!!! <-----\n");
  //       exit(0); // 程序退出
  //     }
  //   }

  //   //[13] 车辆运动控制(速度+方向)
  //   if (!motion.params.debug) // 非调试模式下
  //   {
  //     if ((scene == Scene::RescueScene && rescue.carStoping) || parking.park ||
  //         racing.carStoping) // 特殊区域停车
  //       motion.speed = 0;
  //     else if (scene == Scene::RescueScene && rescue.carExitting) // 倒车出库
  //       motion.speed = -motion.params.speedDown;
  //     else if (scene == Scene::RescueScene) // 减速
  //       motion.speedCtrl(true, true, ctrlCenter);
  //     else if (scene == Scene::BridgeScene) // 坡道速度
  //       motion.speed = motion.params.speedBridge;
  //     else
  //       motion.speedCtrl(true, false, ctrlCenter); // 车速控制

  //     motion.poseCtrl(ctrlCenter.controlCenter); // 姿态控制（舵机）
  //     uart->carControl(motion.speed, motion.servoPwm); // 串口通信控制车辆
  //   }

  //   //[14] 综合显示调试UI窗口
  //   if (motion.params.debug) {
  //     // 帧率计算
  //     auto startTime = chrono::duration_cast<chrono::milliseconds>(
  //                          chrono::system_clock::now().time_since_epoch())
  //                          .count();
  //     printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime,
  //            1000.0 / (startTime - preTime));

  //     display.setNewWindow(1, "Binary", imgBinary);
  //     Mat imgRes =
  //         Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像

  //     switch (scene) {
  //     case Scene::NormalScene:
  //       break;
  //     case Scene::CrossScene:                  //[ 十字区 ]
  //       crossroad.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
  //       break;
  //     case Scene::RingScene:              //[ 环岛 ]
  //       ring.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
  //       break;
  //     case Scene::BridgeScene:              //[ 坡道区 ]
  //       bridge.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
  //       circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
  //              Scalar(40, 120, 250), -1);
  //       putText(imgCorrect, "S", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
  //               FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
  //       break;
  //     case Scene::DangerScene:    //[ 危险区 ]
  //       danger.drawImage(imgRes); // 图像绘制特殊赛道识别结果
  //       circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
  //              Scalar(40, 120, 250), -1);
  //       putText(imgCorrect, "X", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
  //               FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
  //       break;
  //     case Scene::RescueScene:              //[ 救援区 ]
  //       rescue.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
  //       circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
  //              Scalar(40, 120, 250), -1);
  //       putText(imgCorrect, "O", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
  //               FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
  //       break;
  //     case Scene::RacingScene:    //[ 追逐区 ]
  //       racing.drawImage(imgRes); // 图像绘制特殊赛道识别结果
  //       circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
  //              Scalar(40, 120, 250), -1);
  //       putText(imgCorrect, "R", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
  //               FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
  //       break;
  //     case Scene::ParkingScene:    //[ 停车区 ]
  //       parking.drawImage(imgRes); // 图像绘制特殊赛道识别结果
  //       circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
  //              Scalar(40, 120, 250), -1);
  //       putText(imgCorrect, "P", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
  //               FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
  //       break;
  //     default: // 常规道路场景：无特殊路径规划
  //       break;
  //     }

  //     display.setNewWindow(3, getScene(scene),
  //                          imgRes);   // 图像绘制特殊场景识别结果
  //     detection->drawBox(imgCorrect); // 图像绘制AI结果
  //     ctrlCenter.drawImage(tracking,
  //                          imgCorrect); // 图像绘制路径计算结果（控制中心）
  //     display.setNewWindow(4, "Ctrl", imgCorrect);
  //     display.show(); // 显示综合绘图
  //     waitKey(10);    // 等待显示
  //   }

  //   //[15] 状态复位
  //   if (sceneLast != scene) {
  //     if (scene == Scene::NormalScene)
  //       uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
  //     else
  //       uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
  //   }
  //   sceneLast = scene; // 记录当前状态
  //   if (scene == Scene::DangerScene)
  //     scene = Scene::NormalScene;
  //   else if (scene == Scene::CrossScene)
  //     scene = Scene::NormalScene;

  //   //[16] 按键退出程序
  //   if (uart->keypress) {
  //     uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
  //     sleep(1);
  //     printf("-----> System Exit!!! <-----\n");
  //     exit(0); // 程序退出
  //   }
  //cerr<<"one turn"<<endl;
  }
  wri.release();
  cout << "AI quitting..." << endl;
  usleep(300 * 1000);
  shmdt(addr);
  shmdt(image_addr);
  DestroyShm(result_shm);
  DestroyShm(image_shm);
  DestroySem(image_sem);
  DestroySem(result_sem);
  cerr << "finished!" << endl;
  return 0;

  // uart->close(); // 串口通信关闭
  // capture.release();
  // return 0;
}


        
