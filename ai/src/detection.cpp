#include "core/capture.hpp"
#include "core/common.hpp"
#include "core/display.hpp"
#include "core/preprocess.hpp"
#include "utils/shared_mem.h"
#include <unistd.h>
#include <chrono>
#include <csignal>

#define THRESH 2

bool stop = false;
void callback(int signum){
    cerr << "AI received signal, quit!" << endl;
    stop = true;
}

struct PredictResult {
  int type;
  float score;
  int x;
  int y;
  int width;
  int height;
};

static std::shared_ptr<SystemConfig> g_system_config = nullptr;
static std::shared_ptr<ModelConfig> g_model_config = nullptr;
static std::shared_ptr<PaddlePredictor> g_predictor = nullptr;

static int predictorInit() {

  std::vector<Place> valid_places({
      Place{TARGET(kFPGA), PRECISION(kFP16), DATALAYOUT(kNHWC)},
      Place{TARGET(kHost), PRECISION(kFloat)},
      Place{TARGET(kARM), PRECISION(kFloat)},
  });

  paddle::lite_api::CxxConfig config;

  if (g_model_config->is_combined_model) {
    config.set_model_file(g_model_config->model_file);
    config.set_param_file(g_model_config->params_file);
  } else {
    config.set_model_dir(g_model_config->model_params_dir);
  }

  config.set_valid_places(valid_places);

  g_predictor = paddle::lite_api::CreatePaddlePredictor(config);
  if (!g_predictor) {
    std::cout << "Error: CreatePaddlePredictor Failed." << std::endl;
    return -1;
  }
  std::cout << "Predictor Init Success !!!" << std::endl;
  return 0;
}

static std::vector<PredictResult> predict(cv::Mat inputFrame,
                                          std::shared_ptr<Timer> timer) {

  std::vector<PredictResult> predict_ret;

  auto input = g_predictor->GetInput(0);
  input->Resize(
      {1, 3, g_model_config->input_height, g_model_config->input_width});

  if (g_system_config->use_fpga_preprocess) {
    auto *in_data = input->mutable_data<uint16_t>();
    fpga_preprocess(inputFrame, *g_model_config, input);
  } else {
    auto *in_data = input->mutable_data<float>();
    cpu_preprocess(inputFrame, *g_model_config, in_data);
  }

  if (g_model_config->is_yolo) {
    auto img_shape = g_predictor->GetInput(1);
    img_shape->Resize({1, 2});
    auto *img_shape_data = img_shape->mutable_data<int32_t>();
    img_shape_data[0] = inputFrame.rows;
    img_shape_data[1] = inputFrame.cols;
  }

  static bool is_first_run = true;
  if (is_first_run) {
    g_predictor->Run();
    is_first_run = false;
  } else {
    timer->Continue();
    g_predictor->Run();
    timer->Pause();
  }

  auto output = g_predictor->GetOutput(0);
  float *result_data = output->mutable_data<float>();
  int size = output->shape()[0];

  for (int i = 0; i < size; i++) {
    float *data = result_data + i * 6;
    float score = data[1];
    if (score < g_model_config->threshold) {
      continue;
    }
    PredictResult r;
    r.type = (int)data[0];
    r.score = score;
    if (g_model_config->is_yolo) {
      r.x = data[2];
      r.y = data[3];
      r.width = data[4] - r.x;
      r.height = data[5] - r.y;
    } else {
      r.x = data[2] * inputFrame.cols;
      r.y = data[3] * inputFrame.rows;
      r.width = data[4] * inputFrame.cols - r.x;
      r.height = data[5] * inputFrame.rows - r.y;
    }
    predict_ret.push_back(r);
  }
  return predict_ret;
}

void printResults(FrameWrapper &frame_wraper,
                  std::vector<PredictResult> results) {

  for (int i = 0; i < results.size(); ++i) {
    PredictResult r = results[i];
    printf("[PredictorLog]-Ch[%d] ", frame_wraper.channel);

    if (g_model_config->labels.size() > 0) {
      std::cout << "label:" << g_model_config->labels[r.type] << ",";
    }
    std::cout << "index:" << r.type << ",score:" << r.score << " loc:";
    std::cout << r.x << "," << r.y << "," << r.width << "," << r.height
              << std::endl;
  }
}
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
    boundaryCorrection(r, inputFrame.cols, inputFrame.rows);
    if (r.type >= 0 && r.type < g_model_config->labels.size()) {
      cv::Point origin(r.x, r.y);
      std::string label_name = g_model_config->labels[r.type];
      cv::putText(inputFrame, label_name, origin, cv::FONT_HERSHEY_PLAIN, 2,
                  cv::Scalar(255, 0, 0, 255), 2);
    }
    cv::Rect rect(r.x, r.y, r.width, r.height);
    cv::rectangle(inputFrame, rect, Scalar(0, 0, 224), 2);
  }
}

int main() {
  signal(SIGTSTP, callback);
  int ret = 0;
  std::string system_config_path;
  std::string model_config_path;

  system_config_path = "./ai/configs/config.json";

  std::cout << "SystemConfig Path:" << system_config_path << std::endl;
  g_system_config = make_shared<SystemConfig>(system_config_path);

  model_config_path = g_system_config->model_config_path;
  std::cout << "ModelConfig Path:" << model_config_path << std::endl;;
  g_model_config = make_shared<ModelConfig>(g_system_config->model_config_path);

  ret = predictorInit();
  if (ret != 0) {
    std::cout << "Error!!! predictor init failed .\n";
    exit(-1);
  }

  std::vector<PredictResult> predict_results;
  std::shared_ptr<Timer> timer = make_shared<Timer>("Predict", 100);

  int image_sem = GetSem(IMAGE_ID);
  int result_sem = GetSem(RESULT_ID);
  int result_shm = GetShm(RESULT_SHM, 4096);
  int image_shm = GetShm(IMAGE_SHM, 44*4096);
  usleep(1000);
  int* addr = (int*)shmat(result_shm, NULL, 0);
  void* image_addr = shmat(image_shm, NULL, 0);

  Mat frame;
  int bridge_count, tractor_count, corn_count, pig_count;
  bridge_count = tractor_count = corn_count = pig_count = 0;
  int cat_x, cat_y;
  bool found;
  unsigned int count = 0, count_last = 0;

  // chrono::high_resolution_clock::time_point t1, t2;
  // Mat test = Mat::zeros(320, 240, CV_8UC3);
  // t1 = chrono::high_resolution_clock::now();
  // matWrite("raw.bin", test);
  // t2 = chrono::high_resolution_clock::now();
  // cerr << chrono::duration_cast<chrono::microseconds>(t2-t1).count() << endl;
  // InitSem(image_sem);
  // InitSem(result_sem);

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
    // imshow("frame", frame);
    // waitKey(1);
    t1 = chrono::high_resolution_clock::now();
    predict_results = predict(frame, timer);
    t2 = chrono::high_resolution_clock::now();
    dur = chrono::duration_cast<chrono::microseconds>(t2-t1).count();
    cout << "FPS: " << 1000000.0 / dur << endl;

    if (!predict_results.empty()){
      cout << "Found " << predict_results.size() << "results:" << endl;
      for (auto& pr : predict_results){
        pr.x = round(pr.x * (160.0/300));
        pr.width = round(pr.width * (160.0/300));
        pr.y = round(pr.y * (120.0/200));
        pr.height = round(pr.x * (120.0/200));
        cout << pr.type << ",(" << pr.x << "," << pr.y << ")," << pr.width << "," << pr.height << endl;
      }
      cout << endl;
      found = true;
      cat_id = -1;
      cat_max_y = 30;
      for (int i = 0; i < predict_results.size(); i++){
        if (predict_results[i].y > cat_max_y) {
          cat_id = i;
          cat_max_y = predict_results[i].y;
        }
      }
      if (cat_id != -1){
        if (predict_results[cat_id].type == 1){ //work
            if (predict_results[cat_id].y + predict_results[cat_id].width / 2 > 40) {
                bridge_count++;
                cout << "Bridge found!" << endl;
            }
        }
        else if (predict_results[cat_id].type == 2){ //fork
            if (predict_results[cat_id].width>20){
                tractor_count++;
                cout << "Tractor found!" << endl;
            }
        }
        else if (predict_results[cat_id].type == 3){ //station
            corn_count++;
            cout << "Corn found!" << endl;
        }
        else if (predict_results[cat_id].type == 4){ //hill
            pig_count++;
            cout << "Pig found!" << endl;
        }
      }
    }
    else found = false;

    semP(result_sem);
    if (!found){
      bridge_count = tractor_count = corn_count = pig_count = 0;
      addr[0] = 'N';
    }
    else {
      if (bridge_count >= THRESH){
        bridge_count = 0;
        addr[0] = 'B';
      }
      else if (tractor_count >= THRESH){
        tractor_count = 0;
        addr[0] = 'T';
      }
      else if (corn_count >= THRESH){
        corn_count = 0;
        addr[0] = 'C';
      }
      else if (pig_count >= THRESH){
        pig_count = 0;
        addr[0] = 'P';
      } 
    }
    semV(result_sem);

  }
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
}