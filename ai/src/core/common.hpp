#pragma once
#include "../utils/json.hpp"
#include <chrono> // NOLINT
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using json = nlohmann::json;

struct FrameWrapper {
  cv::Mat frame;
  int channel;
};

struct ModelConfig {
  std::string model_parent_dir;

  std::string network_type;

  std::string model_file;
  std::string model_params_dir;
  std::string params_file;

  std::string format;
  uint16_t input_width;
  uint16_t input_height;

  float means[3];
  float scales[3];
  float threshold;

  bool is_yolo;
  bool is_combined_model;

  std::vector<string> labels;

  void assert_check_file_exist(std::string fileName, std::string modelPath) {
    std::ifstream infile(modelPath + fileName);
    if (!infile.good()) {
      printf("Error!!!! ModelConfig: File %s not exit, Please Check your model "
             "path:%s .\n",
             fileName.c_str(), modelPath.c_str());
      exit(-1);
    }
  }
  ModelConfig(std::string model_path) : model_parent_dir(model_path + "/") {

    std::string json_config_path = model_parent_dir + "config.json";
    std::ifstream is(json_config_path);
    if (!is.good()) {
      std::cout << "Error:ModelConfig file path:[" << json_config_path
                << "] not find ." << std::endl;
      exit(-1);
    }

    json value;
    is >> value;
    std::cout << "Config:" << value << std::endl;

    input_width = value["input_width"];
    input_height = value["input_height"];
    format = value["format"];
    std::transform(format.begin(), format.end(), format.begin(), ::toupper);

    std::vector<float> mean = value["mean"];
    for (int i = 0; i < mean.size(); ++i) {
      means[i] = mean[i];
    }

    std::vector<float> scale = value["scale"];
    for (int i = 0; i < scale.size(); ++i) {
      scales[i] = scale[i];
    }

    if (value["threshold"] != nullptr) {
      threshold = value["threshold"];
    } else {
      threshold = 0.5;
      std::cout << "Warnning !!!!,json key: threshold not found, default :"
                << threshold << "\n";
    }

    is_yolo = false;
    if (value["network_type"] != nullptr) {
      network_type = value["network_type"];
      if (network_type == "YOLOV3") {
        is_yolo = true;
      }
    }

    if ((value["model_file_name"] != nullptr) &&
        (value["params_file_name"] != nullptr) &&
        (value["model_dir"] == nullptr)) {
      is_combined_model = true;
      params_file =
          model_parent_dir + value["params_file_name"].get<std::string>();
      model_file =
          model_parent_dir + value["model_file_name"].get<std::string>();
      model_params_dir = "";
    } else if ((value["model_file_name"] == nullptr) &&
               (value["params_file_name"] == nullptr) &&
               (value["model_dir"] != nullptr)) {
      is_combined_model = false;
      model_params_dir =
          model_parent_dir + value["model_dir"].get<std::string>();
      params_file = "";
      model_file = "";
    } else {
      std::cout
          << "json config Error !!!! \n combined_model: need params_file_name "
             "model_file_name, separate_model: need model_dir only.\n";
      exit(-1);
    }

    if (value["labels_file_name"] != nullptr) {
      std::string labels_file_name = value["labels_file_name"];
      std::string label_path = model_parent_dir + labels_file_name;
      std::ifstream file(label_path);
      if (file.is_open()) {
        std::string line;
        while (getline(file, line)) {
          labels.push_back(line);
        }
        file.close();
      } else {
        std::cout << "Open Lable File failed, file path: " << label_path
                  << std::endl;
      }
    }

    if (is_combined_model) {
      assert_check_file_exist(value["model_file_name"], model_parent_dir);
      assert_check_file_exist(value["params_file_name"], model_parent_dir);
    } else {
      assert_check_file_exist(value["model_dir"], model_parent_dir);
    }

    std::cout << "Model Config Init Success !!!" << std::endl;
  }
  ~ModelConfig() {}
};

inline std::string get_file_path(const std::string &fname) {
  size_t pos = fname.find_last_of("/");
  return (std::string::npos == pos) ? "" : fname.substr(0, pos);
}

struct SystemConfig {
  std::string config_dir;
  std::string model_config_path;

  std::string input_type;
  std::string input_path;

  // std::string input_format;
  // uint16_t input_width;
  // uint16_t input_height;

  bool use_fpga_preprocess;

  // for debug
  bool predict_time_log_enable;
  bool predict_log_enable;
  bool display_enable;

  SystemConfig(std::string dir) : config_dir(dir) {
    json value;
    std::ifstream is(config_dir);
    if (!is.good()) {
      std::cout << "Error:SystemConfig file path:[" << config_dir
                << "] not find ." << std::endl;
      exit(-1);
    }
    is >> value;
    std::cout << "SystemConfig:" << value << std::endl;

    std::string confiig_path = get_file_path(dir) + "/";
    model_config_path = value["model_config"];
    model_config_path = confiig_path + model_config_path;

    json input_config = value["input"];
    input_type = input_config["type"];

    if (input_config["path"] != nullptr) {
      input_path = input_config["path"];
      if (input_type == "image") {
        input_path = confiig_path + input_path;
      }
    }

    if (value["fpga_preprocess"] == nullptr) {
      use_fpga_preprocess = true;
    } else {
      use_fpga_preprocess = value["fpga_preprocess"];
    }

    if (value["debug"] != nullptr) {
      json debug_config = value["debug"];
      if (debug_config["predict_time_log_enable"] == nullptr) {
        predict_time_log_enable = true;
      } else {
        predict_time_log_enable = debug_config["predict_time_log_enable"];
      }

      if (debug_config["predict_log_enable"] == nullptr) {
        predict_log_enable = true;
      } else {
        predict_log_enable = debug_config["predict_log_enable"];
      }

      if (debug_config["display_enable"] == nullptr) {
        display_enable = true;
      } else {
        display_enable = debug_config["display_enable"];
      }
    } else {
      display_enable = true;
      predict_log_enable = true;
      predict_time_log_enable = true;
    }

    std::cout << "SystemConfig Init Success !!!" << std::endl;
  }
  ~SystemConfig() {}
};

using Time = decltype(std::chrono::high_resolution_clock::now());
inline Time time() { return std::chrono::high_resolution_clock::now(); }
inline double time_diff(Time t1, Time t2) {
  typedef std::chrono::microseconds ms;
  auto diff = t2 - t1;
  ms counter = std::chrono::duration_cast<ms>(diff);
  return counter.count() / 1000.0;
}

class Timer {
private:
  std::string _name;
  double sum;
  int cur_counts;
  int max_record_counts;
  Time start_time;

public:
  void Continue() {
    if (cur_counts >= max_record_counts) {
      return;
    }
    start_time = time();
  };
  void Pause() {
    if (cur_counts >= max_record_counts) {
      return;
    }
    Time stop_time = time();
    double diff_time = time_diff(start_time, stop_time);
    sum += diff_time;
    cur_counts++;
  };

  void printAverageRunTime() {
    if (cur_counts < 1) {
      return;
    }
    printf("[%s] Total Record %d times ,Cur %d times ,Use [%lfms], Average "
           "[%lfms] \n",
           _name.c_str(), max_record_counts, cur_counts, sum, sum / cur_counts);
  };

  Timer(std::string name, int maxRecordCounts = 30)
      : _name(name), max_record_counts(maxRecordCounts) {
    cur_counts = 0;
    sum = 0;
  };
  ~Timer(){};
};
