#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>


const int kGpuId = 0;
const int kNumClass = 1;
const int kInputH = 640;
const int kInputW = 640;
const float kNmsThresh = 0.45f;
const float kConfThresh = 0.25f;
const int kMaxNumOutputBbox = 1000;  // assume the box outputs no more than kMaxNumOutputBbox boxes that conf >= kNmsThresh;
const int kNumBoxElement = 7;  // left, top, right, bottom, confidence, class, keepflag(whether drop when NMS)

const std::string onnxFile = "../onnx_model/nice.onnx";
// const std::string trtFile = "./yolo11s.plan";
// const std::string testDataDir = "../images";  // 用于推理

// for FP16 mode
const bool bFP16Mode = false;
// for INT8 mode
const bool bINT8Mode = false;
const std::string cacheFile = "./int8.cache";
const std::string calibrationDataPath = "../calibrator";  // 存放用于 int8 量化校准的图像

const std::vector<std::string> vClassNames {
    "White"
};

#endif  // CONFIG_H
