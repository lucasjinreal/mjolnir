
#pragma once

#include <sys/stat.h>

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "cmath"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "type.h"
#include "io.h"

using cv::Mat;
using cv::Point2f;
using cv::Scalar;

namespace mjolnir {

namespace vis {

enum IterMode {
  IMAGE = 0,
  DIR = 1,
  VIDEO = 2,
};

template <class Item> class SourceIterator {
public:
  virtual Item next() = 0;

public:
  // members
  std::vector<Item> item_pool;
  std::vector<std::string> item_str_pool;
  bool is_video_mode = false;
  bool ok = false;
  int crt = 0;
  cv::VideoCapture cap;
  IterMode mode;
};

template <class Item> class ImageSourceIter : public SourceIterator<Item> {
public:
  explicit ImageSourceIter(std::string source);
  ~ImageSourceIter();
  Item next();
  int waitKey();
};

template <class Item> ImageSourceIter<Item>::ImageSourceIter(string source) {
  this->crt = 0;
  this->item_pool.clear();
  this->item_str_pool.clear();

  // judge if this is directory, image file, or video file
  if (mjolnir::os::isfile(source)) {
    // judge if it's video or image file
    if (mjolnir::os::suffix(source) == "mp4" ||
        mjolnir::os::suffix(source) == "avi" ||
        mjolnir::os::suffix(source) == "flv") {
      this->is_video_mode = true;
      this->mode = IterMode::VIDEO;
      auto res = this->cap.open(source);
      if (!res) {
        std::cerr << "open video error! " << source << std::endl;
      }
    } else {
      cv::Mat itm = cv::imread(source);
      this->mode = IterMode::IMAGE;
      this->item_pool.emplace_back(itm);
      this->item_str_pool.push_back("1");
    }
  } else if (mjolnir::os::isdir(source)) {
    this->mode = IterMode::DIR;
    this->item_str_pool = mjolnir::os::list_files(source, true);
  }
  this->ok = true;
}

template <class Item> ImageSourceIter<Item>::~ImageSourceIter() {
  if (this->is_video_mode) {
    this->cap.release();
  }
}

template <class Item> Item ImageSourceIter<Item>::next() {
  if (this->mode == IterMode::VIDEO) {
    cv::Mat a;
    this->cap >> a;
    if (a.empty()) {
      this->ok = false;
    }
    return a;
  } else if (this->mode == IterMode::IMAGE) {
    // we have only 1 item
    this->ok = false;
    cv::Mat a = this->item_pool[this->crt];
    return a;
  } else if (this->mode == IterMode::DIR) {
    cv::Mat a = cv::imread(this->item_str_pool[this->crt]);
    this->crt += 1;
    if (this->crt >= this->item_str_pool.size()) {
      this->ok = false;
      return cv::Mat::zeros(4, 4, CV_32F);
    }
    return a;
  } else {
    return cv::Mat::zeros(4, 4, CV_32F);
  }
}

template <class Item> int ImageSourceIter<Item>::waitKey() {
  if (this->is_video_mode) {
    int k = cv::waitKey(1);
    if (k == 27 || k == 113) {
      exit(0);
    }
  } else {
    cv::waitKey(0);
  }
  return 0;
}

struct RGB {
  float r, g, b;
};

struct RGBA {
  float r, g, b, a;
  inline void print() {
    std::cout << "r:" << r << " g:" << g << " b:" << b << " a:" << a << endl;
  }
};

// conversions
inline cv::Scalar toCvColor(const unsigned char *c, bool swap = false) {
  if (false) {
    return cv::Scalar(c[2], c[1], c[0]);
  } else {
    return cv::Scalar(c[0], c[1], c[2]);
  }
}

// adding default values for better calling
mjolnir::vis::RGBA gen_unique_color(int idx, bool is_track = false,
                                    double hue_step = 0.41, float alpha = 0.7);
void hsv2rgb(float &r, float &g, float &b, float h, float s, float v);
void hsv2rgb(mjolnir::vis::RGBA &rgba, float h, float s, float v);
cv::Scalar gen_unique_color_cv(int idx, bool is_track = false,
                               double hue_step = 0.41, float alpha = 0.7);

cv::Mat createAlpha(cv::Mat &src);

int addAlpha(cv::Mat &src, cv::Mat &dst, cv::Mat &alpha);

// draw detections
cv::Mat VisualizeDet(cv::Mat &img, vector<vector<float>> detections,
                     vector<string> classes_names, bool enable_mask = true,
                     float confidence_threshold = 0.02,
                     bool normalized = false);

cv::Mat VisualizeBox(cv::Mat &img, vector<mjolnir::Box> detections,
                     vector<string> classes_names, bool enable_mask = false,
                     const vector<cv::Scalar> *colors = nullptr,
                     const float line_thickness = 1,
                     const float font_scale = 0.45,
                     float confidence_threshold = 0.02,
                     bool normalized = false);

cv::Mat VisualizeDetectionStyleDetectron2(cv::Mat &img,
                                          vector<mjolnir::Box> detections,
                                          vector<string> classes_names,
                                          bool enable_mask = true,
                                          float confidence_threshold = 0.02,
                                          bool normalized = false);

cv::Mat VisualizeDetectionStyleDetectron2(cv::Mat &img,
                                          vector<mjolnir::Bbox> detections,
                                          vector<string> classes_names,
                                          bool enable_mask = true,
                                          float confidence_threshold = 0.02,
                                          bool normalized = false);

// More modern API which same as alfred
cv::Mat VisualizeDetections(
    cv::Mat &img, vector<mjolnir::Detection> detections,
    const vector<string> classes_names, const vector<cv::Scalar> *colors = NULL,
    const float line_thickness = 1, const float font_scale = 0.38,
    const bool fancy = false, const float confidence_threshold = 0.02,
    const bool enable_mask = false, const bool normalized = false);

cv::Mat VisualizeDetectionsWithLandmark(
    cv::Mat &img, vector<mjolnir::Detection> detections,
    const vector<string> classes_names, const bool enable_mask = false,
    const bool landmark_on = true, const vector<cv::Scalar> *colors = NULL,
    const float line_thickness = 1, const float font_scale = 0.38,
    const bool fancy = false, const float confidence_threshold = 0.02,
    const bool normalized = false);

cv::Mat VisualizeDetectionsWithOverrideColors(
    cv::Mat &img, vector<mjolnir::Detection> detections,
    const vector<string> classes_names,
    const std::map<int, cv::Scalar> *override_colors = NULL,
    const float line_thickness = 1, const bool with_text = true,
    const float font_scale = 0.38, const bool fancy = false,
    const float confidence_threshold = 0.02, const bool enable_mask = false,
    const bool normalized = false);

void VisTextInfos(cv::Mat &img, const vector<std::string> txts,
                  const cv::Point start_pt = cv::Point(10, 20));

/////////////////// Visualize Lane ///////////////////////
cv::Mat VisualizeLanes(cv::Mat &img, const vector<vector<cv::Point>> &lanes,
                       const vector<cv::Scalar> *colors = NULL,
                       const float line_thickness = 12, const float alpha = 1.0,
                       const bool guide_line = true);
void renderHumanPose(std::vector<HumanPose> &poses, cv::Mat &image);
void renderHumanPoseSimple(std::vector<HumanPose> &poses, cv::Mat &image);
void renderPoseCoco17(std::vector<HumanPose> &poses, cv::Mat &image);

} // namespace vis

namespace image {

typedef struct {
  int w;
  int h;
  int c;
  float *data;
} Image;

float *Normalize(cv::Mat img);

float *HWC2CHW(cv::Mat img, const float kMeans[3], const float kStds[3]);
float *HWC2CHW_fast(cv::Mat img, const float kMeans[3], const float kStds[3]);

cv::Mat read2mat(float *prob, cv::Mat out);
cv::Mat map2threeunchar(cv::Mat real_out, cv::Mat real_out_);
cv::Mat resizeAlongShortest(cv::Mat img, int target_w, int target_h);

} // namespace image

namespace dl {

static std::vector<std::string> VOC_CLASSES = {"__background__",
                                               "aeroplane",
                                               "bicycle",
                                               "bird",
                                               "boat",
                                               "bottle",
                                               "bus",
                                               "car",
                                               "cat",
                                               "chair",
                                               "cow",
                                               "diningtable",
                                               "dog",
                                               "horse",
                                               "motorbike",
                                               "person",
                                               "pottedplant",
                                               "sheep",
                                               "sofa",
                                               "train",
                                               "tvmonitor"};

static std::vector<std::string> COCO_CLASSES_NO_BK = {
    "person",        "bicycle",      "car",
    "motorcycle",    "airplane",     "bus",
    "train",         "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench",        "bird",
    "cat",           "dog",          "horse",
    "sheep",         "cow",          "elephant",
    "bear",          "zebra",        "giraffe",
    "backpack",      "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",
    "skis",          "snowboard",    "sports ball",
    "kite",          "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket",
    "bottle",        "wine glass",   "cup",
    "fork",          "knife",        "spoon",
    "bowl",          "banana",       "apple",
    "sandwich",      "orange",       "broccoli",
    "carrot",        "hot dog",      "pizza",
    "donut",         "cake",         "chair",
    "couch",         "potted plant", "bed",
    "dining table",  "toilet",       "tv",
    "laptop",        "mouse",        "remote",
    "keyboard",      "cell phone",   "microwave",
    "oven",          "toaster",      "sink",
    "refrigerator",  "book",         "clock",
    "vase",          "scissors",     "teddy bear",
    "hair drier",    "toothbrush"};
} // namespace dl

namespace color {
// Note: color value is in BGR order.
// Color reference: https://en.wikipedia.org/wiki/Web_colors#X11_color_names

constexpr unsigned char kAliceBlue[3] = {255, 248, 240};
constexpr unsigned char kAntiqueWhite[3] = {215, 235, 250};
constexpr unsigned char kAqua[3] = {255, 255, 0};
constexpr unsigned char kAquamarine[3] = {212, 255, 127};
constexpr unsigned char kAzure[3] = {255, 255, 240};
constexpr unsigned char kBeige[3] = {220, 245, 245};
constexpr unsigned char kBisque[3] = {196, 228, 255};
constexpr unsigned char kBlack[3] = {0, 0, 0};
constexpr unsigned char kBlanchedAlmond[3] = {205, 235, 255};
constexpr unsigned char kBlue[3] = {255, 0, 0};
constexpr unsigned char kBlueViolet[3] = {226, 43, 138};
constexpr unsigned char kBrown[3] = {42, 42, 165};
constexpr unsigned char kBurlyWood[3] = {135, 184, 222};
constexpr unsigned char kCadetBlue[3] = {160, 158, 95};
constexpr unsigned char kChartreuse[3] = {0, 255, 127};
constexpr unsigned char kChocolate[3] = {30, 105, 210};
constexpr unsigned char kCoral[3] = {80, 127, 255};
constexpr unsigned char kCornflowerBlue[3] = {237, 149, 100};
constexpr unsigned char kCornsilk[3] = {220, 248, 255};
constexpr unsigned char kCrimson[3] = {60, 20, 220};
constexpr unsigned char kCyan[3] = {255, 255, 0};
constexpr unsigned char kDarkBlue[3] = {139, 0, 0};
constexpr unsigned char kDarkCyan[3] = {139, 139, 0};
constexpr unsigned char kDarkGoldenrod[3] = {11, 134, 184};
constexpr unsigned char kDarkGray[3] = {169, 169, 169};
constexpr unsigned char kDarkGreen[3] = {0, 100, 0};
constexpr unsigned char kDarkGrey[3] = {169, 169, 169};
constexpr unsigned char kDarkKhaki[3] = {107, 183, 189};
constexpr unsigned char kDarkMagenta[3] = {139, 0, 139};
constexpr unsigned char kDarkOlivegreen[3] = {47, 107, 85};
constexpr unsigned char kDarkOrange[3] = {0, 140, 255};
constexpr unsigned char kDarkOrchid[3] = {204, 50, 153};
constexpr unsigned char kDarkRed[3] = {0, 0, 139};
constexpr unsigned char kDarkSalmon[3] = {122, 150, 233};
constexpr unsigned char kDarkSeaGreen[3] = {143, 188, 143};
constexpr unsigned char kDarkSlateBlue[3] = {139, 61, 72};
constexpr unsigned char kDarkSlateGray[3] = {79, 79, 47};
constexpr unsigned char kDarkSlateGrey[3] = {79, 79, 47};
constexpr unsigned char kDarkTurquoise[3] = {209, 206, 0};
constexpr unsigned char kDarkViolet[3] = {211, 0, 148};
constexpr unsigned char kDeepPink[3] = {147, 20, 255};
constexpr unsigned char kDeepSkyBlue[3] = {255, 191, 0};
constexpr unsigned char kDimGray[3] = {105, 105, 105};
constexpr unsigned char kDimGrey[3] = {105, 105, 105};
constexpr unsigned char kDodgerBlue[3] = {255, 144, 30};
constexpr unsigned char kFireBrick[3] = {34, 34, 178};
constexpr unsigned char kFloralWhite[3] = {240, 250, 255};
constexpr unsigned char kForestGreen[3] = {34, 139, 34};
constexpr unsigned char kFuchsia[3] = {255, 0, 255};
constexpr unsigned char kGainsboro[3] = {220, 220, 220};
constexpr unsigned char kGhostWhite[3] = {255, 248, 248};
constexpr unsigned char kGold[3] = {0, 215, 255};
constexpr unsigned char kGoldenrod[3] = {32, 165, 218};
constexpr unsigned char kGray[3] = {128, 128, 128};
constexpr unsigned char kGreen[3] = {0, 128, 0};
constexpr unsigned char kGreenYellow[3] = {47, 255, 173};
constexpr unsigned char kGrey[3] = {128, 128, 128};
constexpr unsigned char kHoneydew[3] = {240, 255, 240};
constexpr unsigned char kHotPink[3] = {180, 105, 255};
constexpr unsigned char kIndianRed[3] = {92, 92, 205};
constexpr unsigned char kIndigo[3] = {130, 0, 75};
constexpr unsigned char kIvory[3] = {240, 255, 255};
constexpr unsigned char kKhaki[3] = {140, 230, 240};
constexpr unsigned char kLavender[3] = {250, 230, 230};
constexpr unsigned char kLavenderBlush[3] = {245, 240, 255};
constexpr unsigned char kLawnGreen[3] = {0, 252, 124};
constexpr unsigned char kLemonChiffon[3] = {205, 250, 255};
constexpr unsigned char kLightBlue[3] = {230, 216, 173};
constexpr unsigned char kLightCoral[3] = {128, 128, 240};
constexpr unsigned char kLightCyan[3] = {255, 255, 224};
constexpr unsigned char kLightGoldenrodYellow[3] = {210, 250, 250};
constexpr unsigned char kLightGray[3] = {211, 211, 211};
constexpr unsigned char kLightGreen[3] = {144, 238, 144};
constexpr unsigned char kLightGrey[3] = {211, 211, 211};
constexpr unsigned char kLightPink[3] = {193, 182, 255};
constexpr unsigned char kLightSalmon[3] = {122, 160, 255};
constexpr unsigned char kLightSeaGreen[3] = {170, 178, 32};
constexpr unsigned char kLightSkyBlue[3] = {250, 206, 135};
constexpr unsigned char kLightSlateGray[3] = {153, 136, 119};
constexpr unsigned char kLightSlateGrey[3] = {153, 136, 119};
constexpr unsigned char kLightSteelBlue[3] = {222, 196, 176};
constexpr unsigned char kLightYellow[3] = {224, 255, 255};
constexpr unsigned char kLime[3] = {0, 255, 0};
constexpr unsigned char kLimeGreen[3] = {50, 205, 50};
constexpr unsigned char kLinen[3] = {230, 240, 250};
constexpr unsigned char kMagenta[3] = {255, 0, 255};
constexpr unsigned char kMaroon[3] = {0, 0, 128};
constexpr unsigned char kMediumAquamarine[3] = {170, 205, 102};
constexpr unsigned char kMediumBlue[3] = {205, 0, 0};
constexpr unsigned char kMediumOrchid[3] = {211, 85, 186};
constexpr unsigned char kMediumPurple[3] = {219, 112, 147};
constexpr unsigned char kMediumSeaGreen[3] = {113, 179, 60};
constexpr unsigned char kMediumSlateBlue[3] = {238, 104, 123};
constexpr unsigned char kMediumSpringGreen[3] = {154, 250, 0};
constexpr unsigned char kMediumTurquoise[3] = {204, 209, 72};
constexpr unsigned char kMediumVioletRed[3] = {133, 21, 199};
constexpr unsigned char kMidnightBlue[3] = {112, 25, 25};
constexpr unsigned char kMintCream[3] = {250, 255, 245};
constexpr unsigned char kMistyRose[3] = {225, 228, 255};
constexpr unsigned char kMoccasin[3] = {181, 228, 255};
constexpr float kMRTransparency = 0.4f;
constexpr int kMRNumColors = 79;
constexpr unsigned char kNavajoWhite[3] = {173, 222, 255};
constexpr unsigned char kNavy[3] = {128, 0, 0};
constexpr unsigned char kOldLace[3] = {230, 245, 253};
constexpr unsigned char kOlive[3] = {0, 128, 128};
constexpr unsigned char kOliveDrab[3] = {35, 142, 107};
constexpr unsigned char kOrange[3] = {0, 165, 255};
constexpr unsigned char kOrangeRed[3] = {0, 69, 255};
constexpr unsigned char kOrchid[3] = {214, 112, 218};
constexpr unsigned char kPaleGoldenrod[3] = {170, 232, 238};
constexpr unsigned char kPaleGreen[3] = {152, 251, 152};
constexpr unsigned char kPaleTurquoise[3] = {238, 238, 175};
constexpr unsigned char kPaleVioletRed[3] = {147, 112, 219};
constexpr unsigned char kPapayawhip[3] = {213, 239, 255};
constexpr unsigned char kPeachPuff[3] = {185, 218, 255};
constexpr unsigned char kPeru[3] = {63, 133, 205};
constexpr unsigned char kPink[3] = {203, 192, 255};
constexpr unsigned char kPlum[3] = {221, 160, 221};
constexpr unsigned char kPowderBlue[3] = {230, 224, 176};
constexpr unsigned char kPurple[3] = {128, 0, 128};
constexpr unsigned char kRed[3] = {0, 0, 255};
constexpr unsigned char kRosyBrown[3] = {143, 143, 188};
constexpr unsigned char kRoyalBlue[3] = {225, 105, 65};
constexpr unsigned char kSaddleBrown[3] = {19, 69, 139};
constexpr unsigned char kSalmon[3] = {114, 128, 250};
constexpr unsigned char kSandyBrown[3] = {96, 164, 244};
constexpr unsigned char kSeaGreen[3] = {87, 139, 46};
constexpr unsigned char kSeashell[3] = {238, 245, 255};
constexpr unsigned char kSienna[3] = {45, 82, 160};
constexpr unsigned char kSilver[3] = {192, 192, 192};
constexpr unsigned char kSkyBlue[3] = {235, 206, 135};
constexpr unsigned char kSlateBlue[3] = {205, 90, 106};
constexpr unsigned char kSlateGray[3] = {144, 128, 112};
constexpr unsigned char kSlateGrey[3] = {144, 128, 112};
constexpr unsigned char kSnow[3] = {250, 250, 255};
constexpr unsigned char kSpringGreen[3] = {127, 255, 0};
constexpr unsigned char kSteelBlue[3] = {180, 130, 70};
constexpr unsigned char kTan[3] = {140, 180, 210};
constexpr unsigned char kTeal[3] = {128, 128, 0};
constexpr unsigned char kThistle[3] = {216, 191, 216};
constexpr unsigned char kTomato[3] = {71, 99, 255};
constexpr unsigned char kTurquoise[3] = {208, 224, 64};
constexpr unsigned char kViolet[3] = {238, 130, 238};
constexpr unsigned char kWheat[3] = {179, 222, 245};
constexpr unsigned char kWhite[3] = {255, 255, 255};
constexpr unsigned char kWhiteSmoke[3] = {245, 245, 245};
constexpr unsigned char kYellow[3] = {0, 255, 255};
constexpr unsigned char kYellowGreen[3] = {50, 205, 154};

}; // namespace color

} // namespace mjolnir
