
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
#include "structures.h"

using cv::Mat;
using cv::Point2f;
using cv::Scalar;

namespace mjolnir {
namespace vis {

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
inline cv::Scalar toCvColor(const unsigned char *c) {
  return cv::Scalar(c[0], c[1], c[2]);
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
} // namespace mjolnir
