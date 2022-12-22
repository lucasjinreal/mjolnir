#include "vis.h"
#include "simpleocv.h"
#include "type.h"
#include <cassert>
#include <cstdio>

#ifdef USE_OPENCV
#include <opencv2/imgproc.hpp>
#endif

namespace mjolnir {
namespace vis {

RGBA gen_unique_color(int idx, bool is_track, double hue_step, float alpha) {
  // if idx is track id, the color should be
  if (is_track) {
    // we may have 1000+ track ids
    int track_size = 1. / hue_step;
    idx = idx % track_size;
  }

  float h = idx * hue_step - (int)(idx * hue_step);
  double v = 1.0 - (int(idx * hue_step) % 6) / 5.;
  // printf("h: %f, v: %f idx %d hue_step: %f \n", h, v, idx, hue_step);
  RGBA rgba;
  hsv2rgb(rgba, h, 1, v);
  rgba.r = 255 * rgba.r;
  rgba.g = 255 * rgba.g;
  rgba.b = 255 * rgba.b;
  rgba.a = alpha;
  return rgba;
}

void hsv2rgb(RGBA &rgba, float h, float s, float v) {
  if (s == 0) {
    rgba.r = v;
    rgba.g = v;
    rgba.b = v;
    return;
  }
  int i = h * 6.0;
  float f = (h * 6.0) - i;
  float p = v * (1.0 - s);
  float q = v * (1.0 - s * f);
  float t = v * (1.0 - s * (1.0 - f));
  i = i % 6;
  switch (i) {
  case 0:
    rgba.r = v;
    rgba.g = t;
    rgba.b = p;
    break;
  case 1:
    rgba.r = q;
    rgba.g = v;
    rgba.b = p;
    break;
  case 2:
    rgba.r = p;
    rgba.g = v;
    rgba.b = t;
    break;
  case 3:
    rgba.r = p;
    rgba.g = q;
    rgba.b = v;
    break;
  case 4:
    rgba.r = t;
    rgba.g = p;
    rgba.b = v;
    break;
  case 5:
    rgba.r = v;
    rgba.g = p;
    rgba.b = q;
    break;
  default:
    break;
  }
}

void hsv2rgb(float &r, float &g, float &b, float h, float s, float v) {
  double hh, p, q, t, ff;
  long i;
  if (s <= 0.0) { // < is bogus, just shuts up warnings
    r = float(v);
    g = float(v);
    b = float(v);
  }
  hh = h;
  if (hh >= 360.0)
    hh = 0.0;
  hh /= 60.0;
  i = (long)hh;
  ff = hh - i;
  p = v * (1.0 - s);
  q = v * (1.0 - (s * ff));
  t = v * (1.0 - (s * (1.0 - ff)));
  switch (i) {
  case 0:
    r = v;
    g = t;
    b = p;
    break;
  case 1:
    r = q;
    g = v;
    b = p;
    break;
  case 2:
    r = p;
    g = v;
    b = t;
    break;

  case 3:
    r = p;
    g = q;
    b = v;
    break;
  case 4:
    r = t;
    g = p;
    b = v;
    break;
  case 5:
  default:
    r = v;
    g = p;
    b = q;
    break;
  }
}

cv::Scalar gen_unique_color_cv(int idx, bool is_track, double hue_step,
                               float alpha) {
  RGBA cr = gen_unique_color(idx, is_track, hue_step, alpha);
  // cr.print();
  int r = cr.r;
  int g = cr.g;
  int b = cr.b;
  cv::Scalar c(r, g, b);
  return c;
}

#ifdef USE_OPENCV
cv::Mat createAlpha(cv::Mat &src) {
  cv::Mat alpha = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
  cv::Mat gray = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
  cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);
  for (int i = 0; i < src.rows; i++) {
    for (int j = 0; j < src.cols; j++) {
      alpha.at<uchar>(i, j) = 255 - gray.at<uchar>(i, j);
    }
  }
  return alpha;
}

int addAlpha(cv::Mat &src, cv::Mat &dst, cv::Mat &alpha) {
  if (src.channels() == 4) {
    return -1;
  } else if (src.channels() == 1) {
    cv::cvtColor(src, src, cv::COLOR_GRAY2RGB);
  }

  dst = cv::Mat(src.rows, src.cols, CV_8UC4);

  std::vector<cv::Mat> srcChannels;
  std::vector<cv::Mat> dstChannels;
  cv::split(src, srcChannels);

  dstChannels.push_back(srcChannels[0]);
  dstChannels.push_back(srcChannels[1]);
  dstChannels.push_back(srcChannels[2]);
  dstChannels.push_back(alpha);
  cv::merge(dstChannels, dst);
  return 0;
}
#endif

////////////// Vis functions ////////////
void renderHumanPose(std::vector<HumanPose> &poses, cv::Mat &image) {
  // drawing HumanPoses on image
  assert(image.type() == CV_8UC3);
  const std::vector<cv::Scalar> colors = {
      cv::Scalar(255, 0, 0),   cv::Scalar(255, 85, 0),  cv::Scalar(255, 170, 0),
      cv::Scalar(255, 255, 0), cv::Scalar(170, 255, 0), cv::Scalar(85, 255, 0),
      cv::Scalar(0, 255, 0),   cv::Scalar(0, 255, 85),  cv::Scalar(0, 255, 170),
      cv::Scalar(0, 255, 255), cv::Scalar(0, 170, 255), cv::Scalar(0, 85, 255),
      cv::Scalar(0, 0, 255),   cv::Scalar(85, 0, 255),  cv::Scalar(170, 0, 255),
      cv::Scalar(255, 0, 255), cv::Scalar(255, 0, 170), cv::Scalar(255, 0, 85)};
  const std::vector<std::pair<int, int>> limbKeypointsIds = {
      {1, 2}, {1, 5},  {2, 3},   {3, 4},  {5, 6},   {6, 7},
      {1, 8}, {8, 9},  {9, 10},  {1, 11}, {11, 12}, {12, 13},
      {1, 0}, {0, 14}, {14, 16}, {0, 15}, {15, 17}};

  const int stickWidth = 3;
  const cv::Point2f absentKeypoint(-1.0f, -1.0f);
  for (const auto &pose : poses) {
    // we only support 18 keypoints
    assert(pose.keypoints.size() == 18);

    for (size_t keypointIdx = 0; keypointIdx < pose.keypoints.size();
         keypointIdx++) {
      auto kpt = pose.keypoints[keypointIdx];
      if (kpt != absentKeypoint) {
        cv::circle(image, pose.keypoints[keypointIdx], 3, colors[keypointIdx],
                   -1);
      }
    }
  }
  cv::Mat pane = image.clone();
  for (auto &pose : poses) {
    for (const auto &limbKeypointsId : limbKeypointsIds) {
      std::pair<cv::Point2f, cv::Point2f> limbKeypoints(
          pose.keypoints[limbKeypointsId.first],
          pose.keypoints[limbKeypointsId.second]);
      if (limbKeypoints.first == absentKeypoint ||
          limbKeypoints.second == absentKeypoint) {
        continue;
      }

      float meanX = (limbKeypoints.first.x + limbKeypoints.second.x) / 2;
      float meanY = (limbKeypoints.first.y + limbKeypoints.second.y) / 2;
      cv::Point difference = limbKeypoints.first - limbKeypoints.second;
      double length =
          std::sqrt(difference.x * difference.x + difference.y * difference.y);
      int angle = static_cast<int>(std::atan2(difference.y, difference.x) *
                                   180 / CV_PI);
      std::vector<cv::Point> polygon;
#ifdef USE_OPENCV
      cv::ellipse2Poly(cv::Point2d(meanX, meanY),
                       cv::Size2d(length / 2, stickWidth), angle, 0, 360, 1,
                       polygon);
      cv::fillConvexPoly(pane, polygon, colors[limbKeypointsId.second]);
#endif
    }
    // for every pose, if pose has pose_id, means it is tracked
    if (pose.pose_id != -1) {
      // we draw this id
      Box b = pose.to_box();
      cv::putText(image, to_string(pose.pose_id), Point2f(b.xmin, b.ymin),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255));
      cv::rectangle(image, Point2f(b.xmin, b.ymin), Point2f(b.xmax, b.ymax),
                    Scalar(255, 0, 0), 1);
    }
  }
#ifdef USE_OPENCV
  cv::addWeighted(image, 0.6, pane, 0.7, 0, image);
#endif
}

void renderHumanPoseSimple(std::vector<HumanPose> &poses, cv::Mat &image) {
  // a more simple render of human pose estimation
  // we can define our own limbKeypoints to unlink some joint
  const std::vector<std::pair<int, int>> limbKeypointsIds = {
      {1, 2}, {1, 5},  {2, 3},   {3, 4},  {5, 6},   {6, 7},
      {1, 8}, {8, 9},  {9, 10},  {1, 11}, {11, 12}, {12, 13},
      {1, 0}, {0, 14}, {14, 16}, {0, 15}, {15, 17}};
  const cv::Point2f absentKeypoint(-1.0f, -1.0f);

  for (auto &pose : poses) {

    for (int i = 0; i < pose.keypoints.size(); ++i) {
      circle(image, pose.keypoints[i], 8, Scalar(0, 255, 255), -1);
    }

    for (const auto &limbKeypointsId : limbKeypointsIds) {
      std::pair<cv::Point2f, cv::Point2f> limbKeypoints(
          pose.keypoints[limbKeypointsId.first],
          pose.keypoints[limbKeypointsId.second]);
      if (limbKeypoints.first == absentKeypoint ||
          limbKeypoints.second == absentKeypoint) {
        continue;
      }
      cv::line(image, limbKeypoints.first, limbKeypoints.second,
               Scalar(255, 255, 255), 2);
    }
    if (pose.pose_id != -1) {
      // we draw this id
      Box b = pose.to_box();
      cv::putText(image, to_string(pose.pose_id), Point2f(b.xmin, b.ymin),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
      cv::rectangle(image, Point2f(b.xmin, b.ymin), Point2f(b.xmax, b.ymax),
                    Scalar(255, 255, 255), 1);
    }
  }
}

void renderPoseCoco17(std::vector<HumanPose> &poses, cv::Mat &image) {
  const std::vector<std::pair<int, int>> limbKeypointsIds = {
      {16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12}, {7, 13},
      {6, 7},   {6, 8},   {7, 9},   {8, 10},  {9, 11},  {2, 3},  {1, 2},
      {1, 3},   {2, 4},   {3, 5},   {4, 6},   {5, 7},
  };
  const cv::Point2f absentKeypoint(-1.0f, -1.0f);

  for (auto &pose : poses) {

    for (int i = 0; i < pose.keypoints.size(); ++i) {
      circle(image, pose.keypoints[i], 8, Scalar(0, 255, 255), -1);
    }

    for (const auto &limbKeypointsId : limbKeypointsIds) {
      std::pair<cv::Point2f, cv::Point2f> limbKeypoints(
          pose.keypoints[limbKeypointsId.first - 1],
          pose.keypoints[limbKeypointsId.second - 1]);
      if (limbKeypoints.first == absentKeypoint ||
          limbKeypoints.second == absentKeypoint) {
        continue;
      }
      cv::line(image, limbKeypoints.first, limbKeypoints.second,
               Scalar(255, 255, 255), 3);
    }
    if (pose.pose_id != -1) {
      // we draw this id
      Box b = pose.to_box();
      cv::putText(image, to_string(pose.pose_id), Point2f(b.xmin, b.ymin),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
      cv::rectangle(image, Point2f(b.xmin, b.ymin), Point2f(b.xmax, b.ymax),
                    Scalar(255, 255, 255), 1);
    }
  }
}

// Draw boxes
cv::Mat VisualizeDet(cv::Mat &img, vector<vector<float>> detections,
                     vector<string> classes_names, bool enable_mask,
                     float confidence_threshold, bool normalized) {
  // for visualize
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const float font_scale = 0.6;
  const int font_thickness = 2;
  cv::Mat mask = cv::Mat(img);
  for (int i = 0; i < detections.size(); ++i) {
    const vector<float> &d = detections[i];
    // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
    // CHECK_EQ(d.size(), 7);
    int offset = 0;
    if (d.size() == 7) {
      offset = 1;
    }
    // cout << "l: " << d[offset + 0] << " score: " << d[offset + 1] << endl;
    const float score = d[1 + offset];
    if (score >= confidence_threshold) {
      cv::Point pt1, pt2;
      if (normalized) {
        pt1.x = (img.cols * d[2 + offset]);
        pt1.y = (img.rows * d[3 + offset]);
        pt2.x = (img.cols * d[4 + offset]);
        pt2.y = (img.rows * d[5 + offset]);
      } else {
        pt1.x = d[2 + offset];
        pt1.y = d[3 + offset];
        pt2.x = d[4 + offset];
        pt2.y = d[5 + offset];
      }

      cv::Scalar u_c = gen_unique_color_cv(d[offset]);
      cv::rectangle(img, pt1, pt2, u_c, 2, 8, 0);
      cv::rectangle(mask, pt1, pt2, u_c, cv::FILLED, 0);

      char score_str[256];
      sprintf(score_str, "%.2f", score);
      std::string label_text =
          classes_names[d[offset]] + " " + string(score_str);
      int base_line = 0;
      cv::Point text_origin = cv::Point(pt1.x - 2, pt1.y - 3);
      cv::Size text_size = cv::getTextSize(label_text, font, font_scale,
                                           font_thickness, &base_line);
      cv::rectangle(img, cv::Point(text_origin.x, text_origin.y + 5),
                    cv::Point(text_origin.x + text_size.width,
                              text_origin.y - text_size.height - 5),
                    u_c, -1, 0);
      cv::putText(img, label_text, text_origin, font, font_scale,
                  cv::Scalar(0, 0, 0), font_thickness);
    }
  }
  cv::Mat combined;
#ifdef USE_OPENCV
  cv::addWeighted(img, 0.8, mask, 0.6, 0.6, combined);
#endif
  // maybe combine a mask img back later
  return combined;
}

cv::Mat VisualizeBox(cv::Mat &img, vector<mjolnir::Box> detections,
                     vector<string> classes_names, bool enable_mask,
                     const vector<cv::Scalar> *colors,
                     const float line_thickness, const float font_scale,
                     float confidence_threshold, bool normalized) {

  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const int font_thickness = 1;

  cv::Mat mask;
  mask = cv::Mat::zeros(img, CV_8UC3);

  for (int i = 0; i < detections.size(); ++i) {
    mjolnir::Box box = detections[i];
    box.to_xyxy();
    const float score = box.score;
    if (score >= confidence_threshold) {
      cv::Point pt1, pt2;
      if (normalized) {
        pt1.x = (img.cols * box.xmin);
        pt1.y = (img.rows * box.ymin);
        pt2.x = (img.cols * box.xmax);
        pt2.y = (img.rows * box.ymax);
      } else {
        pt1.x = box.xmin;
        pt1.y = box.ymin;
        pt2.x = box.xmax;
        pt2.y = box.ymax;
      }

      cv::Scalar u_c;
      if (colors != nullptr) {
        u_c = (*colors)[box.idx];
      } else {
        u_c = gen_unique_color_cv(box.idx + 5);
      }
      cv::rectangle(img, pt1, pt2, u_c, line_thickness, cv::LINE_AA, 0);
      if (enable_mask) {
        cv::rectangle(mask, pt1, pt2, u_c, cv::FILLED, 0);
      }

      // CV_FONT_HERSHEY_DUPLEX

      char score_str[256];
      snprintf(score_str, sizeof(score_str), "%.1f", score);
      std::string label_text = classes_names[box.idx] + " " + string(score_str);
      const int padding_y = 4;
      const int padding_x = 2;
      int base_line = 0;
      cv::Size text_size = cv::getTextSize(label_text, font, font_scale,
                                           font_thickness, &base_line);
      cv::Point text_origin =
          cv::Point(pt1.x + padding_x, pt1.y - padding_y); // bottom_left

      cv::rectangle(
          mask, cv::Point(pt1.x, pt1.y - text_size.height - 2 * padding_y),
          cv::Point(text_origin.x + text_size.width + padding_x, pt1.y), u_c,
          -1, 0);
      cv::putText(img, label_text, text_origin, font, font_scale,
                  cv::Scalar(255, 255, 255), font_thickness);
    }
  }
#ifdef USE_OPENCV
  cv::addWeighted(img, 0.9, mask, 0.9, 0.5, img);
#endif
  return img;
}

cv::Mat VisualizeDetections(cv::Mat &img, vector<mjolnir::Detection> detections,
                            const vector<string> classes_names,
                            const vector<cv::Scalar> *colors,
                            const float line_thickness, const float font_scale,
                            const bool fancy, const float confidence_threshold,
                            const bool enable_mask, const bool normalized) {
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const int font_thickness = 1;

  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC3);

  for (int i = 0; i < detections.size(); ++i) {
    mjolnir::Detection det = detections[i];
    const auto score = static_cast<float>(det.score);
    if (score >= confidence_threshold) {
      cv::Point pt1, pt2;
      if (normalized) {
        pt1.x = (img.cols * det.x1);
        pt1.y = (img.rows * det.y1);
        pt2.x = (img.cols * det.x2);
        pt2.y = (img.rows * det.y2);
      } else {
        pt1.x = det.x1;
        pt1.y = det.y1;
        pt2.x = det.x2;
        pt2.y = det.y2;
      }

      cv::Scalar u_c;
      if (colors != nullptr) {
        u_c = (*colors)[det.idx];
      } else {
        u_c = gen_unique_color_cv(det.idx);
      }
      // printf("%d %d %d \n", u_c[0], u_c[1], u_c[2]);

      cv::rectangle(img, pt1, pt2, u_c, line_thickness, cv::LINE_AA, 0);
      if (enable_mask) {
        cv::rectangle(mask, pt1, pt2, u_c, cv::FILLED, 0);
      }

      // CV_FONT_HERSHEY_DUPLEX

      char score_str[256];
      snprintf(score_str, sizeof(score_str), "%.1f", score * 100);
      std::string label_text =
          classes_names[det.idx] + " " + string(score_str) + "%";
      int base_line = 4;
      cv::Point text_origin = cv::Point(pt1.x + 2, pt1.y - base_line);
      cv::Size text_size = cv::getTextSize(label_text, font, font_scale,
                                           font_thickness, &base_line);
      cv::rectangle(
          mask, cv::Point(pt1.x, text_origin.y - text_size.height - base_line),
          cv::Point(text_origin.x + text_size.width + 2, pt1.y), u_c, -1, 0);
      cv::putText(img, label_text, text_origin, font, font_scale,
                  cv::Scalar(255, 255, 255), font_thickness);
    }
  }

  if (enable_mask) {
    cv::Mat combined;
#ifdef USE_OPENCV
    cv::addWeighted(img, 0.8, mask, 0.6, 0.6, combined);
#endif
    // maybe combine a mask img back later
    return combined;
  } else {
    return img;
  }
}

cv::Mat VisualizeDetectionsWithLandmark(
    cv::Mat &img, vector<mjolnir::Detection> detections,
    const vector<string> classes_names, const bool enable_mask,
    const bool landmark_on, const vector<cv::Scalar> *colors,
    const float line_thickness, const float font_scale, const bool fancy,
    const float confidence_threshold, const bool normalized) {

  // draw simple box with class names and landmarks, enable_mask not used here
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const int font_thickness = 1;

  cv::Mat mask;
  if (enable_mask) {
    mask = cv::Mat::zeros(img.size(), CV_8UC3);
  }

  for (int i = 0; i < detections.size(); ++i) {
    mjolnir::Detection det = detections[i];
    const auto score = static_cast<float>(det.score);
    if (score >= confidence_threshold) {
      cv::Point pt1, pt2;
      if (normalized) {
        pt1.x = (img.cols * det.x1);
        pt1.y = (img.rows * det.y1);
        pt2.x = (img.cols * det.x2);
        pt2.y = (img.rows * det.y2);
      } else {
        pt1.x = det.x1;
        pt1.y = det.y1;
        pt2.x = det.x2;
        pt2.y = det.y2;
      }

      cv::Scalar u_c;
      if (colors != nullptr) {
        u_c = (*colors)[det.idx];
      } else {
        u_c = gen_unique_color_cv(det.idx + 8);
      }
      cv::rectangle(img, pt1, pt2, u_c, line_thickness, cv::LINE_AA, 0);
      if (enable_mask) {
        cv::rectangle(mask, pt1, pt2, u_c, cv::FILLED, 0);
      }

      char score_str[256];
      snprintf(score_str, sizeof(score_str), "%.1f", score);
      std::string label_text = classes_names[det.idx] + " " + string(score_str);
      const int padding_y = 4;
      const int padding_x = 2;
      int base_line = 0;
      cv::Size text_size = cv::getTextSize(label_text, font, font_scale,
                                           font_thickness, &base_line);
      cv::Point text_origin =
          cv::Point(pt1.x + padding_x, pt1.y - padding_y); // bottom_left

      if (enable_mask) {
        cv::rectangle(
            mask, cv::Point(pt1.x, pt1.y - text_size.height - 2 * padding_y),
            cv::Point(text_origin.x + text_size.width + padding_x, pt1.y), u_c,
            -1, 0);
      } else {
        cv::rectangle(
            img, cv::Point(pt1.x, pt1.y - text_size.height - 2 * padding_y),
            cv::Point(text_origin.x + text_size.width + padding_x, pt1.y), u_c,
            -1, 0);
      }
      cv::putText(img, label_text, text_origin, font, font_scale,
                  cv::Scalar(255, 255, 255), font_thickness);

      // draw landmark
      if (landmark_on) {
        for (int j = 0; j < det.landmarks.size(); ++j) {
          LandmarkPoint p = det.landmarks[j];
          cv::circle(img, cv::Point(p.x, p.y), 2, u_c, -1);
        }
      }
    }
  }

  if (enable_mask) {
#ifdef USE_OPENCV
    cv::Mat combined;
    cv::addWeighted(img, 0.8, mask, 0.6, 0.6, combined);
    // maybe combine a mask img back later
    return combined;
#else
    return img;
#endif
  } else {
    return img;
  }
}

cv::Mat VisualizeDetectionsWithOverrideColors(
    cv::Mat &img, vector<mjolnir::Detection> detections,
    const vector<string> classes_names,
    const std::map<int, cv::Scalar> *override_colors,
    const float line_thickness, const bool with_text, const float font_scale,
    const bool fancy, const float confidence_threshold, const bool enable_mask,
    const bool normalized) {
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const int font_thickness = 1;

  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC3);

  for (int i = 0; i < detections.size(); ++i) {
    mjolnir::Detection det = detections[i];
    const auto score = static_cast<float>(det.score);
    if (score >= confidence_threshold) {
      cv::Point pt1, pt2;
      if (normalized) {
        pt1.x = (img.cols * det.x1);
        pt1.y = (img.rows * det.y1);
        pt2.x = (img.cols * det.x2);
        pt2.y = (img.rows * det.y2);
      } else {
        pt1.x = det.x1;
        pt1.y = det.y1;
        pt2.x = det.x2;
        pt2.y = det.y2;
      }

      cv::Scalar u_c;
      if ((override_colors != nullptr) && override_colors->count(det.idx) > 0) {
        u_c = (*override_colors).at(det.idx);
      } else {
        u_c = gen_unique_color_cv(det.idx + 8);
      }
      cv::rectangle(img, pt1, pt2, u_c, line_thickness, 8);
      if (enable_mask) {
        cv::rectangle(mask, pt1, pt2, u_c, cv::FILLED, 0);
      }

      // CV_FONT_HERSHEY_DUPLEX

      char score_str[256];
      snprintf(score_str, sizeof(score_str), "%.1f", score * 100);
      std::string label_text =
          classes_names[det.idx] + " " + string(score_str) + "%";
      int base_line = 4;
      cv::Point text_origin = cv::Point(pt1.x + 2, pt1.y - base_line);
      cv::Size text_size = cv::getTextSize(label_text, font, font_scale,
                                           font_thickness, &base_line);
      cv::rectangle(
          mask, cv::Point(pt1.x, text_origin.y - text_size.height - base_line),
          cv::Point(text_origin.x + text_size.width + 2, pt1.y), u_c, -1, 0);
      if (with_text) {
        cv::putText(img, label_text, text_origin, font, font_scale,
                    cv::Scalar(255, 255, 255), font_thickness);
      }
    }
  }

  if (enable_mask) {
#ifdef USE_OPENCV
    cv::Mat combined;
    cv::addWeighted(img, 0.8, mask, 0.6, 0.6, combined);
    // maybe combine a mask img back later
    return combined;
#else
    return img;
#endif
  } else {
    return img;
  }
}

cv::Mat VisualizeDetectionStyleDetectron2(
    cv::Mat &img, vector<mjolnir::Box> detections, vector<string> classes_names,
    bool enable_mask, float confidence_threshold, bool normalized) {
  // for detectron2 style drawing bounding boxes
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const float font_scale = 0.35;
  const int font_thickness = 1;
  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC3);
  for (int i = 0; i < detections.size(); ++i) {
    mjolnir::Box box = detections[i];
    box.to_xyxy();
    const float score = box.score;
    if (score >= confidence_threshold) {
      cv::Point pt1, pt2;
      if (normalized) {
        pt1.x = (img.cols * box.xmin);
        pt1.y = (img.rows * box.ymin);
        pt2.x = (img.cols * box.xmax);
        pt2.y = (img.rows * box.ymax);
      } else {
        pt1.x = box.xmin;
        pt1.y = box.ymin;
        pt2.x = box.xmax;
        pt2.y = box.ymax;
      }

      cv::Scalar u_c = gen_unique_color_cv(box.idx);
      cv::rectangle(img, pt1, pt2, u_c, 1, 8, 0);
      cv::rectangle(mask, pt1, pt2, u_c, cv::FILLED, 0);

      char score_str[256];
      sprintf(score_str, "%.1f", score);
      std::string label_text = classes_names[box.idx] + ":" + string(score_str);
      int base_line = 0;
      cv::Point text_origin = cv::Point(pt1.x, pt1.y - 2);
      cv::Size text_size = cv::getTextSize(label_text, font, font_scale,
                                           font_thickness, &base_line);
      cv::rectangle(img, cv::Point(text_origin.x, text_origin.y),
                    cv::Point(text_origin.x + text_size.width,
                              text_origin.y - text_size.height),
                    cv::Scalar(0, 0, 0), -1, 0);
      cv::putText(img, label_text, text_origin, font, font_scale,
                  cv::Scalar(255, 255, 255), font_thickness);
    }
  }

  if (enable_mask) {
#ifdef USE_OPENCV
    cv::Mat combined;
    cv::addWeighted(img, 0.8, mask, 0.6, 0.6, combined);
    // maybe combine a mask img back later
    return combined;
#else
    return img;
#endif
  } else {
    return img;
  }
}

cv::Mat VisualizeDetectionStyleDetectron2(cv::Mat &img,
                                          vector<mjolnir::Bbox> detections,
                                          vector<string> classes_names,
                                          bool enable_mask,
                                          float confidence_threshold,
                                          bool normalized) {
  // for detectron2 style drawing bounding boxes
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const float font_scale = 0.35;
  const int font_thickness = 1;
  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC3);
  for (int i = 0; i < detections.size(); ++i) {
    mjolnir::Bbox box = detections[i];
    const float score = box.score;
    if (score >= confidence_threshold) {
      cv::Point pt1, pt2;
      if (normalized) {
        pt1.x = (img.cols * box.xmin);
        pt1.y = (img.rows * box.ymin);
        pt2.x = (img.cols * box.xmax);
        pt2.y = (img.rows * box.ymax);
      } else {
        pt1.x = box.xmin;
        pt1.y = box.ymin;
        pt2.x = box.xmax;
        pt2.y = box.ymax;
      }

      cv::Scalar u_c = gen_unique_color_cv(box.cid);
      cv::rectangle(img, pt1, pt2, u_c, 1, 8, 0);
      cv::rectangle(mask, pt1, pt2, u_c, cv::FILLED, 0);

      char score_str[256];
      sprintf(score_str, "%.1f", score);
      std::string label_text = classes_names[box.cid] + ":" + string(score_str);
      int base_line = 0;
      cv::Point text_origin = cv::Point(pt1.x, pt1.y - 2);
      cv::Size text_size = cv::getTextSize(label_text, font, font_scale,
                                           font_thickness, &base_line);
      cv::rectangle(img, cv::Point(text_origin.x, text_origin.y),
                    cv::Point(text_origin.x + text_size.width,
                              text_origin.y - text_size.height),
                    cv::Scalar(0, 0, 0), -1, 0);
      cv::putText(img, label_text, text_origin, font, font_scale,
                  cv::Scalar(255, 255, 255), font_thickness);
    }
  }

  if (enable_mask) {
#ifdef USE_OPENCV
    cv::Mat combined;
    cv::addWeighted(img, 0.8, mask, 0.6, 0.6, combined);
    // maybe combine a mask img back later
    return combined;
#else
    return img;
#endif
  } else {
    return img;
  }
}

void VisTextInfos(cv::Mat &img, const vector<std::string> txts,
                  cv::Scalar color, const cv::Point start_pt) {
  for (int i = 0; i < txts.size(); ++i) {
    cv::putText(img, txts[i], cv::Point(start_pt.x, start_pt.y + i * 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.58, color, 2);
  }
}
/////////////////////////// Visualize Lanes ////////////////////
cv::Mat VisualizeLanes(cv::Mat &img, const vector<vector<cv::Point>> &lanes,
                       const vector<cv::Scalar> *colors,
                       const float line_thickness, const float alpha,
                       const bool guide_line) {
  // do some alpha effect
  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC3);
  for (int i = 0; i < lanes.size(); i++) {
    // draw a lane
    cv::Scalar u_c;
    if (colors != nullptr) {
      u_c = (*colors)[i];
    } else {
      u_c = gen_unique_color_cv(i + 6);
    }
// draw line
#ifdef USE_OPENCV
    cv::polylines(mask, lanes[i], false, u_c, line_thickness, cv::LINE_AA);
#endif
  }

#ifdef USE_OPENCV
  cv::Mat combined;
  cv::addWeighted(img, 0.8, mask, 0.6, 0.6, combined);
  // maybe combine a mask img back later
  return combined;
#else
  return img;
#endif
}

} // namespace vis

namespace image {

static const float kMean[3] = {0.485f, 0.456f, 0.406f};
static const float kStdDev[3] = {0.229f, 0.224f, 0.225f};
static const int map_[7][3] = {{0, 0, 0},   {128, 0, 0},   {0, 128, 0},
                               {0, 0, 128}, {128, 128, 0}, {128, 0, 128},
                               {0, 128, 0}};

#ifdef USE_OPENCV

float *Normalize(cv::Mat img) {
  // cv::Mat image(img.rows, img.cols, CV_32FC3);
  float *data;
  data = (float *)calloc(img.rows * img.cols * 3, sizeof(float));
  for (int c = 0; c < 3; ++c) {
    for (int i = 0; i < img.rows; ++i) { // 获取第i行首像素指针
      cv::Vec3b *p1 = img.ptr<cv::Vec3b>(i);
      // cv::Vec3b *p1 = image.ptr<cv::Vec3b>(i);
      for (int j = 0; j < img.cols; ++j) {
        data[c * img.cols * img.rows + i * img.cols + j] =
            (p1[j][c] / 255. - kMean[c]) / kStdDev[c];
      }
    }
  }
  return data;
}

float *HWC2CHW(cv::Mat img, const float kMeans[3], const float kStds[3]) {
  // convert HWC input normal image into CHW format
  // we have to make sure img data type is float
  float *data;
  data = (float *)calloc(img.rows * img.cols * 3, sizeof(float));
  for (int c = 0; c < 3; ++c) {
    for (int i = 0; i < img.rows; ++i) {
      cv::Vec3b *p1 = img.ptr<cv::Vec3b>(i);
      for (int j = 0; j < img.cols; ++j) {
        data[c * img.cols * img.rows + i * img.cols + j] =
            (p1[j][c] - kMeans[c]) / kStds[c];
      }
    }
  }
  return data;
}

cv::Mat read2mat(float *prob, cv::Mat out) {
  for (int i = 0; i < 128; ++i) {
    cv::Vec<float, 7> *p1 = out.ptr<cv::Vec<float, 7>>(i);
    for (int j = 0; j < 128; ++j) {
      for (int c = 0; c < 7; ++c) {
        p1[j][c] = prob[c * 128 * 128 + i * 128 + j];
      }
    }
  }
  return out;
}

cv::Mat map2threeunchar(cv::Mat real_out, cv::Mat real_out_) {
  for (int i = 0; i < 512; ++i) {
    cv::Vec<float, 7> *p1 = real_out.ptr<cv::Vec<float, 7>>(i);
    cv::Vec3b *p2 = real_out_.ptr<cv::Vec3b>(i);
    for (int j = 0; j < 512; ++j) {
      int index = 0;
      float swap;
      for (int c = 0; c < 7; ++c) {
        if (p1[j][0] < p1[j][c]) {
          swap = p1[j][0];
          p1[j][0] = p1[j][c];
          p1[j][c] = swap;
          index = c;
        }
      }
      p2[j][0] = map_[index][2];
      p2[j][1] = map_[index][1];
      p2[j][2] = map_[index][0];
    }
  }
  return real_out_;
}
#endif

cv::Mat resizeAlongShortest(cv::Mat img, int target_w, int target_h) {
  cv::Mat intermediateImg, outputImg;
  int delta_w, delta_h, top, left, bottom, right;
  int new_w = img.size().width;
  int new_h = img.size().height;

  if (static_cast<float>(target_w / img.size().width) <
      static_cast<float>(target_h / img.size().height)) {
    new_w = target_w;
    new_h = (img.size().height * target_w) / img.size().width;
  } else {
    new_h = target_h;
    new_w = (img.size().width * target_h) / img.size().height;
  }
  cv::resize(img, intermediateImg, cv::Size(new_w, new_h));
  float w_scale = target_w / static_cast<float>(new_w);
  float h_scale = target_h / static_cast<float>(new_h);
  delta_w = target_w - new_w;
  delta_h = target_h - new_h;
  top = floor(delta_h / 2);
  bottom = delta_h - floor(delta_h / 2);
  left = floor(delta_w / 2);
  right = delta_w - floor(delta_w / 2);
#ifdef USE_OPENCV
  cv::copyMakeBorder(intermediateImg, outputImg, top, bottom, left, right,
                     cv::BORDER_CONSTANT);
#endif
  return outputImg;
}

} // namespace image

} // namespace mjolnir
