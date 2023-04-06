#pragma once

#include "iostream"
#include <algorithm>
#include <vector>

#if defined(USE_OPENCV) || defined(_WIN32)
#include "opencv2/opencv.hpp"
#else
#include "simpleocv.h"
#endif


using namespace std;

namespace mjolnir {
enum BoxFormat {
  XYXY,
  TLWH,
  BOTH,
};

struct Bbox {
  float xmin;
  float ymin;
  float xmax;
  float ymax;
  float score;
  int cid;
};
struct InstanceSegmentation {
  // Bbox box;

  float xmin;
  float ymin;
  float xmax;
  float ymax;
  float score;
  int cid; // mask size uncertain
  float *mask;
  // vector<float> mask_vec;
  // id for ith position in mem data of masks
  int idx;
};

// simple box declaration
struct Box {
  // left, top, right, bottom
  // left top is the origin
  float top;
  float left;
  float w;
  float h;

  // box score, only for prediction
  float score;
  // label index
  int idx;

  float xmin;
  float ymin;
  float xmax;
  float ymax;

  BoxFormat format;

  Box() {}
  Box(float a, float b, float c, float d, int format = BoxFormat::XYXY) {
    switch (format) {
    case XYXY:
      xmin = a;
      ymin = b;
      xmax = c;
      ymax = d;
      this->format = XYXY;
      break;
    case TLWH:
      top = a;
      left = b;
      w = c;
      h = d;
      this->format = TLWH;
      break;
    default:
      break;
    }
  }

  void to_tlwh() {
    // suppose we have xyxy already
    if (format == XYXY) {
      this->top = xmin;
      this->left = ymin;
      this->w = xmax - xmin;
      this->h = ymax - ymin;
      format = BOTH;
    }
  }
  void to_xyxy() {
    if (format == TLWH) {
      this->xmin = left;
      this->ymin = top;
      this->xmax = left + w;
      this->ymax = top + h;
      format = BOTH;
    }
  }

  float area() {
    switch (this->format) {
    case XYXY:
      return (this->xmax - this->xmin) * (this->ymax - this->ymin);
      break;
    case TLWH:
      return this->w * this->h;
      break;
    default:
      return (this->xmax - this->xmin) * (this->ymax - this->ymin);
      break;
    }
  }
  void print() {
    switch (this->format) {
    case XYXY:
      std::cout << "x1:" << this->xmin << ",y1:" << this->ymin
                << ",x2:" << this->xmax << ",y2:" << this->ymax
                << ",id:" << this->idx << ",score:" << this->score << std::endl;
      break;
    case TLWH:
      std::cout << "top:" << this->top << ",left:" << this->left
                << ",w:" << this->w << ",h:" << this->h << ",id:" << this->idx
                << ",score:" << this->score << std::endl;
      break;
    default:
      std::cout << "x1:" << this->xmin << ",y1:" << this->ymin
                << ",x2:" << this->xmax << ",y2:" << this->ymax
                << ",id:" << this->idx << ",score:" << this->score << std::endl;
      break;
    }
  }
};

struct LandmarkPoint {
  float x;
  float y;
  float z;
};

// detection supports multiple usages
struct Detection {
  float x1;
  float y1;
  float x2;
  float y2;
  // float objectness;
  std::vector<LandmarkPoint> landmarks;
  int idx;
  float score;
};

// #ifdef USE_OPENCV
//  human pose decalaration
struct HumanPose {
  std::vector<cv::Point2f> keypoints;
  float score;
  int pose_type;
  // indicates same pose through video series, using for pose tracking
  int pose_id = -1;
  bool is_face = false;

  HumanPose(
      const std::vector<cv::Point2f> &keypoints = std::vector<cv::Point2f>(),
      const float &score = 0)
      : keypoints(keypoints), score(score){};
  Box to_box() {
    const cv::Point2f absentKeypoint(-1.0f, -1.0f);
    // converts human pose to a bounding box
    vector<float> xss;
    vector<float> yss;
    for (auto const &p : keypoints) {
      // don't count absent points
      if (p.x == absentKeypoint.x && p.y == absentKeypoint.y) {
        continue;
      }
      xss.push_back(p.x);
      yss.push_back(p.y);
      //	  cout << "(" << p.x << "," << p.y << ")" << "\n";
    }

    auto xminmax = std::minmax_element(xss.begin(), xss.end());
    auto yminmax = std::minmax_element(yss.begin(), yss.end());
    Box b((int)*(xminmax.first), (int)*(yminmax.first), (int)*(xminmax.second),
          (int)*(yminmax.second), BoxFormat::XYXY);
    //	b.print();
    return b;
  }
};

} // namespace mjolnir
