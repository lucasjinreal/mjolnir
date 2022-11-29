
#pragma once
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core.hpp>

#include "./file_io.h"

using std::string;
using std::vector;

namespace mjolnir {
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

namespace iter {

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
  if (thor::os::isfile(source)) {
    // judge if it's video or image file
    if (thor::os::suffix(source) == "mp4" ||
        thor::os::suffix(source) == "avi") {
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
  } else if (thor::os::isdir(source)) {
    this->mode = IterMode::DIR;
    this->item_str_pool = thor::os::list_files(source, true);
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
      return;
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

} // namespace iter

} // namespace mjolnir
