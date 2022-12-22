#include "mjolnir/simpleocv.h"
#include "mjolnir/type.h"
#include "mjolnir/vis.h"

using mjolnir::Detection;

int main(int argc, char **argv) {

  std::string img_f = argv[1];

  cv::Mat a = cv::imread(img_f);
  cv::putText(a, "28.9 C from SimpleOCV", cv::Point(20, 45), 1, 0.5,
              cv::Scalar(255, 0, 255));
  cv::imwrite("a_gray.png", a);

  // draw rectangle
  std::vector<Detection> dets;
  Detection d1;
  d1.x1 = 24;
  d1.y1 = 110;
  d1.x2 = 340;
  d1.y2 = 400;
  d1.idx = 2;
  d1.score = 0.9;
  dets.push_back(d1);

  auto res = mjolnir::vis::VisualizeDetections(
      a, dets, mjolnir::dl::COCO_CLASSES_NO_BK, nullptr, 2);
  cv::imwrite("a_det.png", res);

  cv::Mat b;
  cv::resize(a, b, cv::Size(416, 416));
  cv::imwrite("a_resized.png", b);
}