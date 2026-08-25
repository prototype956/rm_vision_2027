#include "runtime/vision_pipeline.hpp"

#include <opencv2/imgproc.hpp>

namespace mv::runtime {

VisionPipeline::VisionPipeline(const VisionPipelineConfig& config)
    : pnp_(config.pnp),
      predictor_(config.predictor),
      corner_refiner_(config.corner_refiner),
      light_detector_(config.light_detector, config.detector.enemy_color) {
  detector_.Init(config.detector);
}

VisionFrameResult VisionPipeline::Process(const VisionFrameInput& input) {
  VisionFrameResult result;
  const auto& image = input.capture.image;
  auto detector_result = detector_.Detect(image);
  result.output.detections = std::move(detector_result.output.detections);
  result.diagnostics.detector = detector_result.diagnostics;

  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  result.output.refinements.reserve(result.output.detections.size());
  result.diagnostics.refinements.reserve(result.output.detections.size());
  for (const auto& detection : result.output.detections) {
    auto refinement = corner_refiner_.Refine(gray_image, detection.corners);
    result.output.refinements.push_back(refinement.output);
    result.diagnostics.refinements.push_back(std::move(refinement.diagnostics));
  }
  auto lightbars = light_detector_.Detect(image, gray_image, result.output.detections,
                                          result.output.refinements);
  result.output.lightbars = std::move(lightbars.output);
  result.diagnostics.lightbars = std::move(lightbars.diagnostics);
  pnp_.ObserveRefinementDiagnostics(result.diagnostics.refinements);
  if (input.spatial) {
    auto pnp = pnp_.ProcessFrame(input.capture.stamp.sequence, input.spatial->calibration,
                                 result.output.detections, result.output.refinements);
    result.output.pnp = std::move(pnp.output);
    result.diagnostics.pnp = std::move(pnp.diagnostics);
  }
  auto prediction = predictor_.ProcessFrame(input.capture.stamp, input.spatial,
                                            result.output.detections, result.output.refinements,
                                            result.output.pnp, result.output.lightbars);
  result.output.prediction = std::move(prediction.output);
  result.diagnostics.prediction = std::move(prediction.diagnostics);
  return result;
}

}  // namespace mv::runtime
