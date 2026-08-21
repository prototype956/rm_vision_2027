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

VisionFrameResult VisionPipeline::Process(const frame::FramePacket& packet) {
  VisionFrameResult result;
  const auto& image = packet.capture.image;
  result.detections = detector_.Detect(image);
  result.detector_stats = detector_.LastStats();

  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  result.refinements.reserve(result.detections.size());
  for (const auto& detection : result.detections) {
    result.refinements.push_back(corner_refiner_.Refine(gray_image, detection.corners));
  }
  result.lightbars =
      light_detector_.Detect(image, gray_image, result.detections, result.refinements);
  const auto SPATIAL = frame::MakeSpatialFrameView(packet);
  const auto* simulation_data = packet.simulation ? &*packet.simulation : nullptr;
  if (SPATIAL) {
    result.pnp =
        pnp_.ProcessFrame(packet.capture.stamp.sequence, *packet.camera_model, *packet.kinematics,
                          simulation_data, result.detections, result.refinements);
  }
  result.prediction =
      predictor_.ProcessFrame(packet.capture.stamp, SPATIAL, simulation_data, result.detections,
                              result.refinements, result.pnp, result.lightbars);
  return result;
}

}  // namespace mv::runtime
