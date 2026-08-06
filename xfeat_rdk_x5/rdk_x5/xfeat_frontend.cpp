#include "xfeat_frontend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void check(int32_t code, const std::string &operation) {
  if (code != 0) {
    throw std::runtime_error(operation + " failed with code " +
                             std::to_string(code));
  }
}

float cubic_weight(float distance) {
  constexpr float alpha = -0.75F;
  distance = std::abs(distance);
  if (distance <= 1.0F) {
    return (alpha + 2.0F) * distance * distance * distance -
           (alpha + 3.0F) * distance * distance + 1.0F;
  }
  if (distance < 2.0F) {
    return alpha * distance * distance * distance -
           5.0F * alpha * distance * distance + 8.0F * alpha * distance -
           4.0F * alpha;
  }
  return 0.0F;
}

float grid_source_coordinate(float point, int full_size, int feature_size) {
  // XFeat normgrid divides by full_size - 1, then grid_sample uses
  // align_corners=false.
  return point * static_cast<float>(feature_size) /
             static_cast<float>(full_size - 1) -
         0.5F;
}

}  // namespace

XFeatBpuFrontend::XFeatBpuFrontend(const std::string &model_path, int top_k,
                                   float detection_threshold, int bpu_core,
                                   std::mutex *bpu_mutex, bool fast_decode,
                                   bool semi_dense_single, int grid_columns,
                                   int grid_rows,
                                   int extraction_grid_maximum_per_cell)
    : top_k_(top_k),
      detection_threshold_(detection_threshold),
      bpu_core_(bpu_core),
      bpu_mutex_(bpu_mutex),
      fast_decode_(fast_decode),
      semi_dense_single_(semi_dense_single),
      grid_columns_(grid_columns),
      grid_rows_(grid_rows),
      extraction_grid_maximum_per_cell_(
          extraction_grid_maximum_per_cell) {
  if (bpu_core_ != 0 && bpu_core_ != 1) {
    throw std::runtime_error("BPU core must be 0 or 1");
  }
  if (top_k_ <= 0 || grid_columns_ <= 0 || grid_rows_ <= 0 ||
      extraction_grid_maximum_per_cell_ < -1) {
    throw std::runtime_error("Invalid XFeat top-k or extraction grid configuration");
  }
  const char *files[] = {model_path.c_str()};
  check(hbDNNInitializeFromFiles(&packed_, files, 1),
        "hbDNNInitializeFromFiles");
  const char **names = nullptr;
  int32_t count = 0;
  check(hbDNNGetModelNameList(&names, &count, packed_),
        "hbDNNGetModelNameList");
  if (count != 1) {
    throw std::runtime_error("Expected one model in the packed DNN file");
  }
  model_name_ = names[0];
  check(hbDNNGetModelHandle(&model_, packed_, names[0]),
        "hbDNNGetModelHandle");
  allocate_tensors();
}

XFeatBpuFrontend::~XFeatBpuFrontend() {
  release_tensors();
  if (packed_) hbDNNRelease(packed_);
}

void XFeatBpuFrontend::allocate_tensors() {
  int32_t input_count = 0;
  int32_t output_count = 0;
  check(hbDNNGetInputCount(&input_count, model_), "hbDNNGetInputCount");
  check(hbDNNGetOutputCount(&output_count, model_), "hbDNNGetOutputCount");
  if (input_count != 1 || output_count != 3) {
    throw std::runtime_error("XFeat core must have one input and three outputs");
  }
  inputs_.resize(input_count);
  outputs_.resize(output_count);

  auto &input = inputs_[0];
  check(hbDNNGetInputTensorProperties(&input.properties, model_, 0),
        "hbDNNGetInputTensorProperties");
  if (input.properties.tensorType != HB_DNN_TENSOR_TYPE_F32 ||
      input.properties.tensorLayout != HB_DNN_LAYOUT_NCHW ||
      input.properties.validShape.numDimensions != 4 ||
      input.properties.validShape.dimensionSize[0] != 1 ||
      input.properties.validShape.dimensionSize[1] != 1) {
    throw std::runtime_error("Expected 1x1xHxW float32 NCHW XFeat input");
  }
  input_height_ = input.properties.validShape.dimensionSize[2];
  input_width_ = input.properties.validShape.dimensionSize[3];
  check(hbSysAllocCachedMem(&input.sysMem[0], input.properties.alignedByteSize),
        "hbSysAllocCachedMem(input)");

  for (int index = 0; index < output_count; ++index) {
    auto &output = outputs_[index];
    check(hbDNNGetOutputTensorProperties(&output.properties, model_, index),
          "hbDNNGetOutputTensorProperties");
    const char *name = nullptr;
    check(hbDNNGetOutputName(&name, model_, index), "hbDNNGetOutputName");
    const std::string output_name = name;
    if (output_name == "dense_descriptors") dense_index_ = index;
    if (output_name == "keypoint_logits") logits_index_ = index;
    if (output_name == "reliability") reliability_index_ = index;
    if (output.properties.tensorType != HB_DNN_TENSOR_TYPE_F32 ||
        output.properties.tensorLayout != HB_DNN_LAYOUT_NCHW ||
        output.properties.validShape.numDimensions != 4) {
      throw std::runtime_error("Expected float32 NCHW XFeat outputs");
    }
    check(hbSysAllocCachedMem(&output.sysMem[0],
                              output.properties.alignedByteSize),
          "hbSysAllocCachedMem(output)");
  }
  if (dense_index_ < 0 || logits_index_ < 0 || reliability_index_ < 0) {
    throw std::runtime_error("Cannot identify XFeat output tensors by name");
  }
  const auto &dense = outputs_[dense_index_].properties.validShape;
  feature_height_ = dense.dimensionSize[2];
  feature_width_ = dense.dimensionSize[3];
  if (dense.dimensionSize[1] != 64 || feature_height_ * 8 != input_height_ ||
      feature_width_ * 8 != input_width_) {
    throw std::runtime_error("Unexpected XFeat dense descriptor dimensions");
  }
}

void XFeatBpuFrontend::release_tensors() {
  for (auto &input : inputs_) {
    if (input.sysMem[0].virAddr) hbSysFreeMem(&input.sysMem[0]);
    input.sysMem[0] = {};
  }
  for (auto &output : outputs_) {
    if (output.sysMem[0].virAddr) hbSysFreeMem(&output.sysMem[0]);
    output.sysMem[0] = {};
  }
}

float XFeatBpuFrontend::output_value(int output_index, int channel, int y,
                                     int x) const {
  const auto &tensor = outputs_[output_index];
  const auto &shape = tensor.properties.alignedShape;
  const auto *data = static_cast<const float *>(tensor.sysMem[0].virAddr);
  const size_t offset =
      ((static_cast<size_t>(channel) * shape.dimensionSize[2] + y) *
           shape.dimensionSize[3] +
       x);
  return data[offset];
}

FeatureSet XFeatBpuFrontend::extract(const cv::Mat &mono8) {
  if (mono8.empty() || mono8.type() != CV_8UC1) {
    throw std::runtime_error("XFeat input must be a non-empty mono8 image");
  }
  FeatureSet result;
  result.image_width = mono8.cols;
  result.image_height = mono8.rows;
  const auto total_start = Clock::now();

  cv::Mat resized;
  if (mono8.cols == input_width_ && mono8.rows == input_height_) {
    resized = mono8;
  } else {
    cv::resize(mono8, resized, cv::Size(input_width_, input_height_), 0.0, 0.0,
               cv::INTER_AREA);
  }
  cv::Mat float_image;
  resized.convertTo(float_image, CV_32F);
  cv::Scalar mean;
  cv::Scalar deviation;
  cv::meanStdDev(float_image, mean, deviation);
  const float denominator =
      std::sqrt(static_cast<float>(deviation[0] * deviation[0]) + 1e-5F);
  float_image = (float_image - static_cast<float>(mean[0])) / denominator;
  if (!float_image.isContinuous()) float_image = float_image.clone();

  auto &input = inputs_[0];
  const size_t input_bytes = float_image.total() * sizeof(float);
  if (input_bytes != static_cast<size_t>(input.properties.alignedByteSize)) {
    throw std::runtime_error("XFeat input has unexpected alignment padding");
  }
  std::memcpy(input.sysMem[0].virAddr, float_image.ptr<float>(), input_bytes);
  check(hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN),
        "hbSysFlushMem(input clean)");
  const auto preprocess_end = Clock::now();

  const auto bpu_queue_start = Clock::now();
  std::unique_lock<std::mutex> bpu_lock;
  if (bpu_mutex_) bpu_lock = std::unique_lock<std::mutex>(*bpu_mutex_);
  const auto bpu_start = Clock::now();
  {
    hbDNNInferCtrlParam control;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&control);
    control.bpuCoreId = bpu_core_ == 0 ? HB_BPU_CORE_0 : HB_BPU_CORE_1;
    hbDNNTaskHandle_t task = nullptr;
    hbDNNTensor *output_pointer = outputs_.data();
    check(hbDNNInfer(&task, &output_pointer, inputs_.data(), model_, &control),
          "hbDNNInfer");
    check(hbDNNWaitTaskDone(task, 5000), "hbDNNWaitTaskDone");
    check(hbDNNReleaseTask(task), "hbDNNReleaseTask");
    for (auto &output : outputs_) {
      check(hbSysFlushMem(&output.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE),
            "hbSysFlushMem(output invalidate)");
    }
  }
  const auto bpu_end = Clock::now();
  if (bpu_lock.owns_lock()) bpu_lock.unlock();

  const int coarse_count = feature_height_ * feature_width_;
  if (semi_dense_single_) {
    struct DenseCandidate {
      int x;
      int y;
      float reliability;
    };
    const auto more_reliable = [](const DenseCandidate &left,
                                  const DenseCandidate &right) {
      if (left.reliability == right.reliability) {
        if (left.y == right.y) return left.x < right.x;
        return left.y < right.y;
      }
      return left.reliability > right.reliability;
    };
    std::vector<DenseCandidate> dense_candidates;
    dense_candidates.reserve(static_cast<size_t>(coarse_count));
    const int configured_quota = extraction_grid_maximum_per_cell_ < 0
                                     ? (top_k_ + grid_columns_ * grid_rows_ - 1) /
                                           (grid_columns_ * grid_rows_)
                                     : extraction_grid_maximum_per_cell_;
    std::vector<std::vector<DenseCandidate>> cells;
    if (configured_quota > 0) {
      cells.resize(static_cast<size_t>(grid_columns_ * grid_rows_));
    }
    for (int y = 0; y < feature_height_; ++y) {
      for (int x = 0; x < feature_width_; ++x) {
        DenseCandidate candidate{x, y,
                                 output_value(reliability_index_, 0, y, x)};
        if (configured_quota > 0) {
          const int column = std::clamp(x * 8 * grid_columns_ / input_width_, 0,
                                        grid_columns_ - 1);
          const int row = std::clamp(y * 8 * grid_rows_ / input_height_, 0,
                                     grid_rows_ - 1);
          cells[static_cast<size_t>(row * grid_columns_ + column)].push_back(
              candidate);
        } else {
          dense_candidates.push_back(candidate);
        }
      }
    }
    if (configured_quota > 0) {
      for (auto &cell : cells) {
        const size_t cell_keep = std::min(
            static_cast<size_t>(configured_quota), cell.size());
        std::partial_sort(cell.begin(), cell.begin() + cell_keep, cell.end(),
                          more_reliable);
        cell.resize(cell_keep);
      }
      for (int rank_in_cell = 0; rank_in_cell < configured_quota; ++rank_in_cell) {
        std::vector<DenseCandidate> round;
        round.reserve(cells.size());
        for (const auto &cell : cells) {
          if (static_cast<size_t>(rank_in_cell) < cell.size()) {
            round.push_back(cell[static_cast<size_t>(rank_in_cell)]);
          }
        }
        std::sort(round.begin(), round.end(), more_reliable);
        const size_t remaining = static_cast<size_t>(top_k_) -
                                 std::min(static_cast<size_t>(top_k_),
                                          dense_candidates.size());
        const size_t round_keep = std::min(remaining, round.size());
        dense_candidates.insert(dense_candidates.end(), round.begin(),
                                round.begin() + round_keep);
        if (dense_candidates.size() >= static_cast<size_t>(top_k_)) break;
      }
    }
    const size_t keep =
        std::min(static_cast<size_t>(top_k_), dense_candidates.size());
    if (configured_quota == 0) {
      std::partial_sort(dense_candidates.begin(), dense_candidates.begin() + keep,
                        dense_candidates.end(), more_reliable);
    }
    result.points.resize(keep);
    result.scores.resize(keep);
    result.scales.assign(keep, 1.0F);
    result.descriptors = cv::Mat(static_cast<int>(keep), 64, CV_32F);
    const float scale_x = static_cast<float>(mono8.cols) / input_width_;
    const float scale_y = static_cast<float>(mono8.rows) / input_height_;
    cv::parallel_for_(cv::Range(0, static_cast<int>(keep)),
                      [&](const cv::Range &range) {
      for (int item = range.start; item < range.end; ++item) {
        const auto &candidate = dense_candidates[static_cast<size_t>(item)];
        result.points[static_cast<size_t>(item)] =
            cv::Point2f(candidate.x * 8.0F * scale_x,
                        candidate.y * 8.0F * scale_y);
        result.scores[static_cast<size_t>(item)] = candidate.reliability;
        float *descriptor = result.descriptors.ptr<float>(item);
        for (int channel = 0; channel < 64; ++channel) {
          descriptor[channel] =
              output_value(dense_index_, channel, candidate.y, candidate.x);
        }
      }
    });
    const auto postprocess_end = Clock::now();
    result.timing.preprocess_ms = elapsed_ms(total_start, preprocess_end);
    result.timing.bpu_queue_ms = elapsed_ms(bpu_queue_start, bpu_start);
    result.timing.bpu_ms = elapsed_ms(bpu_start, bpu_end);
    result.timing.postprocess_ms = elapsed_ms(bpu_end, postprocess_end);
    result.timing.total_ms = elapsed_ms(total_start, postprocess_end);
    return result;
  }

  std::vector<float> normalized_dense(static_cast<size_t>(64) * coarse_count);
  cv::parallel_for_(cv::Range(0, feature_height_), [&](const cv::Range &range) {
    for (int y = range.start; y < range.end; ++y) {
      for (int x = 0; x < feature_width_; ++x) {
        float norm_squared = 0.0F;
        for (int c = 0; c < 64; ++c) {
          const float value = output_value(dense_index_, c, y, x);
          normalized_dense[(static_cast<size_t>(y) * feature_width_ + x) * 64 + c] =
              value;
          norm_squared += value * value;
        }
        const float inverse_norm =
            1.0F / std::sqrt(std::max(norm_squared, 1e-12F));
        for (int c = 0; c < 64; ++c) {
          normalized_dense[(static_cast<size_t>(y) * feature_width_ + x) * 64 + c] *=
              inverse_norm;
        }
      }
    }
  });

  struct Candidate {
    int x;
    int y;
    float score;
  };
  std::vector<Candidate> candidates;
  if (fast_decode_) {
    // One high-quality pixel proposal per 8x8 coarse cell is sufficient for
    // the 600-1024 point SLAM budget. Sorting proposals before suppression is
    // equivalent to max-pool NMS for the selected maxima and avoids materializing,
    // dilating and scanning the full 544x640 heatmap.
    std::vector<Candidate> proposals(static_cast<size_t>(coarse_count),
                                     Candidate{-1, -1, 0.0F});
    cv::parallel_for_(cv::Range(0, feature_height_), [&](const cv::Range &range) {
      for (int cy = range.start; cy < range.end; ++cy) {
        for (int cx = 0; cx < feature_width_; ++cx) {
        float maximum = -std::numeric_limits<float>::infinity();
        for (int channel = 0; channel < 65; ++channel) {
          maximum = std::max(maximum,
                             output_value(logits_index_, channel, cy, cx));
        }
        float sum = 0.0F;
        float best_probability = 0.0F;
        int best_channel = -1;
        for (int channel = 0; channel < 65; ++channel) {
          const float value =
              std::exp(output_value(logits_index_, channel, cy, cx) - maximum);
          sum += value;
          if (channel < 64 && value > best_probability) {
            best_probability = value;
            best_channel = channel;
          }
        }
        const float response = best_probability / std::max(sum, 1e-12F);
          if (best_channel < 0 || response <= detection_threshold_) continue;
          const int x = cx * 8 + best_channel % 8;
          const int y = cy * 8 + best_channel / 8;
          const float reliability = output_value(reliability_index_, 0, cy, cx);
          proposals[static_cast<size_t>(cy) * feature_width_ + cx] =
              {x, y, response * reliability};
        }
      }
    });
    candidates.reserve(static_cast<size_t>(coarse_count));
    for (const auto &proposal : proposals) {
      if (proposal.score > 0.0F) candidates.push_back(proposal);
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &a, const Candidate &b) {
                return a.score > b.score;
              });
    std::vector<uint8_t> suppressed(
        static_cast<size_t>(input_height_) * input_width_, 0);
    std::vector<Candidate> selected;
    selected.reserve(std::min(static_cast<size_t>(top_k_), candidates.size()));
    for (const auto &candidate : candidates) {
      const size_t offset = static_cast<size_t>(candidate.y) * input_width_ + candidate.x;
      if (suppressed[offset]) continue;
      selected.push_back(candidate);
      if (selected.size() >= static_cast<size_t>(top_k_)) break;
      for (int y = std::max(0, candidate.y - 2);
           y <= std::min(input_height_ - 1, candidate.y + 2); ++y) {
        for (int x = std::max(0, candidate.x - 2);
             x <= std::min(input_width_ - 1, candidate.x + 2); ++x) {
          suppressed[static_cast<size_t>(y) * input_width_ + x] = 1;
        }
      }
    }
    candidates.swap(selected);
  } else {
    std::vector<float> heatmap(static_cast<size_t>(input_height_) * input_width_);
    cv::parallel_for_(cv::Range(0, feature_height_), [&](const cv::Range &range) {
      for (int cy = range.start; cy < range.end; ++cy) {
        for (int cx = 0; cx < feature_width_; ++cx) {
          float maximum = -std::numeric_limits<float>::infinity();
          for (int c = 0; c < 65; ++c) {
            maximum = std::max(maximum, output_value(logits_index_, c, cy, cx));
          }
          float sum = 0.0F;
          float probabilities[64];
          for (int c = 0; c < 65; ++c) {
            const float value =
                std::exp(output_value(logits_index_, c, cy, cx) - maximum);
            if (c < 64) probabilities[c] = value;
            sum += value;
          }
          for (int c = 0; c < 64; ++c) {
            const int dy = c / 8;
            const int dx = c % 8;
            heatmap[static_cast<size_t>(cy * 8 + dy) * input_width_ + cx * 8 + dx] =
                probabilities[c] / sum;
          }
        }
      }
    });
    cv::Mat heatmap_image(input_height_, input_width_, CV_32F, heatmap.data());
    cv::Mat pooled_heatmap;
    cv::dilate(heatmap_image, pooled_heatmap,
               cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    candidates.reserve(4096);
    for (int y = 0; y < input_height_; ++y) {
      const float *pooled_row = pooled_heatmap.ptr<float>(y);
      for (int x = 0; x < input_width_; ++x) {
        const float response = heatmap[static_cast<size_t>(y) * input_width_ + x];
        if (response <= detection_threshold_ || response < pooled_row[x]) continue;
        const float source_x =
            grid_source_coordinate(static_cast<float>(x), input_width_, feature_width_);
        const float source_y = grid_source_coordinate(static_cast<float>(y), input_height_,
                                                       feature_height_);
        const int x0 = static_cast<int>(std::floor(source_x));
        const int y0 = static_cast<int>(std::floor(source_y));
        const float dx = source_x - x0;
        const float dy = source_y - y0;
        float reliability = 0.0F;
        for (int oy = 0; oy <= 1; ++oy) {
          for (int ox = 0; ox <= 1; ++ox) {
            const int sx = x0 + ox;
            const int sy = y0 + oy;
            if (sx < 0 || sx >= feature_width_ || sy < 0 || sy >= feature_height_)
              continue;
            const float weight_x = ox ? dx : 1.0F - dx;
            const float weight_y = oy ? dy : 1.0F - dy;
            reliability +=
                output_value(reliability_index_, 0, sy, sx) * weight_x * weight_y;
          }
        }
        candidates.push_back({x, y, response * reliability});
      }
    }
  }

  const size_t keep = std::min(static_cast<size_t>(top_k_), candidates.size());
  std::partial_sort(candidates.begin(), candidates.begin() + keep, candidates.end(),
                    [](const Candidate &a, const Candidate &b) {
                      return a.score > b.score;
                    });
  size_t actual_keep = 0;
  while (actual_keep < keep && candidates[actual_keep].score > 0.0F) ++actual_keep;
  result.points.resize(actual_keep);
  result.scores.resize(actual_keep);
  result.scales.assign(actual_keep, 1.0F);
  result.descriptors = cv::Mat(static_cast<int>(actual_keep), 64, CV_32F);
  const float scale_x = static_cast<float>(mono8.cols) / input_width_;
  const float scale_y = static_cast<float>(mono8.rows) / input_height_;
  cv::parallel_for_(cv::Range(0, static_cast<int>(actual_keep)),
                    [&](const cv::Range &range) {
    for (int item = range.start; item < range.end; ++item) {
      const auto &candidate = candidates[static_cast<size_t>(item)];
      float *descriptor = result.descriptors.ptr<float>(item);
      const float source_x = grid_source_coordinate(static_cast<float>(candidate.x),
                                                    input_width_, feature_width_);
      const float source_y = grid_source_coordinate(static_cast<float>(candidate.y),
                                                    input_height_, feature_height_);
      const int base_x = static_cast<int>(std::floor(source_x));
      const int base_y = static_cast<int>(std::floor(source_y));
      std::fill(descriptor, descriptor + 64, 0.0F);
      float x_weights[4];
      float y_weights[4];
      for (int offset = -1; offset <= 2; ++offset) {
        x_weights[offset + 1] = cubic_weight(source_x - (base_x + offset));
        y_weights[offset + 1] = cubic_weight(source_y - (base_y + offset));
      }
      for (int oy = -1; oy <= 2; ++oy) {
        const int sy = base_y + oy;
        if (sy < 0 || sy >= feature_height_) continue;
        for (int ox = -1; ox <= 2; ++ox) {
          const int sx = base_x + ox;
          if (sx < 0 || sx >= feature_width_) continue;
          const float weight = x_weights[ox + 1] * y_weights[oy + 1];
          const float *source = &normalized_dense[
              (static_cast<size_t>(sy) * feature_width_ + sx) * 64];
          for (int c = 0; c < 64; ++c) descriptor[c] += source[c] * weight;
        }
      }
      float norm_squared = 0.0F;
      for (int c = 0; c < 64; ++c) norm_squared += descriptor[c] * descriptor[c];
      const float inverse_norm =
          1.0F / std::sqrt(std::max(norm_squared, 1e-12F));
      for (int c = 0; c < 64; ++c) descriptor[c] *= inverse_norm;
      result.points[static_cast<size_t>(item)] =
          cv::Point2f(candidate.x * scale_x, candidate.y * scale_y);
      result.scores[static_cast<size_t>(item)] = candidate.score;
    }
  });

  const auto postprocess_end = Clock::now();
  result.timing.preprocess_ms = elapsed_ms(total_start, preprocess_end);
  result.timing.bpu_queue_ms = elapsed_ms(bpu_queue_start, bpu_start);
  result.timing.bpu_ms = elapsed_ms(bpu_start, bpu_end);
  result.timing.postprocess_ms = elapsed_ms(bpu_end, postprocess_end);
  result.timing.total_ms = elapsed_ms(total_start, postprocess_end);
  return result;
}

FixedFineMatcherBpu::FixedFineMatcherBpu(const std::string &model_path,
                                         int bpu_core,
                                         std::mutex *bpu_mutex)
    : bpu_core_(bpu_core), bpu_mutex_(bpu_mutex) {
  if (bpu_core_ != 0 && bpu_core_ != 1) {
    throw std::runtime_error("BPU core must be 0 or 1");
  }
  const char *files[] = {model_path.c_str()};
  check(hbDNNInitializeFromFiles(&packed_, files, 1),
        "hbDNNInitializeFromFiles(fine)");
  const char **names = nullptr;
  int32_t model_count = 0;
  check(hbDNNGetModelNameList(&names, &model_count, packed_),
        "hbDNNGetModelNameList(fine)");
  if (model_count != 1) {
    throw std::runtime_error("Expected one Fine Matcher model");
  }
  model_name_ = names[0];
  check(hbDNNGetModelHandle(&model_, packed_, names[0]),
        "hbDNNGetModelHandle(fine)");
  int32_t input_count = 0;
  int32_t output_count = 0;
  check(hbDNNGetInputCount(&input_count, model_), "hbDNNGetInputCount(fine)");
  check(hbDNNGetOutputCount(&output_count, model_),
        "hbDNNGetOutputCount(fine)");
  if (input_count != 1 || output_count != 1) {
    throw std::runtime_error("Fine Matcher must have one input and one output");
  }
  check(hbDNNGetInputTensorProperties(&input_.properties, model_, 0),
        "hbDNNGetInputTensorProperties(fine)");
  check(hbDNNGetOutputTensorProperties(&output_.properties, model_, 0),
        "hbDNNGetOutputTensorProperties(fine)");
  const auto &input_shape = input_.properties.validShape;
  const auto &output_shape = output_.properties.validShape;
  if (input_.properties.tensorType != HB_DNN_TENSOR_TYPE_F32 ||
      output_.properties.tensorType != HB_DNN_TENSOR_TYPE_F32 ||
      input_shape.numDimensions != 4 || output_shape.numDimensions != 4 ||
      input_shape.dimensionSize[0] != 1 || input_shape.dimensionSize[2] != 128 ||
      input_shape.dimensionSize[3] != 1 || output_shape.dimensionSize[0] != 1 ||
      output_shape.dimensionSize[1] != input_shape.dimensionSize[1] ||
      output_shape.dimensionSize[2] != 64 || output_shape.dimensionSize[3] != 1) {
    throw std::runtime_error(
        "Expected Fine Matcher tensors [1,M,128,1] -> [1,M,64,1]");
  }
  capacity_ = input_shape.dimensionSize[1];
  check(hbSysAllocCachedMem(&input_.sysMem[0],
                            input_.properties.alignedByteSize),
        "hbSysAllocCachedMem(fine input)");
  check(hbSysAllocCachedMem(&output_.sysMem[0],
                            output_.properties.alignedByteSize),
        "hbSysAllocCachedMem(fine output)");
}

FixedFineMatcherBpu::~FixedFineMatcherBpu() {
  release_tensors();
  if (packed_) hbDNNRelease(packed_);
}

void FixedFineMatcherBpu::release_tensors() {
  if (input_.sysMem[0].virAddr) hbSysFreeMem(&input_.sysMem[0]);
  if (output_.sysMem[0].virAddr) hbSysFreeMem(&output_.sysMem[0]);
  input_.sysMem[0] = {};
  output_.sysMem[0] = {};
}

float FixedFineMatcherBpu::output_value(int match, int bin) const {
  const auto &shape = output_.properties.alignedShape;
  const auto *values = static_cast<const float *>(output_.sysMem[0].virAddr);
  const size_t offset =
      (static_cast<size_t>(match) * shape.dimensionSize[2] + bin) *
      shape.dimensionSize[3];
  return values[offset];
}

FineMatchResult FixedFineMatcherBpu::refine(
    const FeatureSet &query, const FeatureSet &train,
    const std::vector<cv::DMatch> &coarse_matches, float minimum_confidence,
    bool refine_y, int image_width, int image_height) {
  FineMatchResult result;
  const auto total_start = Clock::now();
  if (query.descriptors.type() != CV_32F || query.descriptors.cols != 64 ||
      train.descriptors.type() != CV_32F || train.descriptors.cols != 64) {
    throw std::runtime_error("Fine Matcher requires CV_32F x 64 descriptors");
  }
  const size_t valid_count =
      std::min(coarse_matches.size(), static_cast<size_t>(capacity_));
  auto *input_values = static_cast<float *>(input_.sysMem[0].virAddr);
  std::memset(input_values, 0, input_.properties.alignedByteSize);
  for (size_t index = 0; index < valid_count; ++index) {
    const auto &match = coarse_matches[index];
    if (match.queryIdx < 0 || match.queryIdx >= query.descriptors.rows ||
        match.trainIdx < 0 || match.trainIdx >= train.descriptors.rows) {
      throw std::runtime_error("Fine Matcher received an invalid match index");
    }
    // Official XFeat concatenates the coordinate-to-refine first, followed by
    // the fixed/reference descriptor: [right, left].
    float *destination = input_values + index * 128;
    std::memcpy(destination, train.descriptors.ptr<float>(match.trainIdx),
                64 * sizeof(float));
    std::memcpy(destination + 64, query.descriptors.ptr<float>(match.queryIdx),
                64 * sizeof(float));
  }
  check(hbSysFlushMem(&input_.sysMem[0], HB_SYS_MEM_CACHE_CLEAN),
        "hbSysFlushMem(fine input clean)");
  const auto preprocess_end = Clock::now();

  const auto queue_start = Clock::now();
  std::unique_lock<std::mutex> bpu_lock;
  if (bpu_mutex_) bpu_lock = std::unique_lock<std::mutex>(*bpu_mutex_);
  const auto bpu_start = Clock::now();
  hbDNNInferCtrlParam control;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&control);
  control.bpuCoreId = bpu_core_ == 0 ? HB_BPU_CORE_0 : HB_BPU_CORE_1;
  hbDNNTaskHandle_t task = nullptr;
  hbDNNTensor *output_pointer = &output_;
  check(hbDNNInfer(&task, &output_pointer, &input_, model_, &control),
        "hbDNNInfer(fine)");
  check(hbDNNWaitTaskDone(task, 5000), "hbDNNWaitTaskDone(fine)");
  check(hbDNNReleaseTask(task), "hbDNNReleaseTask(fine)");
  check(hbSysFlushMem(&output_.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE),
        "hbSysFlushMem(fine output invalidate)");
  const auto bpu_end = Clock::now();
  if (bpu_lock.owns_lock()) bpu_lock.unlock();

  result.matches.reserve(valid_count);
  result.refined_train_points.reserve(valid_count);
  result.confidences.reserve(valid_count);
  for (size_t index = 0; index < valid_count; ++index) {
    float maximum = -std::numeric_limits<float>::infinity();
    for (int bin = 0; bin < 64; ++bin) {
      maximum = std::max(maximum, 3.0F * output_value(index, bin));
    }
    float denominator = 0.0F;
    float offset_x = 0.0F;
    float offset_y = 0.0F;
    float maximum_probability_numerator = 0.0F;
    for (int bin = 0; bin < 64; ++bin) {
      const float probability_numerator =
          std::exp(3.0F * output_value(index, bin) - maximum);
      denominator += probability_numerator;
      maximum_probability_numerator =
          std::max(maximum_probability_numerator, probability_numerator);
      offset_x += probability_numerator * static_cast<float>(bin % 8 - 4);
      offset_y += probability_numerator * static_cast<float>(bin / 8 - 4);
    }
    const float confidence = maximum_probability_numerator / denominator;
    if (!std::isfinite(confidence) || confidence < minimum_confidence) continue;
    const auto &match = coarse_matches[index];
    cv::Point2f point = train.points[match.trainIdx];
    const float scale = static_cast<size_t>(match.trainIdx) < train.scales.size()
                            ? train.scales[static_cast<size_t>(match.trainIdx)]
                            : 1.0F;
    point.x += scale * offset_x / denominator;
    if (refine_y) point.y += scale * offset_y / denominator;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0F ||
        point.x >= image_width || point.y < 0.0F || point.y >= image_height) {
      continue;
    }
    result.matches.push_back(match);
    result.refined_train_points.push_back(point);
    result.confidences.push_back(confidence);
  }
  const auto postprocess_end = Clock::now();
  result.timing.preprocess_ms = elapsed_ms(total_start, preprocess_end);
  result.timing.bpu_queue_ms = elapsed_ms(queue_start, bpu_start);
  result.timing.bpu_ms = elapsed_ms(bpu_start, bpu_end);
  result.timing.postprocess_ms = elapsed_ms(bpu_end, postprocess_end);
  result.timing.total_ms = elapsed_ms(total_start, postprocess_end);
  return result;
}

std::vector<cv::DMatch> match_xfeat(const FeatureSet &a, const FeatureSet &b,
                                    float minimum_cosine, double *latency_ms) {
  const auto start = Clock::now();
  std::vector<cv::DMatch> matches;
  if (a.descriptors.empty() || b.descriptors.empty()) {
    if (latency_ms) *latency_ms = 0.0;
    return matches;
  }
  // Descriptors are unit-normalized, so minimizing L2 is exactly equivalent
  // to maximizing cosine similarity: ||a-b||^2 = 2 - 2*cos(a,b).
  // OpenCV's BFMatcher uses optimized vector kernels and crossCheck implements
  // the same mutual-nearest-neighbour rule as XFeat's reference matcher.
  cv::BFMatcher matcher(cv::NORM_L2, true);
  matcher.match(a.descriptors, b.descriptors, matches);
  const float maximum_l2 = std::sqrt(2.0F - 2.0F * minimum_cosine);
  matches.erase(std::remove_if(matches.begin(), matches.end(),
                               [maximum_l2](const cv::DMatch &match) {
                                 return match.distance >= maximum_l2;
                               }),
                matches.end());
  for (auto &match : matches) {
    const float cosine = 1.0F - 0.5F * match.distance * match.distance;
    match.distance = 1.0F - cosine;
  }
  std::sort(matches.begin(), matches.end(),
            [](const cv::DMatch &left, const cv::DMatch &right) {
              return left.distance < right.distance;
            });
  if (latency_ms) *latency_ms = elapsed_ms(start, Clock::now());
  return matches;
}

namespace {

struct GuidedCandidate {
  cv::DMatch match;
  float ranking = 0.0F;
};

float descriptor_cosine(const cv::Mat &a, int row_a, const cv::Mat &b,
                        int row_b) {
  const float *left = a.ptr<float>(row_a);
  const float *right = b.ptr<float>(row_b);
  float value = 0.0F;
  for (int channel = 0; channel < 64; ++channel) value += left[channel] * right[channel];
  return value;
}

std::vector<cv::DMatch> guided_match(
    const FeatureSet &a, const FeatureSet &b, float minimum_cosine,
    float ratio_threshold, float vertical_tolerance, float horizontal_radius,
    bool rectified_stereo, size_t maximum_matches, double *latency_ms,
    int grid_columns, int grid_rows, size_t grid_maximum_per_cell) {
  const auto start = Clock::now();
  std::vector<cv::DMatch> result;
  if (a.descriptors.empty() || b.descriptors.empty() || a.points.empty() ||
      b.points.empty() || maximum_matches == 0) {
    if (latency_ms) *latency_ms = elapsed_ms(start, Clock::now());
    return result;
  }
  if (a.descriptors.type() != CV_32F || b.descriptors.type() != CV_32F ||
      a.descriptors.cols != 64 || b.descriptors.cols != 64) {
    throw std::runtime_error("Guided XFeat matching requires CV_32F x 64 descriptors");
  }
  if (grid_columns <= 0 || grid_rows <= 0) {
    throw std::runtime_error("Guided XFeat matching requires a positive grid size");
  }
  if (grid_maximum_per_cell > 0 &&
      (a.image_width <= 0 || a.image_height <= 0)) {
    throw std::runtime_error(
        "Grid-quota matching requires the query image dimensions");
  }

  float maximum_y = 0.0F;
  for (const auto &point : a.points) maximum_y = std::max(maximum_y, point.y);
  for (const auto &point : b.points) maximum_y = std::max(maximum_y, point.y);
  const int row_count = std::max(1, static_cast<int>(std::ceil(maximum_y)) + 2);
  std::vector<std::vector<int>> row_indices(static_cast<size_t>(row_count));
  for (size_t index = 0; index < b.points.size(); ++index) {
    const int row = std::clamp(static_cast<int>(std::lround(b.points[index].y)), 0,
                               row_count - 1);
    row_indices[static_cast<size_t>(row)].push_back(static_cast<int>(index));
  }

  std::vector<int> best_train(a.points.size(), -1);
  std::vector<float> best_similarity(a.points.size(), -2.0F);
  std::vector<float> second_similarity(a.points.size(), -2.0F);
  std::vector<int> reverse_best_query(b.points.size(), -1);
  std::vector<float> reverse_best_similarity(b.points.size(), -2.0F);

  for (size_t query = 0; query < a.points.size(); ++query) {
    const cv::Point2f &point_a = a.points[query];
    const int minimum_row = std::max(
        0, static_cast<int>(std::floor(point_a.y - vertical_tolerance)));
    const int maximum_row = std::min(
        row_count - 1, static_cast<int>(std::ceil(point_a.y + vertical_tolerance)));
    for (int row = minimum_row; row <= maximum_row; ++row) {
      for (const int train : row_indices[static_cast<size_t>(row)]) {
        const cv::Point2f &point_b = b.points[static_cast<size_t>(train)];
        if (std::abs(point_a.y - point_b.y) > vertical_tolerance) continue;
        const float delta_x = point_a.x - point_b.x;
        if (rectified_stereo) {
          if (delta_x < 0.0F || delta_x > horizontal_radius) continue;
        } else if (std::abs(delta_x) > horizontal_radius) {
          continue;
        }
        const float cosine = descriptor_cosine(a.descriptors, static_cast<int>(query),
                                               b.descriptors, train);
        if (cosine > best_similarity[query]) {
          second_similarity[query] = best_similarity[query];
          best_similarity[query] = cosine;
          best_train[query] = train;
        } else if (cosine > second_similarity[query]) {
          second_similarity[query] = cosine;
        }
        if (cosine > reverse_best_similarity[static_cast<size_t>(train)]) {
          reverse_best_similarity[static_cast<size_t>(train)] = cosine;
          reverse_best_query[static_cast<size_t>(train)] = static_cast<int>(query);
        }
      }
    }
  }

  std::vector<GuidedCandidate> candidates;
  candidates.reserve(std::min(a.points.size(), b.points.size()));
  for (size_t query = 0; query < a.points.size(); ++query) {
    const int train = best_train[query];
    if (train < 0 || best_similarity[query] < minimum_cosine ||
        reverse_best_query[static_cast<size_t>(train)] != static_cast<int>(query))
      continue;
    const float best_distance = 1.0F - best_similarity[query];
    const float second_distance = 1.0F - second_similarity[query];
    if (second_similarity[query] > -1.0F &&
        best_distance >= ratio_threshold * second_distance)
      continue;
    const float score_a = query < a.scores.size() ? a.scores[query] : 1.0F;
    const float score_b = static_cast<size_t>(train) < b.scores.size()
                              ? b.scores[static_cast<size_t>(train)]
                              : 1.0F;
    candidates.push_back({cv::DMatch(static_cast<int>(query), train, best_distance),
                          best_similarity[query] * score_a * score_b});
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const GuidedCandidate &left, const GuidedCandidate &right) {
              if (left.ranking == right.ranking) {
                if (left.match.distance == right.match.distance) {
                  if (left.match.queryIdx == right.match.queryIdx)
                    return left.match.trainIdx < right.match.trainIdx;
                  return left.match.queryIdx < right.match.queryIdx;
                }
                return left.match.distance < right.match.distance;
              }
              return left.ranking > right.ranking;
            });
  result.reserve(std::min(candidates.size(), maximum_matches));
  if (grid_maximum_per_cell == 0) {
    for (const auto &candidate : candidates) {
      result.push_back(candidate.match);
      if (result.size() == maximum_matches) break;
    }
  } else {
    std::vector<std::vector<const GuidedCandidate *>> cells(
        static_cast<size_t>(grid_columns * grid_rows));
    for (const auto &candidate : candidates) {
      const cv::Point2f &point =
          a.points[static_cast<size_t>(candidate.match.queryIdx)];
      const int column = std::clamp(
          static_cast<int>(point.x * grid_columns / a.image_width), 0,
          grid_columns - 1);
      const int row = std::clamp(
          static_cast<int>(point.y * grid_rows / a.image_height), 0,
          grid_rows - 1);
      const size_t cell = static_cast<size_t>(row * grid_columns + column);
      if (cells[cell].size() < grid_maximum_per_cell) {
        cells[cell].push_back(&candidate);
      }
    }
    for (size_t rank_in_cell = 0; rank_in_cell < grid_maximum_per_cell;
         ++rank_in_cell) {
      std::vector<const GuidedCandidate *> round;
      round.reserve(cells.size());
      for (const auto &cell : cells) {
        if (rank_in_cell < cell.size()) round.push_back(cell[rank_in_cell]);
      }
      std::sort(round.begin(), round.end(),
                [](const GuidedCandidate *left, const GuidedCandidate *right) {
                  if (left->ranking == right->ranking) {
                    if (left->match.distance == right->match.distance) {
                      if (left->match.queryIdx == right->match.queryIdx)
                        return left->match.trainIdx < right->match.trainIdx;
                      return left->match.queryIdx < right->match.queryIdx;
                    }
                    return left->match.distance < right->match.distance;
                  }
                  return left->ranking > right->ranking;
                });
      for (const auto *candidate : round) {
        result.push_back(candidate->match);
        if (result.size() == maximum_matches) break;
      }
      if (result.size() == maximum_matches) break;
    }
  }
  if (latency_ms) *latency_ms = elapsed_ms(start, Clock::now());
  return result;
}

}  // namespace

std::vector<cv::DMatch> match_xfeat_stereo_guided(
    const FeatureSet &left, const FeatureSet &right, float minimum_cosine,
    float ratio_threshold, float vertical_tolerance, float maximum_disparity,
    size_t maximum_matches, double *latency_ms, int grid_columns, int grid_rows,
    size_t grid_maximum_per_cell) {
  return guided_match(left, right, minimum_cosine, ratio_threshold,
                      vertical_tolerance, maximum_disparity, true,
                      maximum_matches, latency_ms, grid_columns, grid_rows,
                      grid_maximum_per_cell);
}

std::vector<cv::DMatch> match_xfeat_temporal_guided(
    const FeatureSet &reference, const FeatureSet &current, float minimum_cosine,
    float ratio_threshold, float search_radius, size_t maximum_matches,
    double *latency_ms, int grid_columns, int grid_rows,
    size_t grid_maximum_per_cell) {
  return guided_match(reference, current, minimum_cosine, ratio_threshold,
                      search_radius, search_radius, false, maximum_matches,
                      latency_ms, grid_columns, grid_rows,
                      grid_maximum_per_cell);
}
