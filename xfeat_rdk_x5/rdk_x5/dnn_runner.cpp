#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "hb_dnn.h"

namespace {

using Clock = std::chrono::steady_clock;

void check(int32_t code, const std::string &operation) {
  if (code != 0) {
    throw std::runtime_error(operation + " failed with code " +
                             std::to_string(code));
  }
}

std::vector<char> read_file(const std::string &path) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    throw std::runtime_error("Cannot open input: " + path);
  }
  const auto size = stream.tellg();
  if (size <= 0) {
    throw std::runtime_error("Input is empty: " + path);
  }
  std::vector<char> bytes(static_cast<size_t>(size));
  stream.seekg(0);
  stream.read(bytes.data(), size);
  if (!stream) {
    throw std::runtime_error("Cannot read complete input: " + path);
  }
  return bytes;
}

std::string shape_string(const hbDNNTensorShape &shape) {
  std::ostringstream output;
  output << '[';
  for (int32_t index = 0; index < shape.numDimensions; ++index) {
    if (index) output << ',';
    output << shape.dimensionSize[index];
  }
  output << ']';
  return output.str();
}

std::string sanitized(std::string value) {
  for (char &item : value) {
    if (!(item >= 'a' && item <= 'z') && !(item >= 'A' && item <= 'Z') &&
        !(item >= '0' && item <= '9') && item != '_' && item != '-') {
      item = '_';
    }
  }
  return value;
}

std::vector<float> valid_f32(const hbDNNTensor &tensor) {
  const auto &properties = tensor.properties;
  if (properties.tensorType != HB_DNN_TENSOR_TYPE_F32) {
    throw std::runtime_error("Only F32 output dumping is supported, tensor type=" +
                             std::to_string(properties.tensorType));
  }
  if (properties.validShape.numDimensions != 4 ||
      properties.alignedShape.numDimensions != 4) {
    throw std::runtime_error("Only 4D output dumping is supported");
  }

  const auto *source = static_cast<const float *>(tensor.sysMem[0].virAddr);
  std::vector<float> result;
  if (properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    const int n = properties.validShape.dimensionSize[0];
    const int h = properties.validShape.dimensionSize[1];
    const int w = properties.validShape.dimensionSize[2];
    const int c = properties.validShape.dimensionSize[3];
    const int ah = properties.alignedShape.dimensionSize[1];
    const int aw = properties.alignedShape.dimensionSize[2];
    const int ac = properties.alignedShape.dimensionSize[3];
    result.reserve(static_cast<size_t>(n) * h * w * c);
    for (int ni = 0; ni < n; ++ni)
      for (int hi = 0; hi < h; ++hi)
        for (int wi = 0; wi < w; ++wi)
          for (int ci = 0; ci < c; ++ci) {
            const size_t offset =
                ((static_cast<size_t>(ni) * ah + hi) * aw + wi) * ac + ci;
            result.push_back(source[offset]);
          }
  } else if (properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    const int n = properties.validShape.dimensionSize[0];
    const int c = properties.validShape.dimensionSize[1];
    const int h = properties.validShape.dimensionSize[2];
    const int w = properties.validShape.dimensionSize[3];
    const int ac = properties.alignedShape.dimensionSize[1];
    const int ah = properties.alignedShape.dimensionSize[2];
    const int aw = properties.alignedShape.dimensionSize[3];
    result.reserve(static_cast<size_t>(n) * c * h * w);
    for (int ni = 0; ni < n; ++ni)
      for (int ci = 0; ci < c; ++ci)
        for (int hi = 0; hi < h; ++hi)
          for (int wi = 0; wi < w; ++wi) {
            const size_t offset =
                ((static_cast<size_t>(ni) * ac + ci) * ah + hi) * aw + wi;
            result.push_back(source[offset]);
          }
  } else {
    throw std::runtime_error("Unsupported output layout=" +
                             std::to_string(properties.tensorLayout));
  }
  return result;
}

void write_f32(const std::string &path, const std::vector<float> &values) {
  std::ofstream stream(path, std::ios::binary);
  stream.write(reinterpret_cast<const char *>(values.data()),
               static_cast<std::streamsize>(values.size() * sizeof(float)));
  if (!stream) {
    throw std::runtime_error("Cannot write output: " + path);
  }
}

double percentile(std::vector<double> values, double fraction) {
  std::sort(values.begin(), values.end());
  const size_t index = static_cast<size_t>(fraction * (values.size() - 1));
  return values[index];
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 4 || argc > 7) {
    std::cerr << "Usage: " << argv[0]
              << " MODEL.bin INPUT.f32 OUTPUT_DIR [ITERATIONS=50] [WARMUP=10]"
                 " [BPU_CORE=0]\n";
    return 2;
  }

  hbPackedDNNHandle_t packed = nullptr;
  std::vector<hbDNNTensor> inputs;
  std::vector<hbDNNTensor> outputs;

  try {
    const std::string model_path = argv[1];
    const std::string input_path = argv[2];
    const std::string output_dir = argv[3];
    const int iterations = argc > 4 ? std::stoi(argv[4]) : 50;
    const int warmup = argc > 5 ? std::stoi(argv[5]) : 10;
    const int core = argc > 6 ? std::stoi(argv[6]) : 0;
    if (iterations <= 0 || warmup < 0 || (core != 0 && core != 1)) {
      throw std::runtime_error("Invalid iterations, warmup, or BPU core");
    }

    std::cout << "DNN_VERSION=" << hbDNNGetVersion() << '\n';
    const char *model_files[] = {model_path.c_str()};
    const auto load_start = Clock::now();
    check(hbDNNInitializeFromFiles(&packed, model_files, 1),
          "hbDNNInitializeFromFiles");
    const double load_ms = std::chrono::duration<double, std::milli>(
                               Clock::now() - load_start)
                               .count();

    const char **model_names = nullptr;
    int32_t model_count = 0;
    check(hbDNNGetModelNameList(&model_names, &model_count, packed),
          "hbDNNGetModelNameList");
    if (model_count <= 0) throw std::runtime_error("Packed model is empty");
    std::cout << "MODEL_COUNT=" << model_count << " MODEL_NAME="
              << model_names[0] << " LOAD_MS=" << std::fixed
              << std::setprecision(3) << load_ms << '\n';

    hbDNNHandle_t model = nullptr;
    check(hbDNNGetModelHandle(&model, packed, model_names[0]),
          "hbDNNGetModelHandle");

    int32_t input_count = 0;
    int32_t output_count = 0;
    check(hbDNNGetInputCount(&input_count, model), "hbDNNGetInputCount");
    check(hbDNNGetOutputCount(&output_count, model), "hbDNNGetOutputCount");
    if (input_count != 1) {
      throw std::runtime_error("Runner expects exactly one input, got " +
                               std::to_string(input_count));
    }
    inputs.resize(input_count);
    outputs.resize(output_count);

    const auto input_bytes = read_file(input_path);
    for (int32_t index = 0; index < input_count; ++index) {
      auto &tensor = inputs[index];
      check(hbDNNGetInputTensorProperties(&tensor.properties, model, index),
            "hbDNNGetInputTensorProperties");
      const char *name = nullptr;
      check(hbDNNGetInputName(&name, model, index), "hbDNNGetInputName");
      std::cout << "INPUT index=" << index << " name=" << name
                << " layout=" << tensor.properties.tensorLayout
                << " type=" << tensor.properties.tensorType
                << " valid=" << shape_string(tensor.properties.validShape)
                << " aligned=" << shape_string(tensor.properties.alignedShape)
                << " bytes=" << tensor.properties.alignedByteSize << '\n';
      if (tensor.properties.tensorType != HB_DNN_TENSOR_TYPE_F32) {
        throw std::runtime_error("Runner expects an F32 model input");
      }
      if (input_bytes.size() >
          static_cast<size_t>(tensor.properties.alignedByteSize)) {
        throw std::runtime_error("Input file is larger than aligned tensor");
      }
      check(hbSysAllocCachedMem(&tensor.sysMem[0],
                                tensor.properties.alignedByteSize),
            "hbSysAllocCachedMem(input)");
      std::memset(tensor.sysMem[0].virAddr, 0,
                  tensor.properties.alignedByteSize);
      std::memcpy(tensor.sysMem[0].virAddr, input_bytes.data(),
                  input_bytes.size());
      check(hbSysFlushMem(&tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN),
            "hbSysFlushMem(input clean)");
    }

    std::vector<std::string> output_names(output_count);
    for (int32_t index = 0; index < output_count; ++index) {
      auto &tensor = outputs[index];
      check(hbDNNGetOutputTensorProperties(&tensor.properties, model, index),
            "hbDNNGetOutputTensorProperties");
      const char *name = nullptr;
      check(hbDNNGetOutputName(&name, model, index), "hbDNNGetOutputName");
      output_names[index] = name;
      std::cout << "OUTPUT index=" << index << " name=" << name
                << " layout=" << tensor.properties.tensorLayout
                << " type=" << tensor.properties.tensorType
                << " valid=" << shape_string(tensor.properties.validShape)
                << " aligned=" << shape_string(tensor.properties.alignedShape)
                << " bytes=" << tensor.properties.alignedByteSize << '\n';
      check(hbSysAllocCachedMem(&tensor.sysMem[0],
                                tensor.properties.alignedByteSize),
            "hbSysAllocCachedMem(output)");
    }

    hbDNNInferCtrlParam control;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&control);
    control.bpuCoreId = core == 0 ? HB_BPU_CORE_0 : HB_BPU_CORE_1;

    std::vector<double> timings_ms;
    const int total = warmup + iterations;
    for (int run = 0; run < total; ++run) {
      hbDNNTaskHandle_t task = nullptr;
      hbDNNTensor *output_pointer = outputs.data();
      const auto start = Clock::now();
      check(hbDNNInfer(&task, &output_pointer, inputs.data(), model, &control),
            "hbDNNInfer");
      check(hbDNNWaitTaskDone(task, 5000), "hbDNNWaitTaskDone");
      const double elapsed = std::chrono::duration<double, std::milli>(
                                 Clock::now() - start)
                                 .count();
      check(hbDNNReleaseTask(task), "hbDNNReleaseTask");
      if (run >= warmup) timings_ms.push_back(elapsed);
    }

    const double mean_ms = std::accumulate(timings_ms.begin(), timings_ms.end(),
                                           0.0) /
                           timings_ms.size();
    std::cout << "PERF core=" << core << " warmup=" << warmup
              << " iterations=" << iterations << " mean_ms=" << mean_ms
              << " min_ms="
              << *std::min_element(timings_ms.begin(), timings_ms.end())
              << " median_ms=" << percentile(timings_ms, 0.50)
              << " p90_ms=" << percentile(timings_ms, 0.90)
              << " max_ms="
              << *std::max_element(timings_ms.begin(), timings_ms.end()) << '\n';

    std::ofstream metadata(output_dir + "/outputs.json");
    if (!metadata) throw std::runtime_error("Cannot create outputs.json");
    metadata << "{\n  \"dnn_version\": \"" << hbDNNGetVersion()
             << "\",\n  \"model_name\": \"" << model_names[0]
             << "\",\n  \"load_ms\": " << load_ms
             << ",\n  \"mean_inference_ms\": " << mean_ms
             << ",\n  \"outputs\": [\n";
    for (int32_t index = 0; index < output_count; ++index) {
      auto &tensor = outputs[index];
      check(hbSysFlushMem(&tensor.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE),
            "hbSysFlushMem(output invalidate)");
      const auto values = valid_f32(tensor);
      const std::string filename = "output_" + std::to_string(index) + "_" +
                                   sanitized(output_names[index]) + ".f32";
      write_f32(output_dir + "/" + filename, values);
      const auto minmax = std::minmax_element(values.begin(), values.end());
      const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
                          static_cast<double>(values.size());
      metadata << "    {\"index\": " << index << ", \"name\": \""
               << output_names[index] << "\", \"layout\": "
               << tensor.properties.tensorLayout << ", \"valid_shape\": \""
               << shape_string(tensor.properties.validShape)
               << "\", \"aligned_shape\": \""
               << shape_string(tensor.properties.alignedShape)
               << "\", \"file\": \"" << filename << "\", \"count\": "
               << values.size() << ", \"min\": " << *minmax.first
               << ", \"max\": " << *minmax.second << ", \"mean\": "
               << mean << "}" << (index + 1 == output_count ? "\n" : ",\n");
    }
    metadata << "  ]\n}\n";

    for (auto &tensor : inputs) check(hbSysFreeMem(&tensor.sysMem[0]), "free input");
    for (auto &tensor : outputs)
      check(hbSysFreeMem(&tensor.sysMem[0]), "free output");
    check(hbDNNRelease(packed), "hbDNNRelease");
    std::cout << "STATUS=PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "STATUS=FAIL error=" << error.what() << '\n';
    if (packed) hbDNNRelease(packed);
    return 1;
  }
}
