#include "inference.hpp"
#include "tools/logger.hpp"

#include <string>
#include <utility>

// 后端按构建系统检测到的能力参与编译，与对应源文件和链接库保持一致。
#ifdef QIANLI_WITH_TENSORRT
#include "tensorrt/tensorrt_backend.hpp"
#endif

#ifdef QIANLI_WITH_OPENVINO
#include "openvino/openvino_backend.hpp"
#endif

namespace backend::inference
{

Inference::Inference(const setting::InferenceConfig& config)
: config_(config)
{
    tools::logger()->info("[inference] 开始初始化: model={}，workers={}，capacity={}",
                         config_.model_path, config_.worker, config_.capacity);

    // 只按config.type创建被选中的后端，未选中的后端不加载模型、不占用显存/线程。
    switch (config_.type)
    {
    case setting::InferBackend::TENSORRT:
#ifdef QIANLI_WITH_TENSORRT
        backend_ = std::make_unique<TensorRT>(config_);
        break;
#else
        tools::logger()->error("[inference] 当前构建未编译TensorRT后端，请使用openvino或重新构建");
        return;
#endif

    case setting::InferBackend::OPENVINO:
#ifdef QIANLI_WITH_OPENVINO
        backend_ = std::make_unique<OpenVINO>(config_);
        break;
#else
        tools::logger()->error("[inference] 当前构建未编译OpenVINO后端，请使用tensorrt或重新构建");
        return;
#endif
    }

    if (!backend_)
    {
        tools::logger()->error("[inference] 推理后端创建失败，后续任务将跳过");
        return;
    }

    tools::logger()->info("[inference] 初始化结束: backend={}",
                         config_.type == setting::InferBackend::TENSORRT ? "TensorRT" : "OpenVINO");
}

cv::Size Inference::inputSize() const
{
    return backend_ ? backend_->inputSize() : cv::Size{};
}

std::future<std::vector<float>> Inference::submit(Input input, bool async)
{
    if (!backend_)
    {
        tools::logger()->error("[inference] 推理后端未就绪，跳过本次任务");
        return emptyInferenceFuture();
    }

    auto future = async ? backend_->submitOwned(std::move(input))
                        : backend_->submitBorrowed(input);
    if (!async) future.wait();

    return future;
}

} // namespace backend::inference
