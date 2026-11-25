#include "pipeline_setting.hpp"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"

namespace backend::setting
{

InferenceConfig::InferenceConfig(const YAML::Node& config)
{
    // 模型路径
    model_path = tools::read<std::string>(config, "model_path", model_path);

    // 推理后端
    const auto backend = tools::read<std::string>(config, "type", "");
    if (backend == "tensorrt")
    {
        type = InferBackend::TENSORRT;
        const auto node = config["tensorrt"];
        tensorrt.device_id = tools::read<int>(node, "device_id", tensorrt.device_id);
    }

    else if (backend == "openvino")
    {
        type = InferBackend::OPENVINO;
        const auto node = config["openvino"];
        openvino.device = tools::read<std::string>(node, "device", openvino.device);
    }

    else
    {
        tools::logger()->error("[settings] 推理后端必须是tensorrt或者openvino之一，当前为{}，保留默认值", backend);
    }

    // 同步或异步
    const auto value = tools::read<std::string>(config, "mode", "sync");
    if (value == "sync")
        mode = InferMode::SYNC;

    else if (value == "async")
        mode = InferMode::ASYNC;

    else
        tools::logger()->error("[settings] 执行方式必须为同步或异步中的一种，当前为{}，保留默认值", value);

    // 队列配置
    capacity = tools::read<int>(config, "capacity", capacity);
    worker = tools::read<size_t>(config, "worker", worker);
}

PipelineConfig::PipelineConfig(const YAML::Node& config)
    : processing(config["processing"]),
      inference(config["inference"])
{
}

PipelineConfig loadPipelineConfig(const std::string& path)
{
    tools::logger()->info("[settings] 加载配置: {}", path);

    const auto root = tools::load(path);
    PipelineConfig config(root["pipeline"]);

    tools::logger()->info("[settings] 配置加载成功: {}", path);

    return config;
}

} // namespace backend::setting
