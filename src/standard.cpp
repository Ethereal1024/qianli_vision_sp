#include <opencv2/opencv.hpp>
#include <string>

#include "app.hpp"

const std::string keys =
  "{help h usage ? |      | print this message}"
  "{@config-path   | configs/standard3.yaml | yaml config file path }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  return app::run(config_path);
}
