#include <opencv2/opencv.hpp>
#include <string>

#include "app.hpp"

const std::string keys =
  "{help h usage ? | | print this message}"
  "{@config-path   | | yaml config file path }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  return app::run(config_path);
}
