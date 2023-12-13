#include "dataprocess_api.h"

namespace ParkingPerception
{
namespace DataProcess
{
std::shared_ptr<ImageProcess> CreateImageProcess(std::string config_file)
{
  return std::make_shared<ImageProcess>(config_file);
}
}  // namespace DataProcess
}  // namespace ParkingPerception