#include "dataprocess_api.h"

namespace ParkingPerception
{
namespace DataProcess
{
ImageProcess* CreateImageProcess(std::string config_file)
{
  return new ImageProcess(config_file);
}
}  // namespace DataProcess
}  // namespace ParkingPerception