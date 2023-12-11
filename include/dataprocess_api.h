/*
 * @Author: zjj
 * @Date: 2023-12-06 16:42:19
 * @LastEditors: zjj
 * @LastEditTime: 2023-12-08 11:47:03
 * @FilePath: /DataProcess/include/dataprocess_api.h
 * @Description:
 *
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */
#pragma once

#include "data_process.h"

namespace ParkingPerception
{
namespace DataProcess
{
ImageProcess* CreateImageProcess(std::string config_file);
}  // namespace DataProcess
}  // namespace ParkingPerception