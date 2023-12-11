<!--
 * @Author: zjj
 * @Date: 2023-12-06 10:29:59
 * @LastEditors: zjj
 * @LastEditTime: 2023-12-11 17:17:04
 * @FilePath: /DataProcess/README.md
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
-->
# Use of ImageProcess

## 1 Requirement
This codebase has been developed with OpenCV-4.6.0 ...

## 2 Installation
### 2.1 Create docker container
`docker run --gpus all -it -v /mnt/Data/Dataset:/hostdata/Dataset -v /mnt/work/projects:/hostdata/projects -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE --net=host --privileged=true --name=perception_20231204 ubuntu18.04_cu11.1_trt8.4:20231204 /bin/bash`

### 2.2 Install modules *ImageProcess*
```
cd ImageProcess
bash build.sh
```

## 3 Test
### 3.1 Run test
`
bash run_test.sh
`
### 3.2 Results Visualization
<div align=left><img src="./test/ll_seg_filter.jpg" width=300 height=300>
<div align=left><img src="./test/da_seg_filter.jpg" width=300 height=300>


