[1mdiff --git a/CMakeLists.txt b/CMakeLists.txt[m
[1mindex ddd2356..120923b 100644[m
[1m--- a/CMakeLists.txt[m
[1m+++ b/CMakeLists.txt[m
[36m@@ -60,9 +60,9 @@[m [mfind_package(catkin REQUIRED COMPONENTS[m
    imuRawData.msg[m
    cameraExposureCtrl.msg[m
    keyDataOrParam.msg[m
[31m-   panoramicCapture.msg[m
[31m-   panoramicStatus.msg[m
[31m-   imageExtraInfo.msg[m
[32m+[m[32m   wheelSpeed.msg[m
[32m+[m[32m   RCFrame.msg[m
[32m+[m[32m   batInfoBase.msg[m
  )[m
 [m
 ## Generate services in the 'srv' folder[m
