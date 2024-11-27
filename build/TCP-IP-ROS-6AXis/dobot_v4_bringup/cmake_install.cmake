# Install script for directory: /home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/roboert/MP_WS/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_v4_bringup/msg" TYPE FILE FILES
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/msg/RobotStatus.msg"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/msg/ToolVectorActual.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_v4_bringup/srv" TYPE FILE FILES
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/EnableRobot.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/DisableRobot.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ClearError.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SpeedFactor.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/User.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/Tool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/RobotMode.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetPayload.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/DO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/DOInstant.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ToolDO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ToolDOInstant.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/AO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/AOInstant.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/AccJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/AccL.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/VelJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/VelL.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/CP.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/PowerOn.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/RunScript.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/Stop.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/Pause.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/Continue.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/PositiveKin.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/InverseKin.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetCollisionLevel.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetAngle.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetPose.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/EmergencyStop.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ModbusRTUCreate.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ModbusCreate.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ModbusClose.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetInBits.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetInRegs.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetCoils.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetCoils.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetHoldRegs.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetHoldRegs.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetSafeSkin.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/MovJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/MovL.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/RelJointMovJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/MoveJog.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/StopMoveJog.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/DOGroup.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/BrakeControl.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/StartDrag.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/EnableSafeSkin.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetStartPose.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/StartPath.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/InverseSolution.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetErrorID.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/DI.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ToolDI.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/AI.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ToolAI.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/DIGroup.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/StopDrag.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/DragSensivity.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetDO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetAO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetDOGroup.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetTool485.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetSafeWallEnable.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetToolPower.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetToolMode.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetBackDistance.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetPostCollisionMode.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetUser.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetTool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/CalcUser.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/CalcTool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetInputBool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetInputInt.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetInputFloat.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetOutputBool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetOutputInt.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetOutputFloat.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetOutputBool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetOutputInt.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/SetOutputFloat.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/MovLIO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/MovJIO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/Arc.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/Circle.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/RelMovJTool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/RelMovLTool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/RelMovJUser.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/RelMovLUser.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/GetCurrentCommandId.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ServoJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/ServoP.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/srv/TcpDashboard.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_v4_bringup/cmake" TYPE FILE FILES "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_v4_bringup/catkin_generated/installspace/dobot_v4_bringup-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/include/dobot_v4_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/share/roseus/ros/dobot_v4_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/share/common-lisp/ros/dobot_v4_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/share/gennodejs/ros/dobot_v4_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/roboert/anaconda3/bin/python3" -m compileall "/home/roboert/MP_WS/devel/lib/python3/dist-packages/dobot_v4_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/lib/python3/dist-packages/dobot_v4_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_v4_bringup/catkin_generated/installspace/dobot_v4_bringup.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_v4_bringup/cmake" TYPE FILE FILES "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_v4_bringup/catkin_generated/installspace/dobot_v4_bringup-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_v4_bringup/cmake" TYPE FILE FILES
    "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_v4_bringup/catkin_generated/installspace/dobot_v4_bringupConfig.cmake"
    "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_v4_bringup/catkin_generated/installspace/dobot_v4_bringupConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_v4_bringup" TYPE FILE FILES "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_v4_bringup/package.xml")
endif()

