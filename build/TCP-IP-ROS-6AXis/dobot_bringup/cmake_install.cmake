# Install script for directory: /home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup

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
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_bringup/msg" TYPE FILE FILES
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/msg/RobotStatus.msg"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/msg/ToolVectorActual.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_bringup/srv" TYPE FILE FILES
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/EnableRobot.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/DisableRobot.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ClearError.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ResetRobot.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SpeedFactor.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/User.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/Tool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/RobotMode.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/PayLoad.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/DO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/DOExecute.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ToolDO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ToolDOExecute.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/AO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/AOExecute.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/AccJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/AccL.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SpeedJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SpeedL.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/Arc.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/Arch.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/CP.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/LimZ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetArmOrientation.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/PowerOn.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/RunScript.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/StopScript.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/PauseScript.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ContinueScript.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetHoldRegs.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetHoldRegs.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetSafeSkin.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetObstacleAvoid.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/PositiveSolution.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetCollisionLevel.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/EmergencyStop.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/BrakeControl.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/Continues.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/DI.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ToolDI.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/AI.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ToolAI.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/DigitalOutputs.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetAngle.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetCoils.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetErrorID.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/StopmoveJog.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/StopDrag.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/StartDrag.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetPayload.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetCoils.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetArmOrientation.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/RelMovJTool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/RelMovLTool.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/RelMovLUser.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/RelMovJUser.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/InverseSolution.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetPose.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetInRegs.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetInBits.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/DIGroup.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/DOGroup.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/pause.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/User.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/Wait.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/MovJExt.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ModbusCreate.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ModbusClose.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/LoadSwitch.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/RelJointMovJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetTraceStartPose.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetPathStartPose.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/HandleTrajPoints.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetSixForceData.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetCollideDrag.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetTerminalKeys.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SetTerminal485.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/GetTerminal485.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/TCPSpeed.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/TCPSpeedEnd.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/JointMovJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/MovJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/MovL.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/Jump.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/RelMovJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/RelMovL.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/MovLIO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/MovJIO.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/Arc.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/Circle.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ServoJ.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ServoP.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/ServoJParam.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/Sync.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/SyncAll.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/StartTrace.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/StartPath.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/StartFCTrace.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/MoveJog.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/TcpDashboard.srv"
    "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/srv/TcpRealData.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_bringup/cmake" TYPE FILE FILES "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_bringup/catkin_generated/installspace/dobot_bringup-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/include/dobot_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/share/roseus/ros/dobot_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/share/common-lisp/ros/dobot_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/share/gennodejs/ros/dobot_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/roboert/anaconda3/bin/python3" -m compileall "/home/roboert/MP_WS/devel/lib/python3/dist-packages/dobot_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/roboert/MP_WS/devel/lib/python3/dist-packages/dobot_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_bringup/catkin_generated/installspace/dobot_bringup.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_bringup/cmake" TYPE FILE FILES "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_bringup/catkin_generated/installspace/dobot_bringup-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_bringup/cmake" TYPE FILE FILES
    "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_bringup/catkin_generated/installspace/dobot_bringupConfig.cmake"
    "/home/roboert/MP_WS/build/TCP-IP-ROS-6AXis/dobot_bringup/catkin_generated/installspace/dobot_bringupConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot_bringup" TYPE FILE FILES "/home/roboert/MP_WS/src/TCP-IP-ROS-6AXis/dobot_bringup/package.xml")
endif()

