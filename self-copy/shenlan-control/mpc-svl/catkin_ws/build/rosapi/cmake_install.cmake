# Install script for directory: /home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/hua/shenlan-control/mpc/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/hua/shenlan-control/mpc/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/hua/shenlan-control/mpc/catkin_ws/install" TYPE PROGRAM FILES "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/hua/shenlan-control/mpc/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/hua/shenlan-control/mpc/catkin_ws/install" TYPE PROGRAM FILES "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/hua/shenlan-control/mpc/catkin_ws/install/setup.bash;/home/hua/shenlan-control/mpc/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/hua/shenlan-control/mpc/catkin_ws/install" TYPE FILE FILES
    "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/setup.bash"
    "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/hua/shenlan-control/mpc/catkin_ws/install/setup.sh;/home/hua/shenlan-control/mpc/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/hua/shenlan-control/mpc/catkin_ws/install" TYPE FILE FILES
    "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/setup.sh"
    "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/hua/shenlan-control/mpc/catkin_ws/install/setup.zsh;/home/hua/shenlan-control/mpc/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/hua/shenlan-control/mpc/catkin_ws/install" TYPE FILE FILES
    "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/setup.zsh"
    "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/hua/shenlan-control/mpc/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/hua/shenlan-control/mpc/catkin_ws/install" TYPE FILE FILES "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/msg" TYPE FILE FILES "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/msg/TypeDef.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/srv" TYPE FILE FILES
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/DeleteParam.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/GetActionServers.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/GetParam.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/GetParamNames.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/GetTime.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/HasParam.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/MessageDetails.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/Nodes.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/NodeDetails.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/Publishers.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/SearchParam.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/ServiceHost.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/ServiceNode.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/ServiceProviders.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/ServiceRequestDetails.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/ServiceResponseDetails.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/Services.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/ServicesForType.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/ServiceType.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/SetParam.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/Subscribers.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/Topics.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/TopicsForType.srv"
    "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/srv/TopicType.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/cmake" TYPE FILE FILES "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/rosapi-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/hua/shenlan-control/mpc/catkin_ws/devel/.private/rosapi/include/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/hua/shenlan-control/mpc/catkin_ws/devel/.private/rosapi/share/roseus/ros/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/hua/shenlan-control/mpc/catkin_ws/devel/.private/rosapi/share/common-lisp/ros/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/hua/shenlan-control/mpc/catkin_ws/devel/.private/rosapi/share/gennodejs/ros/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/hua/shenlan-control/mpc/catkin_ws/devel/.private/rosapi/lib/python3/dist-packages/rosapi")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/hua/shenlan-control/mpc/catkin_ws/devel/.private/rosapi/lib/python3/dist-packages/rosapi" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/hua/shenlan-control/mpc/catkin_ws/devel/.private/rosapi/lib/python3/dist-packages/rosapi" FILES_MATCHING REGEX "/home/hua/shenlan-control/mpc/catkin_ws/devel/\\.private/rosapi/lib/python3/dist-packages/rosapi/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/rosapi.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/cmake" TYPE FILE FILES "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/rosapi-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi/cmake" TYPE FILE FILES
    "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/rosapiConfig.cmake"
    "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/catkin_generated/installspace/rosapiConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosapi" TYPE FILE FILES "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rosapi" TYPE PROGRAM FILES "/home/hua/shenlan-control/mpc/catkin_ws/src/rosbridge_suite/rosapi/scripts/rosapi_node")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hua/shenlan-control/mpc/catkin_ws/build/rosapi/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
