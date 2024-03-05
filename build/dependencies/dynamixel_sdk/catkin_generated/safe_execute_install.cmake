execute_process(COMMAND "/home/sj/Desktop/VR_ARM/build/dependencies/dynamixel_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sj/Desktop/VR_ARM/build/dependencies/dynamixel_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
