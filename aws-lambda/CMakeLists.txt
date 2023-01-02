cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
project(hello LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wextra -Wpedantic -Wall -O3")
set(CMAKE_CXX_STANDARD 17)
include_directories(/home/harshyadav/vroom/src)

find_package(aws-lambda-runtime REQUIRED)
add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(${PROJECT_NAME} PUBLIC AWS::aws-lambda-runtime)
target_link_libraries(${PROJECT_NAME} PUBLIC /home/harshyadav/vroom/lib/libvroom.a -lpthread -lssl -lcrypto -lglpk)
aws_lambda_package_target(${PROJECT_NAME})