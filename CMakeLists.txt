cmake_minimum_required(VERSION 3.24.2)
project(Projeto2)

set(CMAKE_CXX_STANDARD 17)

add_executable(Projeto2 main.cpp Data.h Data.cpp
        Menu.h Menu.cpp DisplayMenu.h DisplayMenu.cpp DataStructures/Harvesine.h DataStructures/Harvesine.cpp)
