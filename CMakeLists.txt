cmake_minimum_required(VERSION 3.13)
project(test_cmake CXX)
# hello.cppとgood_morning.cppをコンパイルして静的ライブラリlibgreetings.aを作成
add_library(greetings STATIC hello.cpp good_morning.cpp)
# a.outという実行ファイルをmain.cppから作成
add_executable(a.out main.cpp)
# a.outを作成する際にlibgreetings.aをリンク
target_link_libraries(a.out greetings)