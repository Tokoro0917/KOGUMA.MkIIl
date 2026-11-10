cmake_minimum_required(VERSION 3.22)

# VS Code(STM32Cube for Visual Studio Code 拡張)・コマンドラインでビルドするための設定(2026-11-10、ToraNeko.Mk3 と同じ形)。
#   cmake --preset Debug && cmake --build --preset Debug    → build/Debug/KOGUMA.MkIIl.elf
# Core/Src に .c を足したら、下の USER_SOURCES にも1行足す(CubeMX が作るファイルは cmake/stm32cubemx/CMakeLists.txt)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME KOGUMA.MkIIl)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# 自分で書いたソース(Core/Src のうち CubeMX が作らないもの)。1ファイルずつ書く
set(USER_SOURCES
    Core/Src/DistHist.c
    Core/Src/Failsafe.c
    Core/Src/LOG.c
    Core/Src/Maze.c
    Core/Src/Move.c
    Core/Src/PL_encoder.c
    Core/Src/PL_sensor.c
    Core/Src/PL_timer.c
    Core/Src/PillarEdge.c
    Core/Src/SensorNoise.c
    Core/Src/ShortParam.c
    Core/Src/SpeedPlan.c
    Core/Src/UI.c
    Core/Src/WallDistance.c
    Core/Src/Wallsensor.c
    Core/Src/lsm6dsr.c
    Core/Src/motor.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# printf で小数(%f)を出す(走行ログ・出力に使う。STM32CubeIDE の「Use float with printf」と同じ)。
# syscalls.c にない関数(_close など)は nosys.specs の空の関数を使う(STM32CubeIDE と同じ。リンクのときの「not implemented」の警告は出てよい)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float --specs=nosys.specs)

# 書き込み用の .hex・.bin も作り、大きさを出す
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
)
