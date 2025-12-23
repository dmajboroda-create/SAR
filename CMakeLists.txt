cmake_minimum_required(VERSION 3.14)
project(EngineSimulation VERSION 1.0 LANGUAGES CXX)

# Встановлення стандарту C++
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Опції для покриття коду (code coverage)
option(ENABLE_COVERAGE "Enable code coverage" OFF)

if(ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
        message(STATUS "Code coverage enabled")
    endif()
endif()

# ==========================================
# ОСНОВНА ПРОГРАМА
# ==========================================

# Виконуваний файл основної програми
add_executable(engine_simulation
    Main.cpp
)

# ==========================================
# НАЛАШТУВАННЯ GOOGLE TEST
# ==========================================

# Включення тестування
enable_testing()

# Завантаження Google Test через FetchContent
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0  # Остання стабільна версія
)

# Для Windows: запобігання перезапису runtime бібліотек
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Завантаження та компіляція Google Test
FetchContent_MakeAvailable(googletest)

# ==========================================
# ТЕСТОВА ПРОГРАМА
# ==========================================

# Виконуваний файл тестів
add_executable(engine_tests
    test_engine.cpp
)

# Підключення Google Test до тестів
target_link_libraries(engine_tests
    gtest
    gtest_main
)

# Включення автоматичного виявлення тестів
include(GoogleTest)
gtest_discover_tests(engine_tests)

# ==========================================
# ДОДАТКОВІ НАЛАШТУВАННЯ
# ==========================================

# Вивід інформації про конфігурацію
message(STATUS "===========================================")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Coverage Enabled: ${ENABLE_COVERAGE}")
message(STATUS "===========================================")

# Копіювання CSV файлу з результатами (якщо існує)
if(EXISTS "${CMAKE_SOURCE_DIR}/simulation_results.csv")
    configure_file(
        "${CMAKE_SOURCE_DIR}/simulation_results.csv"
        "${CMAKE_BINARY_DIR}/simulation_results.csv"
        COPYONLY
    )
endif()
