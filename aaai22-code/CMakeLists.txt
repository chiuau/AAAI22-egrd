cmake_minimum_required(VERSION 3.19)
project(foctl)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

############################################################################################
# On macOS, search Homebrew for keg-only versions of Bison and Flex. Xcode does
# not provide new enough versions for us to use.
# See https://stackoverflow.com/questions/53877344/cannot-configure-cmake-to-look-for-homebrew-installed-version-of-bison

if (CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
    execute_process(
            COMMAND brew --prefix bison
            RESULT_VARIABLE BREW_BISON
            OUTPUT_VARIABLE BREW_BISON_PREFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if (BREW_BISON EQUAL 0 AND EXISTS "${BREW_BISON_PREFIX}")
        message(STATUS "Found Bison keg installed by Homebrew at ${BREW_BISON_PREFIX}")
        set(BISON_EXECUTABLE "${BREW_BISON_PREFIX}/bin/bison")
    endif()

    execute_process(
            COMMAND brew --prefix flex
            RESULT_VARIABLE BREW_FLEX
            OUTPUT_VARIABLE BREW_FLEX_PREFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if (BREW_FLEX EQUAL 0 AND EXISTS "${BREW_FLEX_PREFIX}")
        message(STATUS "Found Flex keg installed by Homebrew at ${BREW_FLEX_PREFIX}")
        set(FLEX_EXECUTABLE "${BREW_FLEX_PREFIX}/bin/flex")
    endif()
endif()
# ------------------------------------------------------------------------------------------
# find_package(Boost 1.65 REQUIRED COMPONENTS graph)
# ### find_package(Boost 1.73 REQUIRED COMPONENTS graph)
# if(Boost_FOUND)
#     include_directories(${Boost_INCLUDE_DIRS})
# endif()
# ------------------------------------------------------------------------------------------

# FIND_LIBRARY(FL_LIBRARY NAMES libfl fl HINTS /opt/homebrew/opt/flex/lib)

find_package(BISON 3.7 REQUIRED)
find_package(FLEX 2.6 REQUIRED)

BISON_TARGET(FoctlParser foctl_parser.ypp ${CMAKE_CURRENT_BINARY_DIR}/foctl_parser.tab.cpp COMPILE_FLAGS -Wcounterexamples)
FLEX_TARGET(FoctlScanner foctl_scanner.ll ${CMAKE_CURRENT_BINARY_DIR}/foctl_scanner.cpp)
ADD_FLEX_BISON_DEPENDENCY(FoctlScanner FoctlParser)

BISON_TARGET(GqsParser gqs_parser.ypp ${CMAKE_CURRENT_BINARY_DIR}/gqs_parser.tab.cpp COMPILE_FLAGS -Wcounterexamples)
FLEX_TARGET(GqsScanner gqs_scanner.ll ${CMAKE_CURRENT_BINARY_DIR}/gqs_scanner.cpp)
ADD_FLEX_BISON_DEPENDENCY(GqsScanner GqsParser)

# see https://stackoverflow.com/questions/51584960/stdcall-once-throws-stdsystem-error-unknown-error-1
find_package(Threads REQUIRED)
# target_link_libraries(foctl PRIVATE Threads::Threads)

############################################################################################

############################################################################################
# find_library(LEMON_LIBRARY NAMES libemon HINTS "$ENV{HOME}/bin/lemon/lib")
# include_directories($ENV{HOME}/bin/lemon/include)
############################################################################################

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(foctl main.cpp ${BISON_FoctlParser_OUTPUTS} ${FLEX_FoctlScanner_OUTPUTS} ${BISON_GqsParser_OUTPUTS} ${FLEX_GqsScanner_OUTPUTS} shared.h shared.cpp foctl.h foctl.cpp foctl_reader.cpp foctl_reader.h foctl_eval.h comp_tree.h environment.cpp environment.h graph.cpp graph.h name_id_map.cpp name_id_map.h goal_query.cpp goal_query.h gqs_reader.cpp gqs_reader.h string_processing.cpp string_processing.h design.h)
target_link_libraries(foctl ${Boost_LIBRARIES} Threads::Threads)
# target_link_libraries(foctl ${FLEX_LIBRARIES} ${Boost_LIBRARIES} Threads::Threads)
