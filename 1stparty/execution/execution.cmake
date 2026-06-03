find_package(ufoexecution QUIET)
if (NOT ufoexecution_FOUND)
  Include(FetchContent)

  FetchContent_Declare(
    ufoexecution
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufoexecution.git
    GIT_TAG        main
    GIT_PROGRESS   TRUE
  )

  FetchContent_MakeAvailable(ufoexecution)
endif()
