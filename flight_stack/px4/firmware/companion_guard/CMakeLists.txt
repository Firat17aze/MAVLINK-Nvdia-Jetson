px4_add_module(
  MODULE modules__companion_guard
  MAIN companion_guard
  SRCS
    companion_guard.cpp
  DEPENDS
    px4_work_queue
)
