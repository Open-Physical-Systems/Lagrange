cmake_minimum_required(VERSION 3.22)
project(lagrange_example_rl VERSION 1.5.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch robot policies DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/train.py DESTINATION lib/${PROJECT_NAME} RENAME train)
install(PROGRAMS scripts/ppo.py DESTINATION lib/${PROJECT_NAME} RENAME ppo)
install(
    PROGRAMS scripts/fly_policy.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME fly_policy
)
ament_package()
