cmake_minimum_required(VERSION 3.5)
project(h1_2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  h1_2.urdf
  h1_2_handless.urdf
  h1_2_with_FTP_hand.urdf
  h1_2.xml
  h1_2_handless.xml
  h1_2.png
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
