cmake_minimum_required(VERSION 2.8.3)

project(sunny_gazebo)


foreach(dir config launch meshes rviz teleop urdf worlds)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
