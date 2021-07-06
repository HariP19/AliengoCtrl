#add_library
#targer_include_directory(common/include)
#target_library(common)
#target_library(fsmstates)
#thirdparty

# include_directories(${PROJECT_SOURCE_DIR}/common/include/)

add_library(convexMPC SHARED
    convexMPC_interface.cpp  
	convexMPC_interface.h  
	ConvexMPCLocomotion.cpp  
	ConvexMPCLocomotion.h  
	Gait.cpp  
	Gait.h  
    convexMPC_util.h 
    convexMPC_util.cpp   
	SolverMPC.cpp  
    SolverMPC.h 
    RobotState.h 
    RobotState.cpp)


target_link_libraries(convexMPC PUBLIC 
	common 
	qpOASES 
	JCQP #CHECK
	)
target_include_directories(convexMPC PUBLIC
	${PROJECT_SOURCE_DIR}/MIT_Controller/third-party/
	${PROJECT_SOURCE_DIR}/MIT_Controller/third-party/qpOASES/include
	${PROJECT_SOURCE_DIR}/MIT_Controller/common/include/
	)