# FILE(GLOB_RECURSE headers *.hpp)
# FILE(GLOB_RECURSE sources *.cpp)


# add_library(WBIC SHARED ${sources} ${headers})
add_library(WBIC SHARED 
	KinWBC.cpp  
	KinWBC.hpp  
	WBIC.cpp  
	WBIC.hpp
	)

target_link_libraries(WBIC Goldfarb_Optimizer)
#target_link_libraries(WBIC JCQP)
