build_code:
	mkdir -p build
	g++ -g -Wno-deprecated \
		*.cpp \
		-I/usr/include/pcl-1.7/ -I/usr/include/eigen3/ -I/usr/include/vtk-5.8/ \
		-lpthread \
		-lpcl_io \
		-lpcl_common \
		-lpcl_visualization \
		-lpcl_filters \
		-lpcl_sample_consensus \
		-lopencv_core \
		-lopencv_features2d \
		-lopencv_objdetect \
		-lopencv_highgui \
		-lboost_system \
		-o build/main \
		-fopenmp 
