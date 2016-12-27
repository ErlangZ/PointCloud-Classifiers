build_code:
	mkdir -p build
	g++ -g -Wno-deprecated *.cpp \
	    --std=c++11 \
		-I/usr/include/pcl-1.7/ -I/usr/include/eigen3/ -I/usr/include/vtk-5.8/ \
		-lpcl_io \
		-lpcl_common \
		-lpcl_visualization \
		-lpcl_filters \
		-lpcl_sample_consensus \
		-lboost_system \
		-o build/main 
