DEPS = -lopencv_core -lopencv_flann -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lopencv_calib3d -lopencv_nonfree `pkg-config --cflags --libs opencv`
feature:
	g++ featurepoints.cpp -o feature $(DEPS) --std=c++0x
stitching:
	g++ stitch.cpp -o stitch $(DEPS)
clean:
	rm feature stitch
