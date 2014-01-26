#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils.hpp"
#include <cmath>

using namespace cv;

void readme();

/** @function main */
int main( int argc, char** argv )
{
  if( argc != 3 )
  { readme(); return -1; }

  Mat img_object = imread( argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_scene = imread( argv[2], CV_LOAD_IMAGE_COLOR);

  // std::cout<< "Second" <<std::endl;
  // drawPerspeciveTransform(img_object, img_scene, img_matches, HAccurate, "Second");

  ResultMat result = show(img_object, img_scene, std::string("First"), true);
  ResultMat theEnd = show(img_object, result.image, std::string("Second"), false);

  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0);
  obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
  obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);
  std::vector<Point2f> final_corners(4);

  perspectiveTransform(obj_corners, scene_corners, result.homo);
  perspectiveTransform(scene_corners, final_corners, theEnd.homo);

  for(int i = 0; i < 4; i++) {
    std::cout<< "object corner" << obj_corners[i] << std::endl;
    std::cout<< "scene corner" << scene_corners[i] << std::endl;
    std::cout<< "final corner" << final_corners[i] << std::endl;
  }


  Mat warp_dst = Mat::zeros( img_object.rows, img_object.cols, img_object.type());

  Point2f dstTri[4], srcTri[4];

  /// Set your 3 points to calculate the  Affine Transform
  dstTri[0] = Point2f( 0, 0);
  dstTri[1] = Point2f( img_object.cols, 0);
  dstTri[2] = Point2f( img_object.cols, img_object.rows);
  dstTri[3] = Point2f( 0, img_object.rows);

  srcTri[0] = final_corners[0];
  srcTri[1] = final_corners[1];
  srcTri[2] = final_corners[2]; 
  srcTri[3] = final_corners[3]; 

  Mat warp_mat;

  /// Get the Affine Transform
  warp_mat = getPerspectiveTransform(dstTri, srcTri);
  std::cout << "perspetcive" << warp_mat.size() << std::endl;

  /// Apply the Affine Transform just found to the src image
  warpPerspective(img_scene, warp_dst, warp_mat, img_scene.size() );

  imwrite("theend.jpg", warp_dst);

  waitKey(0);
  return 0;
}

  /** @function readme */
  void readme()
  { std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl; }
