#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

using namespace cv;
using std::string;

float cosineDistance(Point2f source, Point2f dst1, Point2f dst2) {
  // a.b / ||a|| ||b||
  Point2f A(dst1.x - source.x, dst1.y - source.y);
  Point2f B(dst2.x - source.x, dst2.y - source.y);
  return 1.0 - (A.x * B.x + A.y * B.y ) / (sqrt(A.x*A.x + A.y*A.y)
    * sqrt(B.x*B.x + B.y*B.y));
}

struct ResultMat {
  Mat image;
  Mat homo;
};

Mat findAccurateHomography(std::vector<Point2f> obj, std::vector<Point2f> scene) {
  // find the homography
  Mat H = findHomography( obj, scene, CV_RANSAC );

  std::vector<Point2f> newScene(obj.size());

  // apply the homography onto the original vectors
  perspectiveTransform( obj, newScene, H);
  std::vector<float> cosineErrors;

  std::vector<Point2f> finalSrcs;
  std::vector<Point2f> finalDests;

  // calculate error 
  /*
  for(int i = 0; i < newScene.size(); i++) {
    cosineErrors.push_back(cosineDistance(obj[i], scene[i], newScene[i]));
  }

  // ignore all the ones with greater than 0.1 error
  for(int i = 0; i < newScene.size(); i++) {
    if(cosineErrors[i] < 0.1) { 
      finalSrcs.push_back(obj[i]);
      finalDests.push_back(scene[i]);
    }
  }

  H = findHomography( finalSrcs, finalDests, CV_RANSAC );
  */
  return H;
}

void getKeyPoints(Mat img, std::vector<KeyPoint>& keypoints) {
  SiftFeatureDetector detector;
  detector.detect( img, keypoints);
}

void getGoodMatches(
  std::vector<KeyPoint> keypoints_object,
  std::vector<KeyPoint> keypoints_scene,
  Mat descriptors_object, 
  Mat descriptors_scene, 
  std::vector<Point2f>& obj, 
  std::vector<Point2f>& scene,
  std::vector<DMatch>& good_matches
) {

  // FlannBasedMatcher matcher;
  BFMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );
  
  //float factor = 3.0; 
  float factor = 5;
  while(good_matches.size() < 4) {
    good_matches.clear();
    for( int i = 0; i < descriptors_object.rows; i++ )
    { if( matches[i].distance < max(factor*min_dist, 0.02))
       { good_matches.push_back( matches[i]); }
    }
    factor *= 1.2;
  }
}

ResultMat drawPerspeciveTransform(Mat img_object, Mat img_scene, Mat img_matches, Mat H, const char* title) {

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 2 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 2 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 2 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 2 );

  //-- Show detected matches
  imwrite( title + std::string("GoodMatchesObjectdetection.jpg"), img_matches );

  ////////////////////////
  // Section for perspective trans
  Mat warp_dst = Mat::zeros( img_object.rows, img_object.cols, img_object.type());

   Point2f dstTri[4], srcTri[4];

   /// Set your 3 points to calculate the  Affine Transform
   dstTri[0] = Point2f( 0, 0);
   dstTri[1] = Point2f( img_object.cols, 0);
   dstTri[2] = Point2f( img_object.cols, img_object.rows);
   dstTri[3] = Point2f( 0, img_object.rows);

   srcTri[0] = scene_corners[0];
   srcTri[1] = scene_corners[1];
   srcTri[2] = scene_corners[2]; 
   srcTri[3] = scene_corners[3]; 

   Mat warp_mat;

   /// Get the Affine Transform
   warp_mat = getPerspectiveTransform(srcTri, dstTri );
   std::cout << "perspetcive" << warp_mat.size() << std::endl;

   /// Apply the Affine Transform just found to the src image
   warpPerspective( img_scene, warp_dst, warp_mat, img_scene.size() );

  imwrite(title + string(".jpg"), warp_dst);
  ResultMat result;
  result.image = warp_dst;
  result.homo = warp_mat;
  return result;
}

ResultMat drawHalfPerspeciveTransform(Mat img_object, Mat img_scene, Mat img_matches, Mat H, const char* title) {

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 2 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 2 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 2 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 2 );

  //-- Show detected matches
  imwrite( title + std::string("GoodMatchesObjectdetection.jpg") , img_matches );

  ////////////////////////
  // Section for perspective trans
  Mat warp_dst = Mat::zeros( img_object.rows, img_object.cols, img_object.type());

   Point2f dstTri[4], srcTri[4];

   /// Set your 3 points to calculate the  Affine Transform
   dstTri[0] = Point2f( 0, 0);
   dstTri[1] = Point2f( img_object.cols, 0);
   dstTri[2] = Point2f( img_object.cols, img_object.rows);
   dstTri[3] = Point2f( 0, img_object.rows);

   srcTri[0] = Point2f(
    (scene_corners[0].x + dstTri[0].x)/2, 
    (scene_corners[0].y + dstTri[0].y)/2
   );
   srcTri[1] = Point2f(
    (scene_corners[1].x + dstTri[1].x)/2, 
    (scene_corners[1].y + dstTri[1].y)/2
   );
   srcTri[2] = Point2f(
    (scene_corners[2].x + dstTri[2].x)/2, 
    (scene_corners[2].y + dstTri[2].y)/2
   );
   srcTri[3] = Point2f(
    (scene_corners[3].x + dstTri[3].x)/2, 
    (scene_corners[3].y + dstTri[3].y)/2
   );

   Mat warp_mat;

   /// Get the Affine Transform
   warp_mat = getPerspectiveTransform(srcTri, dstTri );

   /// Apply the Affine Transform just found to the src image
   warpPerspective( img_scene, warp_dst, warp_mat, img_scene.size() );

  imwrite(title + string(".jpg"), warp_dst);
  ResultMat result;
  result.image = warp_dst;
  result.homo = warp_mat;
  return result;
}

ResultMat show(Mat img_object, Mat img_scene, std::string title, bool half) {
  if( !img_object.data || !img_scene.data )
  { std::cout<< " --(!) Error reading images " << std::endl; exit(1); }

  //-- Step 1: Detect the keypoints using sift Detector
  std::vector<KeyPoint> keypoints_object, keypoints_scene;
  getKeyPoints(img_object, keypoints_object);
  getKeyPoints( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  SiftDescriptorExtractor extractor;

  Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;
  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  
  getGoodMatches(keypoints_object, keypoints_scene, descriptors_object, descriptors_scene, obj, scene, good_matches);

  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  for( int i = 0; i < good_matches.size(); i++ ) {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }


  Mat H = findHomography( obj, scene, CV_RANSAC);
  Mat HAccurate = findAccurateHomography( obj, scene);
  Mat result;
  
  if(half) {
    return drawHalfPerspeciveTransform(img_object, img_scene, img_matches, H, title.c_str());
  } else {
    return drawPerspeciveTransform(img_object, img_scene, img_matches, H, title.c_str());
  }
}
