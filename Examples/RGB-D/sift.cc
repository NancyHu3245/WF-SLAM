
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <iostream>
#include<chrono>


using namespace std;
using namespace cv;

Mat get_sift_match(Mat im1,Mat im2);

int main()
{
    Mat seq1_1 = imread("./pat/1.png",0);
    Mat seq1_2 = imread("./pat/2.png",0);
    Mat res_seq1=get_sift_match(seq1_1,seq1_2);
    imwrite("result_tum_sift_1.bmp", res_seq1);

//     Mat seq1_1 = imread("./pat/seq1/1.ppm",0);
//     Mat seq1_2 = imread("./pat/seq1/2.ppm",0);

//     Mat seq2_1 = imread("./pat/seq2/1.ppm",0);
//     Mat seq2_2 = imread("./pat/seq2/2.ppm",0);

//     Mat seq3_1 = imread("./pat/seq3/1.ppm",0);
//     Mat seq3_2 = imread("./pat/seq3/2.ppm",0);

//     Mat seq4_1 = imread("./pat/seq4/1.ppm",0);
//     Mat seq4_2 = imread("./pat/seq4/2.ppm",0);

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t_1 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t_1 = std::chrono::monotonic_clock::now();
// #endif

//     Mat res_seq1=get_sift_match(seq1_1,seq1_2);
//     Mat res_seq2=get_sift_match(seq2_1,seq2_2);
//     Mat res_seq3=get_sift_match(seq3_1,seq3_2);
//     Mat res_seq4=get_sift_match(seq4_1,seq4_2);
// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t_2 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t_2 = std::chrono::monotonic_clock::now();
// #endif
//     double t_sift= std::chrono::duration_cast<std::chrono::duration<double> >(t_2 - t_1).count();
//     cout<<"sift time is:"<< t_sift<< endl;
//     imwrite("result_seq1_sift.bmp", res_seq1);
//     imwrite("result_seq2_sift.bmp", res_seq2);
//     imwrite("result_seq3_sift.bmp", res_seq3);
//     imwrite("result_seq4_sift.bmp", res_seq4);

    return 0;
}

Mat get_sift_match(Mat im1,Mat im2){
    Mat image1, image2;
    image1=im1.clone();
    image2=im2.clone();
    // resize(im1, image1, Size(int(im1.cols*0.15),int(im1.rows*0.15)));
    // resize(im2, image2, Size(int(im2.cols*0.15),int(im2.rows*0.15)));
    Mat image3 = image1.clone();
    Mat image4 = image2.clone();
    SiftFeatureDetector detector;
    vector<KeyPoint> image1KeyPoint, image2KeyPoint;
    detector.detect(image1, image1KeyPoint);
    detector.detect(image2, image2KeyPoint);
    Mat imageOutput1;
    Mat imageOutput2;

    drawKeypoints(image1, image1KeyPoint, imageOutput1, Scalar(0, 255, 0));
    drawKeypoints(image2, image2KeyPoint, imageOutput2, Scalar(0, 255, 0));

    SiftDescriptorExtractor extractor;
    Mat descriptor1, descriptor2;
    extractor.compute(imageOutput1, image1KeyPoint, descriptor1);
    extractor.compute(imageOutput2, image2KeyPoint, descriptor2);

    BruteForceMatcher<L2<float>> matcher;
    vector<DMatch> matches;
    matcher.match(descriptor1, descriptor2, matches);

 
    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < descriptor1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // find min_dist,max_dist
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //getgood matches
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptor1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }
    Mat img_goodmatch,img_match;
    drawMatches(image3, image1KeyPoint, image4, image2KeyPoint, good_matches, img_goodmatch, Scalar(0, 255, 0), Scalar(0, 255, 0), vector<char>(),2);
    drawMatches(image3, image1KeyPoint, image4, image2KeyPoint, matches, img_match, Scalar(0, 255, 0), Scalar(0, 255, 0), vector<char>(),2);

    imshow("image1", imageOutput1);
    imshow("image2", imageOutput2);
    imshow("image_matched", img_goodmatch);
    return img_match;
}