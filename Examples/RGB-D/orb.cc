#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <iostream>
#include<chrono>

using namespace std;
using namespace cv;

Mat get_match(Mat img_1,Mat img_2);

int main () 
{
    Mat seq1_1 = imread("./pat/1.png",0);
    Mat seq1_2 = imread("./pat/2.png",0);
    Mat res_seq1=get_match(seq1_1,seq1_2);
    imwrite("result_tum_orb_1.bmp", res_seq1);

//     // Mat seq1_1 = imread("./zed/seq1/1.jpg",0);
//     // Mat seq1_2 = imread("./zed/seq1/5.jpg",0);

//     // Mat seq2_1 = imread("./zed/seq2/1.jpg",0);
//     // Mat seq2_2 = imread("./zed/seq2/3.jpg",0);

//     // Mat seq3_1 = imread("./zed/seq3/1.jpg",0);
//     // Mat seq3_2 = imread("./zed/seq3/4.jpg",0);

//     // Mat seq4_1 = imread("./zed/seq4/1.jpg",0);
//     // Mat seq4_2 = imread("./zed/seq4/3.jpg",0);
    
//     Mat seq1_1 = imread("./pat/seq1/1.ppm",0);
//     Mat seq1_2 = imread("./pat/seq1/2.ppm",0);

//     Mat seq2_1 = imread("./pat/seq2/1.ppm",0);
//     Mat seq2_2 = imread("./pat/seq2/2.ppm",0);

//     Mat seq3_1 = imread("./pat/seq3/1.ppm",0);
//     Mat seq3_2 = imread("./pat/seq3/2.ppm",0);

//     Mat seq4_1 = imread("./pat/seq4/1.ppm",0);
//     Mat seq4_2 = imread("./pat/seq4/2.ppm",0);

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
// #endif

//     Mat res_seq1=get_match(seq1_1,seq1_2);
//     Mat res_seq2=get_match(seq2_1,seq2_2);
//     Mat res_seq3=get_match(seq3_1,seq3_2);
//     Mat res_seq4=get_match(seq4_1,seq4_2);

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
// #endif
//     double t_orb= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//     cout<<"orb time is:"<< t_orb<< endl;
    
//     imwrite("result_seq1_orb.bmp", res_seq1);
//     imwrite("result_seq2_orb.bmp", res_seq2);
//     imwrite("result_seq3_orb.bmp", res_seq3);
//     imwrite("result_seq4_orb.bmp", res_seq4);
   
    return 0;
}

Mat get_match(Mat img_1,Mat img_2){
    Mat image1, image2;
    image1=img_1.clone();
    image2=img_2.clone();
    // resize(img_1, image1, Size(int(img_1.cols*0.15),int(img_1.rows*0.15)));
    // resize(img_2, image2, Size(int(img_2.cols*0.15),int(img_2.rows*0.15)));
    Mat image3 = image1.clone();
    Mat image4 = image2.clone();

    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;

    Ptr<FeatureDetector> detector = ORB::create("ORB");
    Ptr<DescriptorExtractor> descriptor = ORB::create("ORB");
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    detector->detect ( image1,keypoints_1 );
    detector->detect ( image2,keypoints_2 );
    descriptor->compute ( image1, keypoints_1, descriptors_1 );
    descriptor->compute ( image2, keypoints_2, descriptors_2 );

    Mat outimg1;
    drawKeypoints( image1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("ORBfeature",outimg1);

    //BruteForce macher
    vector<DMatch> matches;
    matcher->match ( descriptors_1, descriptors_2, matches );

    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < descriptors_1.rows; i++ )
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
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //draw result
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( image3, keypoints_1, image4, keypoints_2, matches, img_match, Scalar(0, 255, 0), Scalar(0, 255, 0),vector<char>(),2);
    drawMatches ( image3, keypoints_1, image4, keypoints_2, good_matches, img_goodmatch ,Scalar(0, 255, 0), Scalar(0, 255, 0),vector<char>(),2);
    imshow ( "img_match", img_match );
    imwrite("im_match.png",img_match);
    imshow ( "img_goodmatch", img_goodmatch );
    return img_goodmatch;
}