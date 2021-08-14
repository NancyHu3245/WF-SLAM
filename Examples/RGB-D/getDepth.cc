// #include <opencv2/opencv.hpp> 
// #include <opencv2/contrib/contrib.hpp>
// #include <opencv2/calib3d/calib3d.hpp> 
// #include <opencv2/legacy/legacy.hpp> 
// #include "time.h"

// using namespace cv;

// int main()
// {
//     IplImage* img_r, *img_l;               
//     img_r = cvLoadImage("./1imR.png", 0);   
//     img_l = cvLoadImage("./1imL.png", 0);   

//     cvShowImage("左边图像", img_l);
//     cvShowImage("右边图像", img_r);

//     CvMat* norm_disparity = cvCreateMat(img_l->height, img_l->width, CV_8U);
//     long time = clock();

//     //BMËã·¨ 
//     //CvMat* disparity = cvCreateMat(img_l->height, img_l->width, CV_32FC1);
//     //CvStereoBMState* BMState = cvCreateStereoBMState(); 
//     //BMState->SADWindowSize = 17;//ËÑË÷´°¿Ú´óÐ¡,Ï¸Äå³Ì¶È¸úÕâ¸öÊýÖµÓÐ¹Ø
//     //BMState->minDisparity = 0;//´ú±íÆ¥ÅäËÑËÕ´ÓÄÄÀï¿ªÊ¼
//     //BMState->numberOfDisparities = 16;//±íÊ¾×î´óËÑË÷ÊÓ²îÊý£¬Ô­À´Îª16
//     //BMState->uniquenessRatio = 25;
//     //cvFindStereoCorrespondenceBM(img_l, img_r, disparity, BMState); //Ð£ÕýÍ¼Ïñ
//     //cvNormalize(disparity, norm_disparity, 45, 160, CV_MINMAX, NULL); //Í¼Ïñ¹éÒ»»¯
//     //cvReleaseMat(&disparity);

//     //GC算法
//     CvMat* disparity_left = cvCreateMat(img_l->height, img_l->width, CV_16S);
//     CvMat* disparity_right = cvCreateMat(img_l->height, img_l->width, CV_16S);
//     CvStereoGCState* state = cvCreateStereoGCState(16, 2);
//     cvFindStereoCorrespondenceGC(img_l,img_r,disparity_left,disparity_right,state,0);
//     cvReleaseStereoGCState(&state);
//     cvConvertScale(disparity_left, norm_disparity, -16);
//     cvReleaseMat(&disparity_left);
//     cvReleaseMat(&disparity_right);

//     //给深度图上伪颜色
//     Mat tempMat = Mat(norm_disparity, true);
//     Mat img_pseudocolor(tempMat.rows, tempMat.cols, CV_8UC3);//构造ＲＧＢ图
//     int tmp = 0;
//     for (int y = 0; y<tempMat.rows; y++)//具体实现
//     {
//         for (int x = 0; x<tempMat.cols; x++)
//         {
//             tmp = tempMat.at<unsigned char>(y, x);
//             img_pseudocolor.at<Vec3b>(y, x)[0] = abs(255 - tmp); //blue 
//             img_pseudocolor.at<Vec3b>(y, x)[1] = abs(127 - tmp); //green 
//             img_pseudocolor.at<Vec3b>(y, x)[2] = abs(0 - tmp); //red 
//         }
//     }
//     printf("图像分辨率:%d*%d\n", img_l->width,img_l->height);
//     printf("双目深度图计算消耗的时间:%dms\n", clock() - time);
//     imshow("结果", img_pseudocolor);//显示图像
//     cvWaitKey(0);

//     //释放内存，关闭窗口
//     cvDestroyAllWindows();
//     cvReleaseImage(&img_l);
//     cvReleaseImage(&img_r);
//     cvReleaseMat(&norm_disparity);
// }


#include <cvaux.h>
#include <highgui.h>
#include <cv.h>
#include <cxcore.h>
#include <iostream>


using namespace std;
using namespace cv;
int main()
{

    IplImage * img1 = cvLoadImage("./1imL.png", 0);
    IplImage * img2 = cvLoadImage("./1imR.png", 0);
    cv::StereoSGBM sgbm;
    int SADWindowSize = 11;
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
    int cn = img1->nChannels;
    int numberOfDisparities = 144;
    sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 300;
    sgbm.speckleRange = 10;
    sgbm.disp12MaxDiff = 1;
    Mat disp, disp8;
    int64 t = getTickCount();
    sgbm((Mat)img1, (Mat)img2, disp);
    t = getTickCount() - t;
    cout << "Time elapsed:" << t * 1 / getTickFrequency() << endl;
    disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

    Mat img3,img4,img5;
    threshold(disp8, img3, 160, 255, THRESH_BINARY);
    threshold(disp8, img4, 80, 255, THRESH_BINARY);
    threshold(disp8, img5, 0, 255, THRESH_BINARY);
    
    img5 = img5 - img4;
    img4 = img4 - img3;
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
   // morphologyEx(disp8, disp8, MORPH_CLOSE, element);

   /* cvNamedWindow("1", 0);
    imshow("1", img3);
    cvNamedWindow("2", 0);
    imshow("2", img4);
    cvNamedWindow("3", 0);
    imshow("3", img5);*/
    namedWindow("left", 1);
    cvShowImage("left", img1);
    namedWindow("right", 1);
    cvShowImage("right", img2);
    namedWindow("disparity", 1);
    imshow("disparity", disp8);
    imwrite("sgbm_disparity.png", disp8);
    waitKey();
    cvDestroyAllWindows();
    return 0;
}