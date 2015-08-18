#include <jni.h>
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>

//#include "motionTrackerHist.h"

using namespace std;
using namespace cv;
 

//int toGray(Mat img, Mat& gray);
//int removeBackground(Mat img, Mat& foreground, double color[3], int offset);
int toHsv(Mat img, Mat& hsv);
int prepImg(Mat rgba, Mat& rgb, int width, int height);
int prepImgGray(Mat rgba, Mat& gray, int width, int height);
int resizeImg(Mat src, Mat& dst, int width, int height);
int processDiffImg(Mat & diffImg);
int trackBiggestMovingObject(Mat & img, Mat & diffImg, Mat & trackImg, Mat & objectPos);
int trackColors(Mat img, Mat& diff, Mat colorList, int offset);
int opticalFlow(Mat& next, Mat& prvs, Mat& mFlow, Mat& mTrack);

#ifdef __cplusplus
extern "C" {
#endif




JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cTrackBiggestObj(JNIEnv*, jobject, jlong addrImg, jlong addrDiffImg, jlong addrTrackImg, jlong addrObjectPos);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cTrackBiggestObj(JNIEnv*, jobject, jlong addrImg, jlong addrDiffImg, jlong addrTrackImg, jlong addrObjectPos){

        Mat& mImg = *(Mat*)addrImg;
        Mat& mDiff = *(Mat*)addrDiffImg;
        Mat& mTrack = *(Mat*)addrTrackImg;
        Mat& mObjPos = *(Mat*)addrObjectPos;

        int conv;
        jint retVal;
        conv = trackBiggestMovingObject(mImg, mDiff, mTrack, mObjPos);
        retVal = (jint)conv;
        return retVal;

}

JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cPrepImg(JNIEnv*, jobject, jlong addrRgba, jlong addrRgbSmall, jint width, jint height);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cPrepImg(JNIEnv*, jobject, jlong addrRgba, jlong addrRgbSmall, jint width, jint height){
        Mat& mRgba = *(Mat*)addrRgba;
        Mat& rgbSmall = *(Mat*)addrRgbSmall;

        int conv;
        jint retVal;


        //conv = toGray(mRgb, mGray);

        conv = prepImg(mRgba, rgbSmall, width, height);
        retVal = (jint)conv;

        return retVal;

}

JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cPrepImgGray(JNIEnv*, jobject, jlong addrRgba, jlong addrGray, jint width, jint height);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cPrepImgGray(JNIEnv*, jobject, jlong addrRgba, jlong addrGray, jint width, jint height){
        Mat& mRgba = *(Mat*)addrRgba;
        Mat& mGray = *(Mat*)addrGray;

        int conv;
        jint retVal;


        //conv = toGray(mRgb, mGray);

        conv = prepImgGray(mRgba, mGray, width, height);
        retVal = (jint)conv;

        return retVal;

}

JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cResizeImg(JNIEnv*, jobject, jlong addrSrc, jlong addrDst, jint width, jint height);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cResizeImg(JNIEnv*, jobject, jlong addrSrc, jlong addrDst, jint width, jint height){

        Mat& mSrc = *(Mat*)addrSrc;
        Mat& mDst = *(Mat*)addrDst;

        int conv;
        jint retVal;
        conv = resizeImg(mSrc, mDst, width, height);
        retVal = (jint)conv;
        return retVal;

}
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cProcessDiffImg(JNIEnv*, jobject, jlong addrDiff);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cProcessDiffImg(JNIEnv*, jobject, jlong addrDiff){

        Mat& mDiff = *(Mat*)addrDiff;

        int conv;
        jint retVal;
        conv = processDiffImg(mDiff);
        retVal = (jint)conv;
        return retVal;
}

JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cTrackColors(JNIEnv*, jobject, jlong addrImg, jlong addrDiffImg, jlong addrColorList, jint offset);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cTrackColors(JNIEnv*, jobject, jlong addrImg, jlong addrDiffImg, jlong addrColorList, jint offset){
        Mat& mImg = *(Mat*)addrImg;
        Mat& mDiff = *(Mat*)addrDiffImg;
        Mat& mColorList = *(Mat*)addrColorList;

        int conv;
        jint retVal;

        conv = trackColors(mImg, mDiff, mColorList, (int)offset);
        retVal = (jint)conv;

        return retVal;

}

JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cOpticalFlow(JNIEnv*, jobject, jlong addrImg, jlong addrPrevImg, jlong addrFlowImg, jlong addrTrackImg);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_cOpticalFlow(JNIEnv*, jobject, jlong addrImg, jlong addrPrevImg, jlong addrFlowImg, jlong addrTrackImg){
        Mat& mImg = *(Mat*)addrImg;
        Mat& mPrevImg = *(Mat*)addrImg;
        Mat& mFlowImg = *(Mat*)addrFlowImg;
        Mat& mTrackImg = *(Mat*)addrTrackImg;
        

        int conv;
        jint retVal;

        conv = opticalFlow(mImg, mPrevImg, mFlowImg, mTrackImg);
        retVal = (jint)conv;

        return retVal;

}



/*
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_trackColor(JNIEnv*, jobject, jlong addrRgba, jlong addrGray, jlong jr, jlong jg, jlong jb, jint offset);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_trackColor(JNIEnv*, jobject, jlong addrRgba, jlong addrGray, jlong jr, jlong jg, jlong jb, jint offset){
        Mat& mRgb = *(Mat*)addrRgba;
        Mat& mGray = *(Mat*)addrGray;

        int conv;
        jint retVal;


        //conv = toGray(mRgb, mGray);
        double* color = new double[3];
        color[0] = (double)jr;
        color[1] = (double)jg;
        color[2] = (double)jb;

        conv = removeBackground(mRgb, mGray, color, (int)offset);
        retVal = (jint)conv;

        return retVal;

}

JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_convertNativeGray(JNIEnv*, jobject, jlong addrRgba, jlong addrGray);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_convertNativeGray(JNIEnv*, jobject, jlong addrRgba, jlong addrGray) {
 
    Mat& mRgb = *(Mat*)addrRgba;
    Mat& mGray = *(Mat*)addrGray;
    
    int conv;
    jint retVal;    

    conv = toGray(mRgb, mGray);
    retVal = (jint)conv;
 
    return retVal;

}*/

JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_rgb2hsv(JNIEnv*, jobject, jlong addrRgba, jlong addrHsv);
JNIEXPORT jint JNICALL Java_com_digitalwonders_ilhan_opencvproject_MainActivity_rgb2hsv(JNIEnv*, jobject, jlong addrRgba, jlong addrHsv) {

    Mat& mRgb = *(Mat*)addrRgba;
    Mat& mHsv = *(Mat*)addrHsv;

    int conv;
    jint retVal;

    conv = toHsv(mRgb, mHsv);
    retVal = (jint)conv;

    return retVal;

}
}

int resizeImg(Mat src, Mat& dst, int width, int height)
{
    resize(src, dst, Size(width, height));

    if (dst.rows == height && dst.cols == width)
    {
        return (1);
    }
    return(0);

}

int prepImg(Mat rgba, Mat& rgbSmall, int width, int height)
{
    Mat rgb;
    cvtColor(rgba, rgb, CV_RGBA2RGB); // Assuming RGBA input
    resize(rgb, rgbSmall, Size(width, height));

    //bgrBig.release();

    if (rgbSmall.rows == height && rgbSmall.cols == width)
    {
        return (height);
    }
    return(0);

}

int prepImgGray(Mat rgba, Mat& gray, int width, int height)
{
    Mat grayBig;
    cvtColor(rgba, grayBig, CV_RGBA2GRAY); // Assuming RGBA input
    resize(grayBig, gray, Size(width, height));

    //bgrBig.release();

    if (gray.rows == height && gray.cols == width)
    {
        return (height);
    }
    return(0);
}

int toHsv(Mat img, Mat& hsv)
{

    cvtColor(img, hsv, CV_BGR2HSV); // Assuming RGBA input

    if (hsv.rows == img.rows && hsv.cols == img.cols)
    {
        return (1);
    }
    return(0);
}

/*int toGray(Mat img, Mat& gray)
{
    cvtColor(img, gray, CV_RGBA2GRAY); // Assuming RGBA input
    
    if (gray.rows == img.rows && gray.cols == img.cols)
    {
        return (1);
    }
    return(0);
}


int removeBackground(Mat img, Mat& foreground, double color[3], int offset) {


    	Scalar lowerb = Scalar(color[0], color[1], color[2], 0);
    	Scalar upperb = Scalar(color[0], color[1], color[2], 255);

    	lowerb.val[0] = lowerb.val[0] - offset;
    	if(lowerb.val[0]<0)
    		lowerb.val[0] = 0;
    	upperb.val[0] = upperb.val[0] + offset;

    	lowerb.val[1] = lowerb.val[1] - offset;
    	if(lowerb.val[1]<0)
    		lowerb.val[1] = 0;
    	upperb.val[1] = upperb.val[1] + offset;
    	if(upperb.val[1]>255)
    		upperb.val[1] = 255;

    	lowerb.val[2] = lowerb.val[2] - offset;
    	if(lowerb.val[2]<0)
    		lowerb.val[2] = 0;
    	upperb.val[2] = upperb.val[2] + offset;
    	if(upperb.val[2]>255)
    		upperb.val[2] = 255;

    	//lowerb.val[2] = 0;
    	//upperb.val[2] = 255;

    	inRange(img, lowerb, upperb, foreground);
    	return (1);
}*/

int getMatWithColorMask(Mat img, Mat& colorMask,double color[3], int offset) {

    Scalar lowerb = Scalar(color[0], color[1], color[2], 0);
    Scalar upperb = Scalar(color[0], color[1], color[2], 255);

    lowerb.val[0] = lowerb.val[0] - offset;
    if(lowerb.val[0]<0)
        lowerb.val[0] = 0;
    upperb.val[0] = upperb.val[0] + offset;

    lowerb.val[1] = lowerb.val[1] - offset;
    if(lowerb.val[1]<0)
        lowerb.val[1] = 0;
    upperb.val[1] = upperb.val[1] + offset;
    if(upperb.val[1]>255)
        upperb.val[1] = 255;

    lowerb.val[2] = lowerb.val[2] - offset;
    if(lowerb.val[2]<0)
        lowerb.val[2] = 0;
    upperb.val[2] = upperb.val[2] + offset;
    if(upperb.val[2]>255)
        upperb.val[2] = 255;

    inRange(img, lowerb, upperb, colorMask);

    return 1;

}

void opticalFlowFarneback( Mat prvs, Mat next, Mat& flow) {
    
    Mat mag, angle;
    Mat hsv, rgb;
    Mat * flowCh = new Mat[2];
    Mat * hsvCh = new Mat[3];
    
    calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0 );
    //calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 7, 1.5, OPTFLOW_USE_INITIAL_FLOW );
    split(flow, flowCh);
    cartToPolar(flowCh[0], flowCh[1], mag, angle, false);
    
    normalize(angle, angle, 0, 1.0, NORM_MINMAX);
    mag.convertTo(hsvCh[2], CV_8U, 255, 0);
    angle.convertTo(hsvCh[0], CV_8U, 180);
    hsvCh[1]=Mat(hsvCh[0].rows, hsvCh[0].cols, CV_8U, Scalar(255));
    
    merge(hsvCh, 3, hsv);
    cvtColor(hsv, rgb, CV_HSV2BGR);
    
    //return hsvCh[2];
}

int opticalFlow(Mat& next, Mat& prvs, Mat& mFlow, Mat& mTrack) {
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int blurKernelSize = 5;
    int kernelSize = 5;
    int ratio = 3;
    int lowThreshold = 50;
    Scalar color = Scalar( 255, 0, 0 );
    
    //blurImg(next);
    blur( next, next, Size( blurKernelSize, blurKernelSize ), Point(-1,-1) );

    if(!prvs.empty()) {
        opticalFlowFarneback( prvs, next, mFlow);
        
        //Canny( mFlow, mFlow, lowThreshold, lowThreshold*ratio, kernelSize );
        
        //findContours( mFlow, contours, hierarchy,
          //  CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
        
        //mFlow = flow.clone();
        /// Draw contours
        mTrack = next.clone();
        for( int i = 0; i< contours.size(); i++ ) {
            if(contourArea(contours[i])>100)
                drawContours( mTrack, contours, i, color, 2, 8, hierarchy, 0, Point() );
        }
    }
    
    //prvs = next.clone();
    
    return 1;
}

int trackColors(Mat img, Mat& diff, Mat colorList, int offset)
{
    double color[3];
    Mat colorMask;

    if(colorList.rows <= 0)
        return 0;
    for(int i=0; i<colorList.rows; i++) {
        color[0] = (double)colorList.at<unsigned char>(i,0);
        color[1] = (double)colorList.at<unsigned char>(i,1);
        color[2] = (double)colorList.at<unsigned char>(i,2);

        getMatWithColorMask(img, colorMask, color, offset);

        //colorMask= getMatWithColorMask(img, color, offset);
        if(i==0)
            bitwise_or(colorMask, colorMask, diff);
        else
            bitwise_or(colorMask, diff, diff);
    }
    colorMask.release();
    return (1);
}

int processDiffImg(Mat & diffImg)
{


		Size kernelOpen(2,2);
        Size kernelClose(5,5);
        Mat element = getStructuringElement( MORPH_ELLIPSE, kernelOpen);


        erode(diffImg, diffImg, element);
        dilate(diffImg, diffImg, element);

        element = getStructuringElement( MORPH_RECT, kernelClose);
        morphologyEx( diffImg, diffImg, MORPH_CLOSE, element );

        element.release();
        //threshold(diffImg, diffImg, 1, 255, CV_THRESH_BINARY_INV);

        return (1);

}

int trackBiggestMovingObject(Mat & img, Mat & diffImg, Mat & trackImg, Mat & objPos)
{
    vector<vector<Point> > contours;
    double biggestContourArea = 0;
    double cContourArea = 0;
    Rect biggestContourRect(0,0,0,0);
    /// Detect edges using canny
    Canny( diffImg, trackImg, 100, 200, 3 );
    /// Find contours
    //findContours( trackImg, contours, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    findContours( diffImg, contours, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    //vector<Rect>::iterator it;
    //it = rectangles.begin();
    for( int i = 0; i< contours.size(); i++ )
    {
        cContourArea = contourArea(contours[i]);
        if(cContourArea<30)
            continue;
        //drawContours( frame, contours, i, 255 );

        if(cContourArea > biggestContourArea) {
            biggestContourArea = cContourArea;
            biggestContourRect = boundingRect(contours[i]);
            continue;
        }
    }
    objPos.at<unsigned char>(0,0) = (biggestContourRect.x + biggestContourRect.width/2) * 256 / diffImg.cols;
    objPos.at<unsigned char>(0,1) = (biggestContourRect.y + biggestContourRect.height/2) * 256 / diffImg.rows;

    rectangle(img, Point(biggestContourRect.x,biggestContourRect.y), Point(biggestContourRect.x+biggestContourRect.width,biggestContourRect.y+biggestContourRect.height),Scalar(255,0,0));
    if(biggestContourArea>0)
        return 1;
    else
        return 0;
}
