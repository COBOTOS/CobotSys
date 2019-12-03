#include <boost/shared_ptr.hpp>
#include <logger/GlogConfig.h>
#include <logger/Logger.h>
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <VisionInputImageStruct.h>
#include <VisionFactoryInterface.h>
#include <VisionInterface.h>
#include <VisionGoodsInfoStruct.h>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>
#include <VisionInputImageStruct.h>
#include <grasp/DetectorBaseSeg.h>

using namespace EAGLE_CAMERA3D_NS_API;
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_VISION_NS_API;

bool computeSingleMask(const cv::Mat &mask, const int &label, cv::Mat &singleMask)
{
    cv::compare(mask, label, singleMask, cv::CMP_EQ);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::morphologyEx(singleMask, singleMask, cv::MORPH_OPEN, element);
}

bool getMask(const cv::Mat &mask, std::vector<cv::Mat> &masks)
{
    LOG_INFO << "mask size: " << mask.rows << " ," << mask.cols;

    if (mask.empty()) {
        LOG_INFO << "ERROcdR:Det DetectorBaseSeg::getMask:gmask.empty()";
        return false;
    }

    //将mask图像分解为每个图像的轮廓
    double min, max;
    cv::minMaxLoc(mask, &min, &max);
    if (max < 1) {
        LOG_INFO << "mask中无有效标记";
        return false;
    }
    //生成各个物体的mask
    for (int i = 1; i <= max; i++) {
        cv::Mat singleMask;
        bool res = computeSingleMask(mask, i, singleMask);
        if (res) {
            masks.push_back(singleMask);
        }
    }
    LOG_INFO << "masks.size = " << masks.size();
    LOG_INFO << "getMask get 2D masks: " << masks.size() << " of " << max << " raw ones";
    return true;
}


int main(){
   std::vector<boost::shared_ptr<VisionInputImage>> images;
   static cv::Mat img2d = cv::imread("/home/cobot/ss/no_good/2dImage.bmp");
    cv::cvtColor(img2d,img2d,cv::COLOR_RGB2GRAY);
    static cv::Mat pointCloud;
    cv::FileStorage fr("/home/cobot/ss/no_good/3dImage.yml", cv::FileStorage::READ);
    fr["data"]>>pointCloud;


   auto _2dImage = new VisionInputImage(boost::shared_ptr<cv::Mat>(&img2d),ImageType::Mono);
   images.push_back(boost::shared_ptr<VisionInputImage>(_2dImage));


   auto cloudImage = new VisionInputImage(boost::shared_ptr<cv::Mat>(&pointCloud),ImageType::Cloud);
   images.push_back(boost::shared_ptr<VisionInputImage>(cloudImage));

    boost::shared_ptr<VisionFactoryInterface> visionFactory = VisionFactoryInterface::create();
    boost::shared_ptr<VisionInterface> visionClient = visionFactory->createVision();

    boost::shared_ptr<VisionGoodsInfo> goodsInfo = boost::shared_ptr<VisionGoodsInfo>(new VisionGoodsInfo());

    goodsInfo->visionAlgorithmType = VisionAlgorithmType::Box;
//        goodsInfo->visionAlgorithmType=VisionAlgorithmType::DeepLearningTube;
    goodsInfo->barCode="6956653490269";
    goodsInfo->algorithmNumber=2;
    goodsInfo->locateBinBoxEnabled=false;
    visionClient->capacity(goodsInfo,EAGLE_DATA_NS_API::VisionCapacity::enable2DCapture);

    visionClient->requestProcessVisionImage(goodsInfo,images);

    auto outImages = visionClient->getProcessedImage();

}


