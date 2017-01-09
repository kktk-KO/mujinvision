#include <mujinvision/mujinvisionmanager.h>

using namespace mujinvision;

class UserImageSubscriberManager : public ImageSubscriberManager
{
public:
    UserImageSubscriberManager(){
    }

    virtual ~UserImageSubscriberManager() {
    }

    void Initialize(const std::map<std::string, CameraPtr >&mNameCamera, const std::string& streamerIp, const unsigned int streamerPort, const rapidjson::Document& params, boost::shared_ptr<zmq::context_t>) {
    }

    void DeInitialize() {
    }

    void Reset() {
    }

    void UpdateCameras(const std::map<std::string, CameraPtr >&mNameCamera) {
    }

    ImagePtr GetColorImageFromBuffer(const std::string& cameraname, unsigned long long& timestamp, unsigned long long& endtimestamp, const double timeout)
    {
        return ImagePtr();
    }
    ImagePtr SnapColorImage(const std::string& cameraname, unsigned long long& timestamp, unsigned long long& endtimestamp, const double timeout=1.0)
    {
        return ImagePtr();
    }

    ImagePtr GetDepthImageFromBuffer(const std::string& cameraname, unsigned long long& starttime, unsigned long long& endtime, const double timeout)
    {
        return ImagePtr();
    }
    ImagePtr SnapDepthImage(const std::string& cameraname, unsigned long long& starttime, unsigned long long& endtime, const double timeout=1.0, const int numimages=-1)
    {
        return ImagePtr();
    }
    ImagePtr SnapDetectionResult(const std::string& cameraname, const double timeout=20.0 /*sec*/, const std::string& extraoptions="")
    {
        return ImagePtr();
    }
    void SnapColorAndDepthImages(const std::string& cameraname, unsigned long long& starttime, unsigned long long& endtime, std::vector<ImagePtr>& colorimages, std::vector<ImagePtr>& depthimages, const double timeout=1.0, const int numimages=-1, const std::string& extraoptions="")
    {
    }
    void GetImagePackFromBuffer(const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, std::vector<ImagePtr>& colorimages, std::vector<ImagePtr>& depthimages, std::vector<ImagePtr>& resultimages, unsigned long long& starttime, unsigned long long& endtime, unsigned long long& imagepacktimestamp, const double timeout, const unsigned long long laterthan)
    {
    }
    void StartCaptureThread(const std::vector<std::string>& cameranames, const double timeout=5.0, const int numimages=-1, const std::string& extraoptions="")
    {
    }

    void StopCaptureThread(const std::vector<std::string>& cameranames, const double timeout=5.0)
    {
    }

    uint64_t GetSubscribeStartedTimeStamp() const
    {
        return 0;
    }

    int GetCollisionPointCloud(const std::string& cameraname, std::vector<double>& points, unsigned long long& starttime, unsigned long long& endtime, const double voxelsize, const double stddev, const size_t numnn, const std::string& regionnameforocclusionchecking="", const double timeout=20.0, unsigned long long newerthan=0, const size_t subsample=1) {
        return -2;
    }

    int GetCollisionPointCloud(const std::string& cameraname, ImagePtr depthimage, std::vector<double>& points, const double voxelsize, const double stddev, const size_t numnn, const std::string& regionnameforocclusionchecking="", const double timeout=20.0, unsigned long long newerthan=0, const size_t subsample=1) {
        return -2;
    }
};

typedef boost::shared_ptr<UserImageSubscriberManager> UserImageSubscriberManagerPtr;
typedef boost::shared_ptr<UserImageSubscriberManager const> UserImageSubscriberManagerConstPtr;
typedef boost::weak_ptr<UserImageSubscriberManager> UserImageSubscriberManagerWeakPtr;

class UserObjectDetector : public ObjectDetector
{
public:
    UserObjectDetector() {
    }

    virtual ~UserObjectDetector() {
    }

    void Initialize(const std::string& detectorconf, const std::string& targetname, const std::map<std::string, RegionPtr >& mNameRegion, const std::map<std::string, std::map<std::string, CameraPtr> >& mNameCamera, const std::map< std::string, std::string>& extraInitializationOptions = std::map< std::string, std::string>(), const CheckPreemptFn& preemptfn = CheckPreemptFn(), const bool getgil=false) {
    }

    void DeInitialize() {
    }

    void SetNumThreads(uint32_t nThreads) {
    }

    uint32_t GetNumThreads() const {
    }

    void UpdateRegion(const std::string& regionname, RegionPtr region, const std::map<std::string, CameraPtr >& mNameCamera) {
    }

    void DetectObjects(const std::string& regionname, const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, std::vector<DetectedObjectPtr>& detectedobjects, std::string& resultstate, const bool fastdetection, const bool bindetection, const bool checkcontaineremptyonly, const unsigned int checkpreemptbits) {
    }

    void DetectObjects(const std::string& regionname, const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, const std::vector<ImagePtr>& resultimages, std::vector<DetectedObjectPtr>& detectedobjects, std::string& resultstate, const bool fastdetection, const bool bindetection, const bool checkcontaineremptyonly, const unsigned int checkpreemptbits) {
    }

    void DetectInImage(const std::string& regionname, const std::string& colorcameraname, std::vector<DetectedObjectPtr>& resultscolorcamera) {
    }

    void RefineDetectionWithDepthData(const std::string& regionname, const std::string& colorcameraname, const std::string& depthcameraname, const std::vector<DetectedObjectPtr>& resultscolorcamera, std::vector<DetectedObjectPtr>& resultsdepthcamera, std::vector<unsigned int>& indicescolorcamera) {
    }

    void GetPointCloudObstacle(const std::string& regionname, const std::string& depthcameraname, const std::vector<DetectedObjectPtr>& resultsdepthcamera, std::vector<double>& points, const double voxelsize=0.01, const bool fast=false, const bool getgil=false, const double stddev=0.01, const size_t numnn=80, const unsigned int checkpreemptbits=1) {
    }

    void GetCameraPointCloud(const std::string& regionname, const std::string& depthcameraname, ImageConstPtr depthimage, std::vector<double>& points, const double voxelsize=0.01) {
    }

    void SetImage(const std::string& colorcameraname, ImageConstPtr colorimage) {
    }

    void SetColorImage(ImagePtr colorimage) {
    }

    void SetDepthImage(ImagePtr depthimage) {
    }
};
typedef boost::shared_ptr<UserObjectDetector> UserObjectDetectorPtr;

class UserDetectorManager : public DetectorManager
{
public:
    UserDetectorManager() {
    }
    virtual ~UserDetectorManager() {
    }

    ObjectDetectorPtr CreateObjectDetector(const std::string& detectorconfig, const std::string& targetname, const std::map<std::string, RegionPtr >& mNameRegion, const std::map<std::string, std::map<std::string, CameraPtr> >& mNameCamera, const boost::function<void(const std::string& msg, const std::string& err)>& setstatusfn, const std::map<std::string, std::string>& extraInitializationOptions = std::map<std::string, std::string>(), const CheckPreemptFn& preemptfn=CheckPreemptFn())
    {
        UserObjectDetectorPtr detector(new UserObjectDetector());
        return boost::dynamic_pointer_cast<ObjectDetector>(detector);
    }
};

typedef boost::shared_ptr<UserDetectorManager> UserDetectorManagerPtr;
typedef boost::shared_ptr<UserDetectorManager const> UserDetectorManagerConstPtr;
typedef boost::weak_ptr<UserDetectorManager> UserDetectorManagerWeakPtr;

int main(int argc, char* argv[])
{
    UserImageSubscriberManagerPtr imagesubscribermanager(new UserImageSubscriberManager());
    UserDetectorManagerPtr detectormanager(new UserDetectorManager());
    MujinVisionManagerPtr visionmanager;
    visionmanager.reset(new MujinVisionManager(imagesubscribermanager, detectormanager,50001, 50002, 50003, "/tmp", "/tmp"));
    
    while (!visionmanager->IsShutdown()) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
}
