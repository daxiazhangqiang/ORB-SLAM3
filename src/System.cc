/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

void System::SaveMapPointsPLY(const string &filename)
{
    cout << "Saving map points to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    // 每个子图一种颜色：PLY 本身没有“分组”概念，颜色是唯一能在 CloudCompare 里目视区分的办法
    // 注意：未融合过的子图各自独立坐标系（尺度/位姿都不对应），这份合并文件只是叠加展示
    static const int kPalette[10][3] = {
        {0, 255, 0},   {255, 64, 64},  {64, 128, 255}, {255, 200, 0},  {180, 80, 255},
        {0, 220, 220}, {255, 120, 200}, {160, 255, 80}, {255, 255, 255}, {120, 120, 120}};

    // 先一次性收集所有有效地图点坐标与颜色，保证头部声明数量与实际写入数量完全一致
    vector<Eigen::Vector3f> vPoints;
    vector<int> vColors;
    size_t mapIndex = 0;
    for(Map* pMap : vpMaps)
    {
        vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();
        for(MapPoint* pMP : vpMPs)
        {
            if(!pMP->isBad())
            {
                vPoints.push_back(pMP->GetWorldPos());
                vColors.push_back(static_cast<int>(mapIndex % 10));
            }
        }
        mapIndex++;
    }

    ofstream f(filename.c_str());
    if(!f.is_open())
    {
        cerr << "ERROR: could not open " << filename << endl;
        return;
    }
    f << fixed << setprecision(9);
    f << "ply" << endl;
    f << "format ascii 1.0" << endl;
    f << "comment ORB-SLAM3 map points; ALL maps in one file, color = submap (unaligned frames!)" << endl;
    f << "comment for per-submap files use map_points_mapNN.ply" << endl;
    f << "element vertex " << vPoints.size() << endl;
    f << "property float x" << endl;
    f << "property float y" << endl;
    f << "property float z" << endl;
    f << "property uchar red" << endl;
    f << "property uchar green" << endl;
    f << "property uchar blue" << endl;
    f << "end_header" << endl;

    for(size_t i = 0; i < vPoints.size(); i++)
    {
        const int* c = kPalette[vColors[i]];
        f << vPoints[i](0) << " " << vPoints[i](1) << " " << vPoints[i](2) << " "
          << c[0] << " " << c[1] << " " << c[2] << endl;
    }

    f.close();
    cout << "Saved " << vPoints.size() << " map points from " << vpMaps.size()
         << " submaps to " << filename << "（各子图坐标系未对齐，颜色区分）" << endl;
}

void System::SaveMapPointsPLYPerMap(const string &prefix)
{
    // 逐子图单独导出：每个文件都在自己子图的坐标系里，CloudCompare 可以任意挑一张看
    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    for(Map* pMap : vpMaps)
    {
        vector<Eigen::Vector3f> vPoints;
        vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();
        for(MapPoint* pMP : vpMPs)
            if(!pMP->isBad())
                vPoints.push_back(pMP->GetWorldPos());

        const string name = prefix + "_map" + to_string(pMap->GetId()) + ".ply";
        ofstream f(name.c_str());
        if(!f.is_open())
        {
            cerr << "ERROR: could not open " << name << endl;
            continue;
        }
        f << fixed << setprecision(9);
        f << "ply" << endl;
        f << "format ascii 1.0" << endl;
        f << "comment ORB-SLAM3 submap id=" << pMap->GetId()
          << " keyframes=" << pMap->GetAllKeyFrames().size()
          << " (own coordinate frame)" << endl;
        f << "element vertex " << vPoints.size() << endl;
        f << "property float x" << endl;
        f << "property float y" << endl;
        f << "property float z" << endl;
        f << "property uchar red" << endl;
        f << "property uchar green" << endl;
        f << "property uchar blue" << endl;
        f << "end_header" << endl;
        for(const Eigen::Vector3f& pos : vPoints)
            f << pos(0) << " " << pos(1) << " " << pos(2) << " 0 255 0" << endl;
        f.close();
        cout << "Saved " << vPoints.size() << " map points (submap id=" << pMap->GetId()
             << ", KFs=" << pMap->GetAllKeyFrames().size() << ") to " << name << endl;
    }
}

void System::SaveKeyFramesPLY(const string &filename)
{
    cout << "Saving keyframes to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();

    // 先一次性收集所有有效关键帧相机中心，保证头部声明数量与实际写入数量一致
    vector<Eigen::Vector3f> vCenters;
    for(KeyFrame* pKF : vpKFs)
        if(!pKF->isBad())
            vCenters.push_back(pKF->GetCameraCenter());

    ofstream f(filename.c_str());
    if(!f.is_open())
    {
        cerr << "ERROR: could not open " << filename << endl;
        return;
    }
    f << fixed << setprecision(9);
    f << "ply" << endl;
    f << "format ascii 1.0" << endl;
    f << "comment ORB-SLAM3 keyframe camera centers" << endl;
    f << "element vertex " << vCenters.size() << endl;
    f << "property float x" << endl;
    f << "property float y" << endl;
    f << "property float z" << endl;
    f << "property uchar red" << endl;
    f << "property uchar green" << endl;
    f << "property uchar blue" << endl;
    f << "end_header" << endl;

    for(const Eigen::Vector3f& c : vCenters)
        f << c(0) << " " << c(1) << " " << c(2) << " 255 0 0" << endl;

    f.close();
    cout << "Saved " << vCenters.size() << " keyframes to " << filename << endl;
}

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const int initFr, const string &strSequence):
    mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbResetActiveMap(false),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;
    else if(mSensor==IMU_RGBD)
        cout << "RGB-D-Inertial" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if(!node.empty() && node.isString() && node.string() == "1.0"){
        settings_ = new Settings(strSettingsFile,mSensor);

        mStrLoadAtlasFromFile = settings_->atlasLoadFile();
        mStrSaveAtlasToFile = settings_->atlasSaveFile();

        cout << (*settings_) << endl;
    }
    else{
        settings_ = nullptr;
        cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
        if(!node.empty() && node.isString())
        {
            mStrLoadAtlasFromFile = (string)node;
        }

        node = fsSettings["System.SaveAtlasToFile"];
        if(!node.empty() && node.isString())
        {
            mStrSaveAtlasToFile = (string)node;
        }
    }

    node = fsSettings["loopClosing"];
    bool activeLC = true;
    if(!node.empty())
    {
        activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }

    mStrVocabularyFilePath = strVocFile;

    bool loadedAtlas = false;

    if(mStrLoadAtlasFromFile.empty())
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Atlas
        cout << "Initialization of Atlas from scratch " << endl;
        mpAtlas = new Atlas(0);
    }
    else
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        cout << "Load File" << endl;

        // Load the file with an earlier session
        //clock_t start = clock();
        cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if(!isRead)
        {
            cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
            exit(-1);
        }
        //mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);


        //cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " << mpKeyFrameDatabase->mnNumWords << endl;

        loadedAtlas = true;

        mpAtlas->CreateNewMap();

        //clock_t timeElapsed = clock() - start;
        //unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
        //cout << "Binary file read in " << msElapsed << " ms" << endl;

        //usleep(10*1000*1000);
    }


    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR || mSensor==IMU_RGBD)
        mpAtlas->SetInertialSensor();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    cout << "Seq. Name: " << strSequence << endl;
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_, strSequence);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR,
                                     mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD, strSequence);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);
    mpLocalMapper->mInitFr = initFr;
    if(settings_)
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    else
        mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if(mpLocalMapper->mThFarPoints!=0)
    {
        cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    //Initialize the Loop Closing thread and launch
    // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR, activeLC); // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    //usleep(10*1000*1000);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    //if(false) // TODO
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
    }

    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

}

Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=STEREO && mSensor!=IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if(settings_ && settings_->needToRectify()){
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if(settings_ && settings_->needToResize()){
        cv::resize(imLeft,imLeftToFeed,settings_->newImSize());
        cv::resize(imRight,imRightToFeed,settings_->newImSize());
    }
    else{
        imLeftToFeed = imLeft.clone();
        imRightToFeed = imRight.clone();
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
    {
        vector<IMU::Point> vImu = ApplyImuBias(vImuMeas);
        for(size_t i_imu = 0; i_imu < vImu.size(); i_imu++)
            mpTracker->GrabImuData(vImu[i_imu]);
    }

    // std::cout << "start GrabImageStereo" << std::endl;
    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed,imRightToFeed,timestamp,filename);

    // std::cout << "out grabber" << std::endl;

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=RGBD  && mSensor!=IMU_RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    cv::Mat imDepthToFeed = depthmap.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;

        cv::resize(depthmap,imDepthToFeed,settings_->newImSize());
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_RGBD)
    {
        vector<IMU::Point> vImu = ApplyImuBias(vImuMeas);
        for(size_t i_imu = 0; i_imu < vImu.size(); i_imu++)
            mpTracker->GrabImuData(vImu[i_imu]);
    }

    Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed,imDepthToFeed,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{

    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbShutDown)
            return Sophus::SE3f();
    }

    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            cout << "SYSTEM-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        vector<IMU::Point> vImu = ApplyImuBias(vImuMeas);
        for(size_t i_imu = 0; i_imu < vImu.size(); i_imu++)
            mpTracker->GrabImuData(vImu[i_imu]);
    }

    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}



void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

bool System::SelectLargestMap(int* mapId, size_t* keyframes)
{
    Map* best = nullptr;
    size_t bestKFs = 0;
    for(Map* pMap : mpAtlas->GetAllMaps())
    {
        if(!pMap || pMap->IsBad())
            continue;
        const size_t count = pMap->GetAllKeyFrames().size();
        if(count > bestKFs)
        {
            best = pMap;
            bestKFs = count;
        }
    }
    if(!best)
        return false;
    mpAtlas->ChangeMap(best);
    if(mapId)
        *mapId = static_cast<int>(best->GetId());
    if(keyframes)
        *keyframes = bestKFs;
    return true;
}

void System::EnableRelocalizationOnly()
{
    // 先停建图线程，再让跟踪器从首帧开始只做重定位
    mpLocalMapper->RequestStop();
    while(!mpLocalMapper->isStopped())
        usleep(1000);
    mpTracker->InformOnlyTracking(true);
    mpTracker->InformRelocalizationOnly(true);
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

namespace {

// 有界等待 + join 一个后台线程。
// 目的：不要用"退出时挂死"换掉"退出时崩溃"，所以两种情况特意**不** join：
//   ① 调用者本身就是这个线程 —— viewer 菜单里的 Stop 按钮会从 viewer 线程调用
//      System::Shutdown()（src/Viewer.cc），此时 join 自己会抛 system_error 并 terminate；
//   ② 超时内还没结束 —— 例如 LoopClosing 正卡在一次 FullInertialBA 里（上游注释里
//      也留了 "isRunningGBA → break anyway" 的同类处理）。
// 这两种情况下线程保持原样，由进程退出时统一回收，与改动前的行为一致。
template <typename TThreadOwner>
void JoinThreadBounded(std::thread* pThread, TThreadOwner* pOwner,
                       const char* name, int timeout_ms)
{
    if(!pThread || !pOwner)
        return;

    if(pThread->get_id() == std::this_thread::get_id())   // ① 不能 join 自己
        return;

    int waited_ms = 0;
    while(waited_ms < timeout_ms && !pOwner->isFinished())
    {
        usleep(5000);
        waited_ms += 5;
    }

    if(pOwner->isFinished())
    {
        if(pThread->joinable())
            pThread->join();
    }
    else                                                  // ② 等不到就不等，别把退出变成挂死
    {
        cout << "[i] " << name << " thread is still running after "
             << timeout_ms << " ms; leaving it to process teardown." << endl;
    }
}

} // namespace

void System::Shutdown()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbShutDown)      // 幂等：示例会显式调一次，~System() 之后还会再调一次
            return;
        mbShutDown = true;
    }

    cout << "Shutdown" << endl;

    // 停机 + 有界等待抽到 RequestMappingStop()：请求结束、等后台静默、幂等可重入
    RequestMappingStop();

    // Atlas 序列化必须在地图写线程停止后进行，超时则拒绝保存不一致快照
    if(!mStrSaveAtlasToFile.empty())
    {
        if(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()
           || mpLoopCloser->isRunningGBA())
            throw std::runtime_error("Atlas save refused: mapping/loop/GBA still running");
        Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
        SaveAtlas(FileType::BINARY_FILE);
    }

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif


}

System::~System()
{
    // RAII 兜底：调用者（示例、以及以后要写的 ROS2 节点）忘记显式 Shutdown() 时，
    // 也不能让后台线程在进程拆台时继续跑。
    //
    // 这里只"停线程 + join"，刻意不 delete mpTracker / mpAtlas / ...：退出阶段真正的
    // 风险是"边拆对象边有线程在用"，而不是内存没还 —— 进程马上结束，未释放的内存由内核
    // 回收，代价为零；反过来 delete 会把线程间交叉引用、GL 上下文归属这些所有权问题引进来。
    Shutdown();
}

bool System::isShutDown() {
    unique_lock<mutex> lock(mMutexReset);
    return mbShutDown;
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }
    f.close();
    // cout << endl << "trajectory saved!" << endl;
}

std::vector<IMU::Point> System::ApplyImuBias(const std::vector<IMU::Point>& vImuMeas)
{
    // 把 yaml 里的 IMU.AccBias / IMU.GyroBias（常数零偏**先验**）从测量里扣掉。
    // 不填这两个字段时是零向量 → 原样返回，行为与原生 ORB-SLAM3 完全一致。
    // 注意：这只是先验；算法内部对零偏的随机游走建模与在线估计照旧。
    if(!settings_)
        return vImuMeas;
    const Eigen::Vector3f ba = settings_->imuAccBias();
    const Eigen::Vector3f bg = settings_->imuGyroBias();
    if(ba.isZero(0.0f) && bg.isZero(0.0f))
        return vImuMeas;

    std::vector<IMU::Point> vOut;
    vOut.reserve(vImuMeas.size());
    for(const IMU::Point& p : vImuMeas)
        vOut.emplace_back(IMU::Point(p.a.x()-ba.x(), p.a.y()-ba.y(), p.a.z()-ba.z(),
                                     p.w.x()-bg.x(), p.w.y()-bg.y(), p.w.z()-bg.z(), p.t));
    return vOut;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
          << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

    }

    f.close();
}

void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    int numMaxKFs = 0;
    Map* pBiggerMap = nullptr;
    std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
    for(Map* pMap :vpMaps)
    {
        std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    // 所有地图都是 0 个关键帧时（例如惯性初始化从未成功），pBiggerMap 从未被赋值。
    // 原来这里直接用 pBiggerMap->GetAllKeyFrames()，会解引用未初始化指针 → SIGSEGV。
    if(!pBiggerMap || numMaxKFs == 0)
    {
        std::cout << "[i] No map has any keyframe (initialization never succeeded) - no trajectory to save." << std::endl;
        return;
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

void System::SaveTrajectoryEuRoC(const string &filename, Map* pMap)
{

    cout << endl << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    int numMaxKFs = 0;

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

/*void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap = nullptr;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap || numMaxKFs == 0)   // 同 SaveTrajectoryEuRoC：空地图时别解引用空/未初始化指针
    {
        std::cout << "[i] No map has any keyframe - no keyframe trajectory to save." << std::endl;
        return;
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose_();
    else
        Twb = vpKFs[0]->GetPoseInverse_();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;


        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
            Sophus::SE3f Twb = Tbw.inverse();

            Eigen::Vector3f twb = Twb.translation();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}*/


/*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap = nullptr;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap || numMaxKFs == 0)   // 同 SaveTrajectoryEuRoC：空地图时别解引用空/未初始化指针
    {
        std::cout << "[i] No map has any keyframe - no trajectory to save." << std::endl;
        return;
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    }
    f.close();
}*/

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap = nullptr;   // 下面的 if(!pBiggerMap) 只有先初始化才可靠
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap)
{
    cout << endl << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

/*void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPoseCv() * Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
}*/

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        if(!pKF)
            continue;

        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Tow;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << setprecision(9) << Rwc(0,0) << " " << Rwc(0,1)  << " " << Rwc(0,2) << " "  << twc(0) << " " <<
             Rwc(1,0) << " " << Rwc(1,1)  << " " << Rwc(1,2) << " "  << twc(1) << " " <<
             Rwc(2,0) << " " << Rwc(2,1)  << " " << Rwc(2,2) << " "  << twc(2) << endl;
    }
    f.close();
}


void System::SaveDebugData(const int &initIdx)
{
    // 0. Save initialization trajectory
    SaveTrajectoryEuRoC("init_FrameTrajectoy_" +to_string(mpLocalMapper->mInitSect)+ "_" + to_string(initIdx)+".txt");

    // 1. Save scale
    ofstream f;
    f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mScale << endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mRwg(0,0) << "," << mpLocalMapper->mRwg(0,1) << "," << mpLocalMapper->mRwg(0,2) << endl;
    f << mpLocalMapper->mRwg(1,0) << "," << mpLocalMapper->mRwg(1,1) << "," << mpLocalMapper->mRwg(1,2) << endl;
    f << mpLocalMapper->mRwg(2,0) << "," << mpLocalMapper->mRwg(2,1) << "," << mpLocalMapper->mRwg(2,2) << endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mCostTime << endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
    f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" +to_string(mpLocalMapper->mInitSect)+ "_" +to_string(initIdx)+".txt", ios_base::app);
    f << fixed;
    for(int i=0; i<mpLocalMapper->mcovInertial.rows(); i++)
    {
        for(int j=0; j<mpLocalMapper->mcovInertial.cols(); j++)
        {
            if(j!=0)
                f << ",";
            f << setprecision(15) << mpLocalMapper->mcovInertial(i,j);
        }
        f << endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mInitTime << endl;
    f.close();
}


int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

int System::GetTrackingInliers()
{
    return mpTracker->GetMatchesInliers();
}


namespace {
// 多关键帧几何一致性判据，不改变角点阈值或地图点剔除规则
bool QualityStablePoint(MapPoint* point)
{
    if (!point || point->isBad()) return false;
    const auto observations = point->GetObservations();
    if (observations.size() < 3) return false;
    const Eigen::Vector3f world = point->GetWorldPos();
    if (!world.allFinite()) return false;
    std::vector<Eigen::Vector3f> rays;
    double first = 1e30, last = -1e30;
    int tested = 0, good = 0;
    for (const auto& observation : observations) {
        auto* kf = observation.first;
        const int index = std::get<0>(observation.second);
        if (!kf || kf->isBad() || index < 0 || index >= int(kf->mvKeysUn.size())) continue;
        if (++tested > 12) { --tested; break; }
        const auto camera_point = kf->GetPose() * world;
        if (camera_point.z() <= 0 || !camera_point.allFinite()) continue;
        const auto& key = kf->mvKeysUn[index];
        const auto pixel = kf->mpCamera->project(camera_point);
        const double dx = pixel.x() - key.pt.x, dy = pixel.y() - key.pt.y;
        if (key.octave < 0 || key.octave >= int(kf->mvInvLevelSigma2.size())) continue;
        const double error = (dx*dx + dy*dy) * kf->mvInvLevelSigma2[key.octave];
        if (!std::isfinite(error) || error > 5.991) continue;
        ++good;
        first = std::min(first, kf->mTimeStamp); last = std::max(last, kf->mTimeStamp);
        rays.push_back((world - kf->GetCameraCenter()).normalized());
    }
    if (good < 3 || good < tested * .8 || last - first < 1.0) return false;
    for (size_t i = 0; i < rays.size(); ++i)
        for (size_t j = i + 1; j < rays.size(); ++j)
            if (rays[i].dot(rays[j]) < std::cos(1.5 * M_PI / 180.0)) return true;
    return false;
}
}

// 跟踪线程在帧处理后采样，地图更新锁保护几何快照；最多扫描2000点避免无界遍历
MapQualitySample System::GetMapQuality(bool inventory)
{
    MapQualitySample result;
    result.state = GetTrackingState();
    auto maps = mpAtlas->GetAllMaps();
    Map* primary = nullptr;
    for (auto* map : maps)
        if (map && !map->IsBad() && (!primary || map->KeyFramesInMap() > primary->KeyFramesInMap() ||
            (map->KeyFramesInMap() == primary->KeyFramesInMap() && map->GetId() < primary->GetId()))) primary = map;
    if (!primary) return result;
    result.map_id = primary->GetId();
    result.active_map_id = mpAtlas->GetCurrentMap()->GetId();
    // 仅读取各子图容器计数，不遍历其他子图的地图点
    for (auto* map : maps)
        if (map && map != primary && !map->IsBad())
            result.other_submaps.emplace_back(map->GetId(), map->MapPointsInMap());
    std::unique_lock<std::mutex> lock(primary->mMutexMapUpdate);
    result.revision = primary->GetLastBigChangeIdx();
    result.keyframes = primary->KeyFramesInMap(); result.points = primary->MapPointsInMap();
    auto keyframes = primary->GetAllKeyFrames();
    keyframes.erase(std::remove_if(keyframes.begin(), keyframes.end(), [](KeyFrame* k) { return !k || k->isBad(); }), keyframes.end());
    std::sort(keyframes.begin(), keyframes.end(), [](KeyFrame* a, KeyFrame* b) { return a->mTimeStamp < b->mTimeStamp; });
    std::map<unsigned long, KeyFrame*> anchors;
    // 每十个最终关键帧选一个位置朝向锚点，首尾均保留，单位不依赖尺度
    for (size_t i = 0; i < keyframes.size(); ++i)
        if (i % 10 == 0 || i + 1 == keyframes.size()) {
            anchors[keyframes[i]->mnId] = keyframes[i];
            result.route_anchors.push_back(keyframes[i]->mnId);
        }
    if (inventory) {
        auto points = primary->GetAllMapPoints();
        const size_t step = std::max(size_t(1), (points.size() + 1999) / 2000);
        for (size_t i = 0; i < points.size(); i += step) {
            ++result.tested_points;
            if (QualityStablePoint(points[i])) ++result.stable_points;
        }
    }
    if (result.state != 2 || result.active_map_id != result.map_id) return result;
    const Frame& frame = mpTracker->mCurrentFrame;
    const auto center = frame.GetPose().inverse().translation();
    std::set<int> cells;
    std::map<unsigned long, int> votes;
    for (size_t i = 0; i < frame.mvpMapPoints.size(); ++i) {
        auto* point = frame.mvpMapPoints[i];
        if (!point || point->isBad() || point->GetMap() != primary ||
            i >= frame.mvbOutlier.size() || frame.mvbOutlier[i] || point->Observations() < 1) continue;
        ++result.inliers;
        if (!QualityStablePoint(point)) continue;
        ++result.stable_matches;
        if (i < frame.mvKeysUn.size()) {
            const auto& pt = frame.mvKeysUn[i].pt;
            if (pt.x >= 0 && pt.x < 640 && pt.y >= 0 && pt.y < 480)
                cells.insert(int(pt.y / 160) * 4 + int(pt.x / 160));
        }
        const auto world = point->GetWorldPos();
        for (const auto& observation : point->GetObservations()) {
            auto* kf = observation.first;
            if (!kf || !anchors.count(kf->mnId)) continue;
            const Eigen::Vector3f ray = world - kf->GetCameraCenter();
            const Eigen::Vector3f current_ray = world - center;
            // 匹配同一点仍须验证观察方向和相对深度范围，不能远处静止刷覆盖
            if (ray.norm() > 1e-6 && current_ray.norm() > 1e-6 &&
                (center - kf->GetCameraCenter()).norm() <= .3 * ray.norm() &&
                ray.normalized().dot(current_ray.normalized()) >= std::cos(30.0 * M_PI / 180.0) &&
                (frame.GetPose().rotationMatrix() * kf->GetPose().rotationMatrix().transpose()).trace() >= 1 + 2*std::cos(30.0*M_PI/180.0))
                ++votes[kf->mnId];
        }
    }
    result.grid_cells = cells.size();
    for (const auto& vote : votes) if (vote.second >= 12) result.visible_anchors.push_back(vote.first);
    return result;
}

int System::GetRelocalizationInliers()
{
    return mpTracker->GetRelocalizationInliers();
}

// 只读取容器计数，适合分钟级监控
vector<System::SubmapStats> System::GetSubmapStats()
{
    vector<SubmapStats> result;
    for(Map* m : mpAtlas->GetAllMaps())
        if(m && !m->IsBad())
            result.push_back({m->GetId(), m->KeyFramesInMap(), m->MapPointsInMap()});
    sort(result.begin(), result.end(), [](const SubmapStats& a, const SubmapStats& b) {
        return a.keyframes != b.keyframes ? a.keyframes > b.keyframes : a.id < b.id;
    });
    return result;
}

// 以最终有效关键帧为尺度依据，避免跨断图或使用优化前的历史位姿
void System::PrepareSubmapExport(const string &prefix)
{
    RequestMappingStop();
    if(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        throw runtime_error("Submap export refused: mapping/loop/GBA still running");
    mpAtlas->PreSave();
    ofstream manifest(prefix + "_submaps.csv");
    manifest.exceptions(ios::badbit | ios::failbit);
    manifest << "map_id,keyframes,map_points,primary\n";
    auto stats = GetSubmapStats();
    for(const auto& st : stats)
    {
        manifest << st.id << ',' << st.keyframes << ',' << st.points << ','
                 << (st.id == stats.front().id ? 1 : 0) << '\n';
        for(Map* m : mpAtlas->GetAllMaps())
            if(m->GetId() == st.id)
                SaveKeyFrameTrajectoryEuRoC(prefix + "_" + to_string(st.id) + "_kf_raw.txt", m);
    }
}

// 序列化列表仅在后台静默时切换，单图文件沿用标准 Atlas 格式
void System::SaveSubmaps(const string &prefix, unsigned long primaryId, double scale, bool saveAtlas)
{
    if(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        throw runtime_error("Submap save refused: mapping/loop/GBA still running");
    const auto stats = GetSubmapStats();
    if(stats.empty()) return;
    const unsigned long mainId = stats.front().id;
    if(!std::isfinite(static_cast<float>(scale)) || scale <= 0.0
       || (primaryId != static_cast<unsigned long>(-1) && primaryId != mainId)
       || (primaryId == static_cast<unsigned long>(-1) && scale != 1.0))
        throw runtime_error("Invalid primary map scale");
    for(Map* m : mpAtlas->GetAllMaps())
        if(m->GetId() == mainId && scale != 1.0)
            m->ApplyScaledRotation(Sophus::SE3f(), static_cast<float>(scale), true);
    mpAtlas->PreSave();
    const auto backup = mpAtlas->mvpBackupMaps;
    const string checksum = saveAtlas ? CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE) : "";
    const string voc = mStrVocabularyFilePath.substr(mStrVocabularyFilePath.find_last_of("/\\") + 1);
    // 临时文件写完后原子替换，避免留下看似完整的半份地图
    auto save = [&](const string& path) {
        const string tmp = path + ".tmp";
        ofstream out(tmp, ios::binary);
        out.exceptions(ios::badbit | ios::failbit);
        { boost::archive::binary_oarchive ar(out); ar << voc; ar << checksum; ar << mpAtlas; }
        out.close();
        if(std::rename(tmp.c_str(), path.c_str()) != 0) throw runtime_error("Cannot rename " + tmp);
    };
    try {
        if(saveAtlas) save(prefix + ".osa");
        for(Map* m : backup)
        {
            const string base = prefix + "_" + to_string(m->GetId());
            mpAtlas->mvpBackupMaps = {m};
            if(saveAtlas) save(base + ".osa");
            vector<Eigen::Vector3f> points;
            for(MapPoint* p : m->GetAllMapPoints())
                if(p && !p->isBad()) points.push_back(p->GetWorldPos());
            ofstream ply(base + ".ply");
            ply.exceptions(ios::badbit | ios::failbit);
            ply << "ply\nformat ascii 1.0\ncomment submap " << m->GetId()
                << " independent coordinates; " << (m->GetId() == mainId && primaryId == mainId ? "wheel scaled" : "native units")
                << "\nelement vertex " << points.size()
                << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n" << setprecision(9);
            for(const auto& p : points) ply << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
            SaveKeyFrameTrajectoryEuRoC(base + "_kf.txt", m);
            cout << "[submap] " << base << " KFs=" << m->KeyFramesInMap()
                 << " points=" << points.size() << (m->GetId() == mainId ? " PRIMARY" : "") << endl;
        }
    } catch(...) { mpAtlas->mvpBackupMaps = backup; throw; }
    mpAtlas->mvpBackupMaps = backup;
}

int System::GetAtlasMapCount()
{
    // CountMaps() 自身持 mMutexAtlas，这里不再加锁，避免同锁重入
    return mpAtlas->CountMaps();
}

int System::GetKeyFrameCount()
{
    return static_cast<int>(mpAtlas->GetNumLivedKF());
}

double System::GetLastKeyFrameTimestamp()
{
    // Tracking::GetLastKeyFrame() 是公开接口；mpLastKeyFrame 本身是 protected
    KeyFrame* pLast = mpTracker->GetLastKeyFrame();
    return pLast ? pLast->mTimeStamp : -1.0;
}

void System::RequestMappingStop()
{
    // 幂等：RequestFinish 只置标志；JoinThreadBounded 在已 join 后是空操作
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
        mpViewer->RequestFinish();

    JoinThreadBounded(mptViewer,       mpViewer,       "Viewer",       2000);
    JoinThreadBounded(mptLocalMapping, mpLocalMapper, "LocalMapping", 5000);
    JoinThreadBounded(mptLoopClosing,  mpLoopCloser,  "LoopClosing",  30000);
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    if ((aux>0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if ((mpTracker->mState==Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}


bool System::isFinished()
{
    return (GetTimeFromIMUInit()>0.1);
}

void System::ChangeDataset()
{
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

float System::GetImageScale()
{
    return mpTracker->GetImageScale();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double& time)
{
    mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double& time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

void System::SaveAtlas(int type){
    if(!mStrSaveAtlasToFile.empty())
    {
        //clock_t start = clock();

        // Save the current session
        mpAtlas->PreSave();

        string pathSaveFileName = "./";
        pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
        pathSaveFileName = pathSaveFileName.append(".osa");

        string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);
        std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
        string strVocabularyName = mStrVocabularyFilePath.substr(found+1);

        if(type == TEXT_FILE) // File text
        {
            cout << "Starting to write the save text file " << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::text_oarchive oa(ofs);

            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            cout << "End to write the save text file" << endl;
        }
        else if(type == BINARY_FILE) // File binary
        {
            cout << "Starting to write the save binary file" << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            cout << "End to write save binary file" << endl;
        }
    }
}

bool System::LoadAtlas(int type)
{
    string strFileVoc, strVocChecksum;
    bool isRead = false;

    // 兼容相对 stem、显式 .osa 文件名和绝对路径
    string pathLoadFileName = mStrLoadAtlasFromFile;
    if(pathLoadFileName.size() < 4 || pathLoadFileName.substr(pathLoadFileName.size()-4) != ".osa")
        pathLoadFileName += ".osa";
    if(!pathLoadFileName.empty() && pathLoadFileName[0] != '/')
        pathLoadFileName = "./" + pathLoadFileName;

    if(type == TEXT_FILE) // File text
    {
        cout << "Starting to read the save text file " << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        cout << "End to load the save text file " << endl;
        isRead = true;
    }
    else if(type == BINARY_FILE) // File binary
    {
        cout << "Starting to read the save binary file"  << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::binary_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        cout << "End to load the save binary file" << endl;
        isRead = true;
    }

    if(isRead)
    {
        //Check if the vocabulary is the same
        string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);

        if(strInputVocabularyChecksum.compare(strVocChecksum) != 0)
        {
            cout << "The vocabulary load isn't the same which the load session was created " << endl;
            cout << "-Vocabulary name: " << strFileVoc << endl;
            return false; // Both are differents
        }

        mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
        mpAtlas->SetORBVocabulary(mpVocabulary);
        mpAtlas->PostLoad();

        return true;
    }
    return false;
}

string System::CalculateCheckSum(string filename, int type)
{
    string checksum = "";

    unsigned char c[MD5_DIGEST_LENGTH];

    std::ios_base::openmode flags = std::ios::in;
    if(type == BINARY_FILE) // Binary file
        flags = std::ios::in | std::ios::binary;

    ifstream f(filename.c_str(), flags);
    if ( !f.is_open() )
    {
        cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
        return checksum;
    }

    MD5_CTX md5Context;
    char buffer[1024];

    MD5_Init (&md5Context);
    while ( int count = f.readsome(buffer, sizeof(buffer)))
    {
        MD5_Update(&md5Context, buffer, count);
    }

    f.close();

    MD5_Final(c, &md5Context );

    for(int i = 0; i < MD5_DIGEST_LENGTH; i++)
    {
        char aux[10];
        sprintf(aux,"%02x", c[i]);
        checksum = checksum + aux;
    }

    return checksum;
}

} //namespace ORB_SLAM
