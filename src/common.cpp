/**
 * @file monocular_mode.cpp
 * @brief ROS 2 node implementation for ORB-SLAM3 Monocular and Monocular-Inertial modes.
 */

#include "ros2_orb_slam3/common.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdlib>
#include <filesystem>
#include <limits>
#include <vector>

namespace
{

/**
 * @brief Resolves the sharing directory path of the ros2_orb_slam3 package.
 * @param logger The ROS logger instance for outputting warnings/errors.
 * @return Absolute path string to the package root directory.
 */
std::string resolvePackagePath(const rclcpp::Logger &logger)
{
    try
    {
        return ament_index_cpp::get_package_share_directory("ros2_orb_slam3");
    }
    catch (const std::exception &ex)
    {
        const char *envOverride = std::getenv("ROS2_ORB_SLAM3_PATH");
        if (envOverride != nullptr)
        {
            RCLCPP_INFO(logger, "Using ROS2_ORB_SLAM3_PATH override: %s", envOverride);
            return std::string(envOverride);
        }

        const char *homeEnv = std::getenv("HOME");
        std::filesystem::path fallback = homeEnv != nullptr ?
            (std::filesystem::path(homeEnv) / "ros2_ws" / "src" / "ros2_orb_slam3") :
            std::filesystem::path("ros2_orb_slam3");

        RCLCPP_WARN(logger,
                    "Could not resolve share directory via ament_index_cpp (%s). Falling back to %s",
                    ex.what(), fallback.string().c_str());
        return fallback.string();
    }
}

bool hasSnapPath(const char *value)
{
    return value != nullptr && std::string(value).find("/snap/") != std::string::npos;
}

void sanitizeSnapGuiEnvironment(const rclcpp::Logger &logger)
{
    // Snap-injected GUI variables can pull incompatible core20 glibc libs at runtime.
    const std::vector<const char *> vars = {
        "GTK_PATH",
        "GTK_PATH_VSCODE_SNAP_ORIG",
        "XDG_DATA_DIRS",
        "XDG_DATA_DIRS_VSCODE_SNAP_ORIG",
    };

    for (const char *name : vars)
    {
        const char *value = std::getenv(name);
        if (hasSnapPath(value))
        {
            RCLCPP_WARN(logger,
                        "Unsetting %s to prevent snap/core runtime conflicts with Pangolin.",
                        name);
            unsetenv(name);
        }
    }
}

}

/**
 * @brief Constructor for the MonocularMode ROS 2 Node.
 */
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    pAgent = nullptr;
    sensorType = ORB_SLAM3::System::MONOCULAR;
    useInertialMeasurements = false;
    hasLastImageTimestamp = false;
    lastImageTimestamp = 0.0;
    timeStep = 0.0;
    
    workerThreadRunning_ = false;
    workerThreadShouldStop_ = false;
    totalFramesProcessed_ = 0;
    totalFramesDropped_ = 0;
    lastMetricsTime_ = std::chrono::high_resolution_clock::now();

    const char *homeCStr = std::getenv("HOME");
    homeDir = homeCStr != nullptr ? homeCStr : "";
    packagePath = resolvePackagePath(this->get_logger());

    settingsFileBasePath = packagePath + "/orb_slam3/config";
    settingsDirMonocular = settingsFileBasePath + "/Monocular/";
    settingsDirMonocularInertial = settingsFileBasePath + "/Monocular-Inertial/";
    
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); 
    this->declare_parameter("voc_file_arg", "file_not_set"); 
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); 
    this->declare_parameter("settings_name_arg", "smartrollerz"); 
    this->declare_parameter("wait_for_handshake_arg", true); 
    this->declare_parameter("use_external_timestep_arg", false); 
    
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "";
    settingsParamOverride = "";

    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsParamOverride = param3.as_string();

    rclcpp::Parameter param4 = this->get_parameter("settings_name_arg");
    defaultSettingsName = param4.as_string();

    rclcpp::Parameter param5 = this->get_parameter("wait_for_handshake_arg");
    waitForHandshake = param5.as_bool();

    rclcpp::Parameter param6 = this->get_parameter("use_external_timestep_arg");
    useExternalTimestep = param6.as_bool();

    if (vocFilePath == "file_not_set" || vocFilePath == "file_path_not_set")
    {
        auto defaultVoc = std::filesystem::path(packagePath) / "orb_slam3" / "Vocabulary" / "ORBvoc.txt.bin";
        if (std::filesystem::exists(defaultVoc))
        {
            vocFilePath = defaultVoc.string();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Default vocabulary file %s missing. Please set voc_file_arg.",
                        defaultVoc.string().c_str());
            vocFilePath.clear();
        }
    }

    if (settingsParamOverride == "file_not_set" || settingsParamOverride == "file_path_not_set")
    {
        settingsParamOverride.clear();
    }

    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    if (!settingsParamOverride.empty())
    {
        RCLCPP_INFO(this->get_logger(), "settings override %s", settingsParamOverride.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "settings_name_arg %s", defaultSettingsName.c_str());
    RCLCPP_INFO(this->get_logger(), "wait_for_handshake_arg %s", waitForHandshake ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "use_external_timestep_arg %s", useExternalTimestep ? "true" : "false");

    subexperimentconfigName = "/mono_py_driver/experiment_settings"; 
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; 
    subImgMsgName = "/mono_py_driver/img_msg"; 
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; 
    subImuMsgName = "/mono_py_driver/imu_msg"; 

    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 50, std::bind(&MonocularMode::Img_callback, this, _1));

    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 50, std::bind(&MonocularMode::Timestep_callback, this, _1));

    subImuMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(subImuMsgName, 200,
        std::bind(&MonocularMode::Imu_callback, this, _1));

    if (waitForHandshake)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),
                    "Direct startup enabled: initializing SLAM with settings_name_arg=%s",
                    defaultSettingsName.c_str());
        initializeVSLAM(defaultSettingsName);
    }
}

/**
 * @brief Destructor for the MonocularMode class, handling safe thread join operations and resource cleanup.
 */
MonocularMode::~MonocularMode()
{   
    if (workerThreadRunning_)
    {
        workerThreadShouldStop_ = true;
        frameQueue_cv_.notify_one();
        if (slamWorkerThread_.joinable())
        {
            slamWorkerThread_.join();
        }
        workerThreadRunning_ = false;
    }
    
    if (pAgent != nullptr)
    {
        pAgent->Shutdown();
        delete pAgent;
        pAgent = nullptr;
    }
    pass;
}

/**
 * @brief Callback function that handles receiving configuration settings strings from the external driver.
 * @param msg The incoming ROS String message containing the configuration identifier.
 */
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    receivedConfig = experimentConfig;
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    initializeVSLAM(experimentConfig);
}

/**
 * @brief Initializes the target core ORB-SLAM3 tracking subsystem using resolved configuration profiles.
 * @param configString Name or path reference to the targeted tracking profile.
 */
void MonocularMode::initializeVSLAM(std::string& configString){

    if (vocFilePath.empty())
    {
        RCLCPP_ERROR(get_logger(), "Vocabulary path is empty. Please provide voc_file_arg.");
        rclcpp::shutdown();
        return;
    }

    auto fileExists = [](const std::string& path) -> bool {
        std::ifstream file(path);
        return file.good();
    };

    std::string resolvedPath;

    if (!settingsParamOverride.empty() && settingsParamOverride.find(".yaml") != std::string::npos)
    {
        if (fileExists(settingsParamOverride))
        {
            resolvedPath = settingsParamOverride;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Konfigurationsdatei %s existiert nicht.", settingsParamOverride.c_str());
            rclcpp::shutdown();
            return;
        }
    }
    else
    {
        std::vector<std::string> lookupDirs;
        if (!settingsParamOverride.empty())
        {
            std::string base = settingsParamOverride;
            if (base.back() != '/')
            {
                base.push_back('/');
            }
            lookupDirs.push_back(base);
        }

        if (!settingsDirMonocular.empty())
        {
            lookupDirs.push_back(settingsDirMonocular);
        }

        if (!settingsDirMonocularInertial.empty())
        {
            lookupDirs.push_back(settingsDirMonocularInertial);
        }

        for (const auto& dir : lookupDirs)
        {
            std::string candidate = dir + configString + ".yaml";
            if (fileExists(candidate))
            {
                resolvedPath = candidate;
                break;
            }
        }
    }

    if (resolvedPath.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Konfiguration %s konnte nicht gefunden werden.", configString.c_str());
        rclcpp::shutdown();
        return;
    }

    settingsFilePath = resolvedPath;
    useInertialMeasurements = settingsFilePath.find("Monocular-Inertial") != std::string::npos;

    if (!useInertialMeasurements)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imuMeasurementBuffer.clear();
    }

    sensorType = useInertialMeasurements ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR;
    enablePangolinWindow = true; 
    enableOpenCVWindow = true; 

    if (enablePangolinWindow)
    {
        sanitizeSnapGuiEnvironment(this->get_logger());
    }

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Aktiver Sensormodus: %s", useInertialMeasurements ? "Monocular-Inertial" : "Monocular");

    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    hasLastImageTimestamp = false;
    lastImageTimestamp = 0.0;
    
    workerThreadShouldStop_ = false;
    totalFramesProcessed_ = 0;
    totalFramesDropped_ = 0;
    lastMetricsTime_ = std::chrono::high_resolution_clock::now();
    slamWorkerThread_ = std::thread(&MonocularMode::RunSlamProcessing, this);
    
    std::cout << "MonocularMode node initialized" << std::endl; 
}

/**
 * @brief Callback function that receives and tracks external execution timestamps.
 * @param time_msg Incoming Float64 message object holding the synchronized timestamp value.
 */
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    timeStep = time_msg.data;
}

/**
 * @brief Callback function that ingests raw image feeds, associates active sensor buffers, and enqueues tasks.
 * @param msg Incoming Image frame matrix structure.
 */
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    if (pAgent == nullptr)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "SLAM-System ist noch nicht initialisiert. Bild wird ignoriert.");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr; 
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg); 
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image: %s", e.what());
        return;
    }

    const double headerTimestamp = rclcpp::Time(msg.header.stamp).seconds();
    const bool hasHeaderTimestamp = headerTimestamp > 0.0;
    const bool hasExternalTimestamp = timeStep > 0.0;
    const double frameTimestamp =
        (useExternalTimestep && hasExternalTimestamp) ? timeStep :
        (hasHeaderTimestamp ? headerTimestamp : timeStep);

    static size_t image_counter = 0;
    image_counter++;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Img callback aktiv: count=%zu topic_encoding=%s size=%ux%u frame_ts=%.6f (header=%.6f ext=%.6f)",
        image_counter,
        msg.encoding.c_str(),
        msg.width,
        msg.height,
        frameTimestamp,
        headerTimestamp,
        timeStep
    );
    
    std::vector<ORB_SLAM3::IMU::Point> imuMeasurements;
    if (useInertialMeasurements)
    {
        imuMeasurements.reserve(512);  
        const double lowerBound = hasLastImageTimestamp ? lastImageTimestamp : std::numeric_limits<double>::lowest();
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            while (!imuMeasurementBuffer.empty() && imuMeasurementBuffer.front().t <= frameTimestamp)
            {
                const ORB_SLAM3::IMU::Point& sample = imuMeasurementBuffer.front();
                if (sample.t > lowerBound)
                {
                    imuMeasurements.emplace_back(sample);
                }
                imuMeasurementBuffer.pop_front();
            }
        }

        lastImageTimestamp = frameTimestamp;
        hasLastImageTimestamp = true;
    }
    else
    {
        lastImageTimestamp = frameTimestamp;
        hasLastImageTimestamp = true;
    }

    FrameData frameData;
    frameData.image = cv_ptr->image.clone();  
    frameData.timestamp = frameTimestamp;
    frameData.imuMeasurements = std::move(imuMeasurements);
    
    {
        std::unique_lock<std::mutex> lock(frameQueue_mutex_);
        
        if (frameQueue_.size() >= frameQueueMaxSize_)
        {
            frameQueue_.pop();  
            totalFramesDropped_++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Frame queue full, dropping oldest frame. Total dropped: %zu", totalFramesDropped_);
        }
        
        frameQueue_.push(frameData);
    }
    
    frameQueue_cv_.notify_one();
}

/**
 * @brief Thread execution worker loop responsible for extracting and feeding frames asynchronously to SLAM.
 */
void MonocularMode::RunSlamProcessing()
{
    workerThreadRunning_ = true;
    RCLCPP_INFO(this->get_logger(), "SLAM processing worker thread started");
    
    while (!workerThreadShouldStop_)
    {
        FrameData frameData;
        
        {
            std::unique_lock<std::mutex> lock(frameQueue_mutex_);
            
            frameQueue_cv_.wait_for(lock, std::chrono::milliseconds(100), 
                [this]() { return !frameQueue_.empty() || workerThreadShouldStop_; });
            
            if (frameQueue_.empty())
            {
                continue;  
            }
            
            if (workerThreadShouldStop_)
            {
                break;  
            }
            
            frameData = frameQueue_.front();
            frameQueue_.pop();
        }
        
        if (pAgent != nullptr)
        {
            auto startTime = std::chrono::high_resolution_clock::now();
            
            Sophus::SE3f Tcw;
            if (useInertialMeasurements)
            {
                Tcw = pAgent->TrackMonocular(frameData.image, frameData.timestamp, frameData.imuMeasurements);
            }
            else
            {
                Tcw = pAgent->TrackMonocular(frameData.image, frameData.timestamp);
            }
            
            auto endTime = std::chrono::high_resolution_clock::now();
            auto frameDuration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
            
            totalFramesProcessed_++;
            
            if (totalFramesProcessed_ % 50 == 0)
            {
                auto now = std::chrono::high_resolution_clock::now();
                auto timeSinceLastMetrics = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastMetricsTime_).count();
                
                if (timeSinceLastMetrics > 0)
                {
                    double fps = (50.0 / timeSinceLastMetrics) * 1000.0;
                    size_t queueSize = 0;
                    {
                        std::unique_lock<std::mutex> lock(frameQueue_mutex_);
                        queueSize = frameQueue_.size();
                    }
                    
                    RCLCPP_INFO(this->get_logger(),
                        "SLAM Metrics: processed=%zu, fps=%.1f, queue_depth=%zu, dropped=%zu, last_frame_ms=%ld",
                        totalFramesProcessed_, fps, queueSize, totalFramesDropped_, frameDuration);
                    
                    lastMetricsTime_ = now;
                }
            }
        }
    }
    
    workerThreadRunning_ = false;
    RCLCPP_INFO(this->get_logger(), "SLAM processing worker thread stopped");
}

/**
 * @brief Callback function that buffers incoming continuous linear acceleration and angular velocity vector sequences.
 * @param msg The incoming ROS Imu message.
 */
void MonocularMode::Imu_callback(const sensor_msgs::msg::Imu& msg)
{
    double timestamp = rclcpp::Time(msg.header.stamp).seconds();

    cv::Point3f acc(static_cast<float>(msg.linear_acceleration.x),
                    static_cast<float>(msg.linear_acceleration.y),
                    static_cast<float>(msg.linear_acceleration.z));
    cv::Point3f gyro(static_cast<float>(msg.angular_velocity.x),
                     static_cast<float>(msg.angular_velocity.y),
                     static_cast<float>(msg.angular_velocity.z));

    ORB_SLAM3::IMU::Point imuPoint(acc, gyro, timestamp);

    const size_t kMaxBufferSize = 6000;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imuMeasurementBuffer.emplace_back(imuPoint);
        while (imuMeasurementBuffer.size() > kMaxBufferSize)
        {
            imuMeasurementBuffer.pop_front();
        }
    }
}