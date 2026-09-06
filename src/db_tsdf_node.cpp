/*
 * TSDFNode: ROS 2 node for LiDAR->TDF mapping.
 * Subscribes to PointCloud2, looks up the transform via tf2_ros::Buffer,
 * filters, transforms, and integrates the cloud into the TSDF grid.
 * Publishes the filtered, global-frame cloud for visualization.
 */

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <spawn.h>
#include <sys/wait.h>
#include <nlohmann/json.hpp>
#include <db_tsdf/export_worker.hpp>
#include <db_tsdf/provenance.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

// TF2
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

// PCL
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/common/transforms.h>

// DB-TSDF
#include <db_tsdf/tsdf3d.hpp>

#ifndef DB_TSDF_MASK_BITS
#define DB_TSDF_MASK_BITS 16
#endif

#if DB_TSDF_MASK_BITS == 16
using TSDFBackend = TSDF3D16;
#elif DB_TSDF_MASK_BITS == 32
using TSDFBackend = TSDF3D32;
#else
#error "DB_TSDF_MASK_BITS must be 16 or 32"
#endif

using Json = nlohmann::json;
namespace fs = std::filesystem;
extern char **environ;

class TSDFNode : public rclcpp::Node
{
private:
    template<class T> T startupParameter(const std::string& name,const T& value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only=true;
        if constexpr (std::is_same_v<T,int>) {
            const auto result=this->declare_parameter<int64_t>(name,value,descriptor);
            if (result<std::numeric_limits<int>::min() || result>std::numeric_limits<int>::max())
                throw std::invalid_argument(name+" exceeds integer range");
            return static_cast<int>(result);
        } else return this->declare_parameter<T>(name,value,descriptor);
    }
    struct GeoOrigin
    {
        bool configured{false};
        double latitude_deg{0.0};
        double longitude_deg{0.0};
        double ellipsoid_height_m{0.0};
        double map_yaw_rad{0.0};
        std::string altitude_reference{"ellipsoid"};
    };

public:
    TSDFNode(const std::string &node_name)
        : Node(node_name)
    {
        // Parameters
        m_inCloudTopic      = startupParameter<std::string>("in_cloud", "/os_cloud_node/points");
        m_odomFrameId       = startupParameter<std::string>("odom_frame_id", "odom");
        m_fixedFrameId      = startupParameter<std::string>("fixed_frame_id", "");
        m_earthFrameId      = startupParameter<std::string>("earth_frame_id", "earth");
        m_useTf             = startupParameter<bool>("use_tf", true);
        m_useTfTopic        = startupParameter<bool>("use_tf_topic", false);
        m_inTfTopic         = startupParameter<std::string>("in_tf_topic", "/gt_icp/transform");
        m_inGpsTopic        = startupParameter<std::string>("in_gps_topic", "/gps/fix");
        m_outGpsTopic       = startupParameter<std::string>("out_gps_topic", "/current_gps_fix");

        // A map frame is required for a geographically anchored TSDF. Leave
        // fixed_frame_id empty only for legacy local-only deployments, where
        // odom_frame_id remains the transform target.
        if (m_fixedFrameId.empty()) {
            m_fixedFrameId = m_odomFrameId;
            RCLCPP_WARN(this->get_logger(),
                        "fixed_frame_id is unset; using legacy odom_frame_id '%s'. "
                        "Set fixed_frame_id='map' when robot_localization publishes map -> odom.",
                        m_fixedFrameId.c_str());
        }

        m_geoOrigin.configured = startupParameter<bool>("geo_origin_configured", false);
        m_geoOrigin.latitude_deg = startupParameter<double>("geo_origin_latitude", 0.0);
        m_geoOrigin.longitude_deg = startupParameter<double>("geo_origin_longitude", 0.0);
        m_geoOrigin.ellipsoid_height_m = startupParameter<double>("geo_origin_altitude", 0.0);
        m_geoOrigin.map_yaw_rad = startupParameter<double>("geo_origin_yaw", 0.0);
        m_geoOrigin.altitude_reference = startupParameter<std::string>(
            "geo_origin_altitude_reference", "ellipsoid");
        if (m_geoOrigin.configured) {
            validateGeoOrigin(m_geoOrigin);
        }

        m_tdfGridSizeX_low  = startupParameter<double>("tdfGridSizeX_low", -10.0);
        m_tdfGridSizeX_high = startupParameter<double>("tdfGridSizeX_high", 10.0);
        m_tdfGridSizeY_low  = startupParameter<double>("tdfGridSizeY_low", -10.0);
        m_tdfGridSizeY_high = startupParameter<double>("tdfGridSizeY_high", 10.0);
        m_tdfGridSizeZ_low  = startupParameter<double>("tdfGridSizeZ_low", -10.0);
        m_tdfGridSizeZ_high = startupParameter<double>("tdfGridSizeZ_high", 10.0);
        m_tdfGridRes        = startupParameter<double>("tdf_grid_res", 0.10);
        rcl_interfaces::msg::ParameterDescriptor capacity_descriptor;
        capacity_descriptor.read_only=true;
        capacity_descriptor.dynamic_typing=true;  // Accept legacy integer-valued YAML doubles.
        auto capacity=this->declare_parameter("tdf_max_cells",rclcpp::ParameterValue(10000),capacity_descriptor);
        m_tdfMaxCells=capacity.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER ?
            static_cast<double>(capacity.get<int64_t>()) : capacity.get<double>();
        m_minRange          = startupParameter<double>("min_range", 1.0);
        m_maxRange          = startupParameter<double>("max_range", 100.0);
        m_PcDownsampling    = startupParameter<int>("pc_downsampling", 1);
        m_occMinHits        = startupParameter<int>("occ_min_hits", 1);
        m_binsAz            = startupParameter<int>("bins_az", 40);
        m_binsEl            = startupParameter<int>("bins_el", 40);
        m_shadowRadius      = startupParameter<int>("shadow_radius", 6);
        m_distanceMode      = startupParameter<std::string>("distance_mode", "L1");
        m_kernelSize        = startupParameter<int>("kernel_size", 11);
        m_verboseInit       = startupParameter<bool>("verbose_init", false);

        m_colorScheme       = startupParameter<std::string>("color_scheme", "grayscale");
        m_colorBins         = startupParameter<int>("color_bins", 32);

        m_numThreads = startupParameter<int>("num_threads", 0);
        m_capacityPolicy = startupParameter<std::string>("capacity_policy", "stop");
        m_memoryBudget = startupParameter<double>("memory_budget_mb", 0.0);
        m_snapshotBudget = startupParameter<double>("snapshot_budget_mb", 0.0);
        m_sensorFrame = startupParameter<std::string>("sensor_frame", "");
        m_publishCloud = startupParameter<bool>("publish_cloud", true);
        m_legacyAllowNearest = startupParameter<bool>("legacy_allow_nearest", true);
        m_allowLatestTf = startupParameter<bool>("allow_latest_tf", false);
        m_tfTimeout = startupParameter<double>("tf_timeout", 0.1);
        const auto legacySkew=startupParameter<double>("legacy_max_skew",0.1);
        if (!std::isfinite(legacySkew) || legacySkew<0 || legacySkew>10)
            throw std::invalid_argument("legacy_max_skew must be finite and between 0 and 10 seconds");
        m_maxSkew = rclcpp::Duration::from_seconds(legacySkew);
        m_outputDirectory = fs::absolute(startupParameter<std::string>("output_directory", "maps"));
        m_initialCheckpoint = startupParameter<std::string>("initial_checkpoint", "");
        const auto meshMode = startupParameter<std::string>("mesh_mode", "occupancy");
        const double meshSigma = startupParameter<double>("mesh_smoothing_sigma", -1.0);
        const auto reliability = startupParameter<std::string>("cloud_reliability", "best_effort");
        const int queueDepth = startupParameter<int>("cloud_queue_depth", 10);
        if (m_PcDownsampling < 1 || !std::isfinite(m_minRange) || !std::isfinite(m_maxRange) ||
            m_minRange < 0 || m_maxRange <= m_minRange)
            throw std::invalid_argument("pc_downsampling must be positive and ranges finite, nonnegative, ordered");
        if (!std::isfinite(m_tdfMaxCells) || m_tdfMaxCells < 1 ||
            m_tdfMaxCells > std::numeric_limits<int>::max() || std::floor(m_tdfMaxCells)!=m_tdfMaxCells)
            throw std::invalid_argument("tdf_max_cells must be a positive integer");
        if (m_colorBins < 1 || m_colorBins > 4096 || normalizeColorScheme(m_colorScheme)!=m_colorScheme)
            throw std::invalid_argument("invalid color_scheme or color_bins (must be 1..4096)");
        if (queueDepth<1 || queueDepth>100000 || (reliability!="reliable" && reliability!="best_effort"))
            throw std::invalid_argument("cloud_queue_depth must be 1..100000; cloud_reliability reliable or best_effort");
        if (!std::isfinite(m_snapshotBudget) || m_snapshotBudget<0 ||
            !std::isfinite(m_tfTimeout) || m_tfTimeout<0 || m_tfTimeout>10 || m_maxSkew.nanoseconds()<0)
            throw std::invalid_argument("invalid snapshot budget or TF timing limits");
        if (m_sensorFrame.size() && (!m_useTf || m_useTfTopic))
            throw std::invalid_argument("sensor_frame requires generic TF mode");
        fs::create_directories(m_outputDirectory);
        const auto probe=m_outputDirectory/(".write_probe_"+std::to_string(getpid()));
        { std::ofstream out(probe); out.exceptions(std::ios::badbit|std::ios::failbit); out<<"ok"; out.close(); }
        fs::remove(probe);

        RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Initializing DB-TSDF Node with Parameters:");
        RCLCPP_INFO(this->get_logger(), "  Distance Mask:   %d-bit", DB_TSDF_MASK_BITS);
        RCLCPP_INFO(this->get_logger(), " ");

        RCLCPP_INFO(this->get_logger(), "  Grid Params:");
        RCLCPP_INFO(this->get_logger(), "    Resolution:     %.3f m", m_tdfGridRes);
        RCLCPP_INFO(this->get_logger(), "    Max Cells:      %d", (int)m_tdfMaxCells);

        RCLCPP_INFO(this->get_logger(), "  Kernel Params:");
        RCLCPP_INFO(this->get_logger(), "    Kernel Size:    %dx%dx%d", m_kernelSize, m_kernelSize, m_kernelSize);
        RCLCPP_INFO(this->get_logger(), "    Occ. Min. Hits: %d", m_occMinHits);
        RCLCPP_INFO(this->get_logger(), "    Shadow Radius:  %d voxels", m_shadowRadius);
        RCLCPP_INFO(this->get_logger(), "    Distance Mode:  %s", m_distanceMode.c_str());

        RCLCPP_INFO(this->get_logger(), "  Filtering Params:");
        RCLCPP_INFO(this->get_logger(), "    Downsampling:   1 in every %d points", m_PcDownsampling);
        RCLCPP_INFO(this->get_logger(), "    Range (Min/Max):  %.1f m / %.1f m", m_minRange, m_maxRange);

        RCLCPP_INFO(this->get_logger(), "  ATAK Export Params:");
        RCLCPP_INFO(this->get_logger(), "    Color Scheme:   %s", m_colorScheme.c_str());
        RCLCPP_INFO(this->get_logger(), "    Color Bins:     %d", m_colorBins);
        RCLCPP_INFO(this->get_logger(), "    Fixed Frame:    %s", m_fixedFrameId.c_str());
        RCLCPP_INFO(this->get_logger(), "    Geo Origin:     %s", m_geoOrigin.configured ? "configured" : "not configured");

        RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");

        // TF buffer and listener
        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

        // Publishers
        m_cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());
        m_mapPub=create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud",rclcpp::QoS(1).reliable().transient_local());
        m_gpsPub = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            m_outGpsTopic, rclcpp::QoS(1).reliable().transient_local());

        // Subscriptions
        auto input_qos = rclcpp::QoS(rclcpp::KeepLast(queueDepth));
        if (reliability=="best_effort") input_qos.best_effort();
        else input_qos.reliable();

        m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudTopic,
            input_qos,
            std::bind(&TSDFNode::pointcloudCallback, this, std::placeholders::_1));

        if (m_useTf && m_useTfTopic) {
            RCLCPP_INFO(this->get_logger(), "Using Legacy TF Mode: subscribing to '%s'", m_inTfTopic.c_str());
            m_tfSub = this->create_subscription<geometry_msgs::msg::TransformStamped>(
                m_inTfTopic,
                input_qos,
                std::bind(&TSDFNode::tfCallback, this, std::placeholders::_1));
        } else if (m_useTf) {
            RCLCPP_INFO(this->get_logger(), "Using Generic TF Mode (listening to /tf)");
        }

        // GPS subscription for ATAK-ROS2 geo-referencing
        m_gpsSub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            m_inGpsTopic,
            rclcpp::SensorDataQoS(),
            std::bind(&TSDFNode::gpsCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "Subscribing to GPS topic '%s'; publishing valid fixes on '%s'",
                    m_inGpsTopic.c_str(), m_outGpsTopic.c_str());

        save_service_pcd_ = this->create_service<std_srvs::srv::Trigger>("save_grid_pcd",
            std::bind(&TSDFNode::saveGridPCD, this, std::placeholders::_1, std::placeholders::_2));

        save_service_ply_ = this->create_service<std_srvs::srv::Trigger>("save_grid_ply",
            std::bind(&TSDFNode::saveGridPLY, this, std::placeholders::_1, std::placeholders::_2));

        save_service_csv_ = this->create_service<std_srvs::srv::Trigger>("save_grid_csv",
            std::bind(&TSDFNode::saveGridCSV, this, std::placeholders::_1, std::placeholders::_2));

        save_service_mesh_ = this->create_service<std_srvs::srv::Trigger>("save_grid_mesh",
            std::bind(&TSDFNode::saveGridMesh, this, std::placeholders::_1, std::placeholders::_2));

        save_service_atak_ = this->create_service<std_srvs::srv::Trigger>("save_grid_atak_zip",
            std::bind(&TSDFNode::saveGridAtakZip, this, std::placeholders::_1, std::placeholders::_2));

        get_geo_origin_service_ = this->create_service<std_srvs::srv::Trigger>("get_geo_origin",
            std::bind(&TSDFNode::getGeoOrigin, this, std::placeholders::_1, std::placeholders::_2));

        // TDF grid allocation
        m_grid3d.setup(m_tdfGridSizeX_low, m_tdfGridSizeX_high,
                       m_tdfGridSizeY_low, m_tdfGridSizeY_high,
                       m_tdfGridSizeZ_low, m_tdfGridSizeZ_high,
                       m_tdfGridRes,
                       m_kernelSize,
                       m_occMinHits,
                       m_binsAz,
                       m_binsEl,
                       m_shadowRadius,
                       m_distanceMode,
                       m_tdfMaxCells,
                       m_verboseInit, m_numThreads, m_capacityPolicy, m_memoryBudget);
        m_grid3d.configureMesh(meshMode,meshSigma);
        if (!m_initialCheckpoint.empty()) m_grid3d.loadCheckpoint(m_initialCheckpoint,frameSignature());
        m_exports=std::make_unique<db_tsdf::ExportWorker>(2);
        m_statusPublisher=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());
        m_statusService=create_service<std_srvs::srv::Trigger>("get_status",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
            std::lock_guard lock(m_gridMutex);
            response->success=true; response->message=metadataLocked().dump();
        });
        m_exportStatusService=create_service<std_srvs::srv::Trigger>("export_status",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
            Json jobs=Json::array();
            for (const auto& job : m_exports->statuses())
                jobs.push_back({{"id",job.id},{"path",job.path},{"state",job.state},{"error",job.error}});
            response->success=true; response->message=jobs.dump();
        });
        m_resetService=create_service<std_srvs::srv::Trigger>("reset_map",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
            std::lock_guard lock(m_gridMutex);
            m_grid3d.clear(); m_integrated=0; m_lastStamp=0; ++m_epoch;
            m_received=0; m_tfFailures=0; m_invalidClouds=0; m_invalidPoints=0;
            m_outsidePoints=0; m_skippedBlocks=0; m_rangeFiltered=0; m_subsampled=0;
            m_legacyNearest=0; m_invalidTransforms=0; m_lastCallbackMs=0;
            response->success=true; response->message="Map cleared; existing export snapshots are unchanged.";
        });
        m_checkpointService=create_service<std_srvs::srv::Trigger>("save_grid_checkpoint",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
            queueExport("checkpoint",[this](auto& map,const fs::path& dir) {
                map.saveCheckpoint((dir/"map.dbtsdf").string(),frameSignature());
            },response);
        });
        m_previewService=create_service<std_srvs::srv::Trigger>("publish_map",[this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
            const auto id=m_exports->enqueue("map_cloud",[this] {
                Json metadata;
                auto snapshot=takeSnapshot(metadata);
                sensor_msgs::msg::PointCloud2 cloud;
                pcl::toROSMsg(snapshot->grid().surfaceCloud(),cloud);
                cloud.header.frame_id=m_fixedFrameId;
                cloud.header.stamp=rclcpp::Time(metadata.at("last_stamp_ns").get<int64_t>(),RCL_ROS_TIME);
                m_mapPub->publish(cloud);
            });
            response->success=id!=0;
            response->message=Json({{"job_id",id},{"state",id?"queued":"busy"}}).dump();
        });
        m_statusTimer=create_wall_timer(std::chrono::seconds(1),[this] {
            std_msgs::msg::String message;
            { std::lock_guard lock(m_gridMutex); message.data=metadataLocked().dump(); }
            m_statusPublisher->publish(message);
        });
        RCLCPP_INFO(get_logger(),"Grid capacity: %.1f MiB; initial allocation: %.1f MiB; workers: %d; policy: %s",
            m_grid3d.grid().estimatedBytes()/1048576.0,m_grid3d.grid().allocatedBytes()/1048576.0,
            m_grid3d.threadCount(),m_capacityPolicy.c_str());

        RCLCPP_INFO(this->get_logger(),
            "DB-TSDF is ready! Grid: %.1f x %.1f x %.1f m @ %.3f m/voxel",
            fabs(m_tdfGridSizeX_high - m_tdfGridSizeX_low),
            fabs(m_tdfGridSizeY_high - m_tdfGridSizeY_low),
            fabs(m_tdfGridSizeZ_high - m_tdfGridSizeZ_low),
            m_tdfGridRes);
    }

    ~TSDFNode() override {
        if (m_exports) m_exports->shutdown();
        RCLCPP_INFO(this->get_logger(), "Node closed successfully.");
    }

private:
    static bool isValidFix(const sensor_msgs::msg::NavSatFix &fix)
    {
        return fix.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
               std::isfinite(fix.latitude) && std::isfinite(fix.longitude) &&
               std::isfinite(fix.altitude) &&
               fix.latitude >= -90.0 && fix.latitude <= 90.0 &&
               fix.longitude >= -180.0 && fix.longitude <= 180.0 &&
               std::all_of(fix.position_covariance.begin(),fix.position_covariance.end(),
                   [](double v){return std::isfinite(v);});
    }

    static void validateGeoOrigin(const GeoOrigin &origin)
    {
        if (!std::isfinite(origin.latitude_deg) || !std::isfinite(origin.longitude_deg) ||
            !std::isfinite(origin.ellipsoid_height_m) || !std::isfinite(origin.map_yaw_rad) ||
            origin.latitude_deg < -90.0 || origin.latitude_deg > 90.0 ||
            origin.longitude_deg < -180.0 || origin.longitude_deg > 180.0) {
            throw std::invalid_argument(
                "geo_origin_configured requires finite WGS-84 latitude/longitude, "
                "ellipsoid altitude, and map yaw");
        }
        if (origin.altitude_reference != "ellipsoid") {
            throw std::invalid_argument(
                "geo_origin_altitude_reference must be 'ellipsoid'; MSL heights require a geoid conversion");
        }
    }

    static std::string jsonEscape(const std::string &value)
    {
        std::ostringstream escaped;
        for (const unsigned char c : value) {
            switch (c) {
                case '\\': escaped << "\\\\"; break;
                case '"':  escaped << "\\\""; break;
                case '\n': escaped << "\\n"; break;
                case '\r': escaped << "\\r"; break;
                case '\t': escaped << "\\t"; break;
                default:
                    if (c < 0x20) {
                        escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                                << static_cast<int>(c) << std::dec << std::setfill(' ');
                    } else {
                        escaped << c;
                    }
            }
        }
        return escaped.str();
    }

    std::optional<sensor_msgs::msg::NavSatFix> latestGpsSnapshot() const
    {
        std::lock_guard<std::mutex> lock(m_gpsMutex);
        if (!m_hasLatestGps) {
            return std::nullopt;
        }
        return m_latestGps;
    }

    std::string geoReferenceJson(const std::optional<sensor_msgs::msg::NavSatFix> &latest_fix) const
    {
        std::ostringstream json;
        json << std::fixed << std::setprecision(8)
             << "{\n"
             << "  \"schema\": \"db-tsdf-georef/v1\",\n"
             << "  \"coordinate_system\": \"WGS84_ENU\",\n"
             << "  \"earth_frame_id\": \"" << jsonEscape(m_earthFrameId) << "\",\n"
             << "  \"map_frame_id\": \"" << jsonEscape(m_fixedFrameId) << "\",\n"
             << "  \"origin\": {\n"
             << "    \"latitude_deg\": " << m_geoOrigin.latitude_deg << ",\n"
             << "    \"longitude_deg\": " << m_geoOrigin.longitude_deg << ",\n"
             << "    \"ellipsoid_height_m\": " << m_geoOrigin.ellipsoid_height_m << ",\n"
             << "    \"map_x_axis_yaw_from_east_rad\": " << m_geoOrigin.map_yaw_rad << ",\n"
             << "    \"altitude_reference\": \"" << jsonEscape(m_geoOrigin.altitude_reference) << "\"\n"
             << "  },\n"
             << "  \"latest_valid_fix\": ";

        if (!latest_fix) {
            json << "null\n";
        } else {
            const auto &fix = *latest_fix;
            json << "{\n"
                 << "    \"latitude_deg\": " << fix.latitude << ",\n"
                 << "    \"longitude_deg\": " << fix.longitude << ",\n"
                 << "    \"ellipsoid_height_m\": " << fix.altitude << ",\n"
                 << "    \"status\": " << static_cast<int>(fix.status.status) << ",\n"
                 << "    \"position_covariance_type\": "
                 << static_cast<int>(fix.position_covariance_type) << ",\n"
                 << "    \"position_covariance_m2\": ["
                 << fix.position_covariance[0] << ", " << fix.position_covariance[1] << ", "
                 << fix.position_covariance[2] << ", " << fix.position_covariance[3] << ", "
                 << fix.position_covariance[4] << ", " << fix.position_covariance[5] << ", "
                 << fix.position_covariance[6] << ", " << fix.position_covariance[7] << ", "
                 << fix.position_covariance[8] << "],\n"
                 << "    \"timestamp_sec\": " << fix.header.stamp.sec << ",\n"
                 << "    \"timestamp_nanosec\": " << fix.header.stamp.nanosec << "\n"
                 << "  }\n";
        }
        json << "}";
        return json.str();
    }

    std::string frameSignature() const {
        return m_fixedFrameId+"\n"+(m_geoOrigin.configured ? geoReferenceJson(std::nullopt) : "local");
    }
    Json metadataLocked() const {
        Json parameters=Json::object();
        for (const auto& name : list_parameters({},10).names) {
            const auto p=get_parameter(name);
            switch (p.get_type()) {
                case rclcpp::ParameterType::PARAMETER_BOOL: parameters[name]=p.as_bool(); break;
                case rclcpp::ParameterType::PARAMETER_INTEGER: parameters[name]=p.as_int(); break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE: parameters[name]=p.as_double(); break;
                case rclcpp::ParameterType::PARAMETER_STRING: parameters[name]=p.as_string(); break;
                default: parameters[name]=p.value_to_string();
            }
        }
        const auto& grid=m_grid3d.grid();
        return {{"schema","db-tsdf-run/v1"},{"ready",true},{"mask_bits",DB_TSDF_MASK_BITS},
            {"source_sha256",DB_TSDF_SOURCE_SHA256},{"git_revision",DB_TSDF_GIT_REVISION},
            {"compiler",__VERSION__},{"build_type",DB_TSDF_BUILD_TYPE},{"build",db_tsdf::buildInfo()},
            {"fixed_frame",m_fixedFrameId},{"input_topic",m_pcSub->get_topic_name()},{"map_epoch",m_epoch},
            {"parameters",parameters},{"num_threads",m_grid3d.threadCount()},
            {"received_frames",m_received.load()},{"integrated_frames",m_integrated},
            {"tf_failures",m_tfFailures.load()},{"invalid_clouds",m_invalidClouds.load()},
            {"invalid_points",m_invalidPoints.load()},{"outside_points",m_outsidePoints},
            {"range_filtered_points",m_rangeFiltered.load()},{"subsampled_points",m_subsampled.load()},
            {"invalid_transforms",m_invalidTransforms.load()},{"legacy_nearest_frames",m_legacyNearest.load()},
            {"skipped_blocks",m_skippedBlocks},{"last_stamp_ns",m_lastStamp},
            {"last_callback_ms",m_lastCallbackMs.load()},
            {"allocated_blocks",grid.activeCount()},{"capacity_blocks",grid.capacity()},
            {"allocated_bytes",grid.allocatedBytes()},{"capacity_bytes",grid.estimatedBytes()},
            {"evictions",grid.evictions()},{"rejected_blocks",grid.rejectedBlocks()}};
    }
    std::unique_ptr<TSDFBackend> takeSnapshot(Json& metadata) {
        std::lock_guard lock(m_gridMutex);
        if (m_snapshotBudget>0 && m_grid3d.grid().allocatedBytes()/1048576.0>m_snapshotBudget)
            throw std::runtime_error("export snapshot exceeds snapshot_budget_mb");
        auto snapshot=std::make_unique<TSDFBackend>(m_grid3d);
        metadata=metadataLocked();
        return snapshot;
    }
    void queueExport(const std::string& label,
                     std::function<void(TSDFBackend&,const fs::path&)> action,
                     const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        const auto stamp=std::chrono::system_clock::now().time_since_epoch().count();
        const auto path=m_outputDirectory/(label+"_"+std::to_string(stamp)+"_"+std::to_string(++m_exportCounter));
        const auto id=m_exports->enqueue(path.string(),[this,path,action=std::move(action)] {
            Json metadata;
            auto snapshot=takeSnapshot(metadata);
            metadata["state_hash"]=snapshot->grid().stateHash();
            const auto staging=fs::path(path.string()+".partial");
            if (!fs::create_directory(staging)) throw std::runtime_error("export staging path already exists");
            try {
                action(*snapshot,staging);
                metadata["outputs_sha256"]=db_tsdf::fileChecksums(staging);
                std::ofstream out(staging/"run.json");
                out.exceptions(std::ios::badbit|std::ios::failbit);
                out << metadata.dump(2) << '\n'; out.close();
                fs::rename(staging,path);
                RCLCPP_INFO(get_logger(),"Export complete: %s",path.c_str());
            } catch (...) {
                std::error_code error; fs::remove_all(staging,error); throw;
            }
        });
        response->success=id!=0;
        response->message=Json({{"job_id",id},{"path",path.string()},
            {"state",id ? "queued" : "busy"}}).dump();
    }
    static void zipFiles(const fs::path& directory) {
        std::vector<std::string> args{"zip","-j","-q",(directory/"atak_mesh.zip").string()};
        for (const char* file : {"atak_mesh.obj","atak_mesh.mtl","atak_mesh.origin.json"})
            args.push_back((directory/file).string());
        std::vector<char*> argv;
        for (auto& arg : args) argv.push_back(arg.data());
        argv.push_back(nullptr);
        pid_t pid;
        const int error=posix_spawnp(&pid,"zip",nullptr,nullptr,argv.data(),environ);
        if (error) throw std::runtime_error("could not start zip: "+std::to_string(error));
        int status;
        while (waitpid(pid,&status,0)<0) {
            if (errno!=EINTR) throw std::runtime_error("could not wait for zip");
        }
        if (!WIFEXITED(status) || WEXITSTATUS(status)!=0) throw std::runtime_error("zip export failed");
    }

    static std::string normalizeColorScheme(std::string scheme)
    {
        std::transform(scheme.begin(), scheme.end(), scheme.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        if (scheme == "grayscale" || scheme == "warm" || scheme == "cool" || scheme == "jet") {
            return scheme;
        }
        return "grayscale";
    }

    // Parameters
    std::string m_inCloudTopic;
    std::string m_odomFrameId;
    std::string m_fixedFrameId;
    std::string m_earthFrameId;
    bool m_useTf{true};
    double m_minRange, m_maxRange;
    int m_PcDownsampling;
    int m_kernelSize;
    bool m_verboseInit{false};
    int m_occMinHits;
    int m_binsAz;
    int m_binsEl;
    int m_shadowRadius;
    std::string m_distanceMode;
    std::string m_inTfTopic;
    std::string m_inGpsTopic;
    std::string m_outGpsTopic;
    bool m_useTfTopic{false};

    sensor_msgs::msg::NavSatFix m_latestGps;
    bool m_hasLatestGps{false};
    GeoOrigin m_geoOrigin;
    mutable std::mutex m_gpsMutex;

    std::string m_colorScheme;
    int m_colorBins;

    // TDF grid and geometry
    TSDFBackend m_grid3d;
    double m_tdfGridSizeX_low, m_tdfGridSizeX_high,
           m_tdfGridSizeY_low, m_tdfGridSizeY_high,
           m_tdfGridSizeZ_low, m_tdfGridSizeZ_high,
           m_tdfGridRes, m_tdfMaxCells;
    std::mutex m_gridMutex;

    // TF data
    std::deque<geometry_msgs::msg::TransformStamped> m_tfHist;
    rclcpp::Duration m_maxSkew{0, 100'000'000};
    std::mutex m_tfMutex;
    std::unique_ptr<db_tsdf::ExportWorker> m_exports;
    int m_numThreads{0};
    std::string m_capacityPolicy,m_sensorFrame,m_initialCheckpoint;
    fs::path m_outputDirectory;
    double m_memoryBudget{0},m_snapshotBudget{0},m_tfTimeout{0.1};
    bool m_publishCloud{true},m_allowLatestTf{false},m_legacyAllowNearest{true};
    std::atomic<uint64_t> m_received{0},m_tfFailures{0},m_invalidClouds{0},m_invalidPoints{0};
    std::atomic<uint64_t> m_rangeFiltered{0},m_subsampled{0},m_legacyNearest{0},m_invalidTransforms{0};
    std::atomic<double> m_lastCallbackMs{0};
    uint64_t m_integrated{0},m_outsidePoints{0},m_skippedBlocks{0},m_exportCounter{0},m_epoch{0};
    int64_t m_lastStamp{0};
    pcl::PointCloud<pcl::PointXYZ> m_pclInput,m_pclFiltered,m_pclOutput;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_statusPublisher;
    rclcpp::TimerBase::SharedPtr m_statusTimer;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_statusService,m_exportStatusService,m_resetService,m_checkpointService,m_previewService;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudPub,m_mapPub;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m_tfSub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gpsSub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_gpsPub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_pcd_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_ply_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_csv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_mesh_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_atak_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_geo_origin_service_;

    // TF management
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

    // Callbacks and helpers
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
    void tfCallback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg);
    void gpsCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
    Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped);

    void saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridPLY(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridMesh(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveGridAtakZip(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void getGeoOrigin(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

void TSDFNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
{
    auto start = std::chrono::steady_clock::now();
    ++m_received;
    if (cloud->width==0 || cloud->height==0 || cloud->header.stamp.sec<0 || cloud->header.stamp.nanosec>=1000000000) {
        ++m_invalidClouds; return;
    }
    for (const char* name : {"x","y","z"}) {
        const auto field=std::find_if(cloud->fields.begin(),cloud->fields.end(),
            [&](const auto& f){return f.name==name;});
        if (field==cloud->fields.end() || field->datatype!=sensor_msgs::msg::PointField::FLOAT32 ||
            field->count!=1 || static_cast<uint64_t>(field->offset)+4>cloud->point_step) {
            ++m_invalidClouds;
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Input must contain float32 XYZ fields");
            return;
        }
    }
    if (cloud->is_bigendian || static_cast<uint64_t>(cloud->width)*cloud->point_step>cloud->row_step ||
        static_cast<uint64_t>(cloud->height)*cloud->row_step>cloud->data.size()) {
        ++m_invalidClouds; return;
    }

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    rclcpp::Time t_query = cloud->header.stamp;
    if (m_useTf && t_query.nanoseconds() == 0) {
        if (!m_allowLatestTf) {
            ++m_tfFailures;
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Zero cloud stamp rejected; timestamp the cloud or explicitly allow_latest_tf");
            return;
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Cloud timestamp is 0. Using latest available transform (rclcpp::Time(0)).");
        t_query = rclcpp::Time(0,0,RCL_ROS_TIME);
    }

    if (m_useTf) {
        if (m_useTfTopic) {
            std::lock_guard<std::mutex> lock(m_tfMutex);
            if (m_tfHist.empty()) {
                ++m_tfFailures;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "No TF received yet on LEGACY topic %s -> discarding cloud",
                                     m_inTfTopic.c_str());
                return;
            }

            try {
                const auto stamp=t_query.nanoseconds();
                auto after=std::lower_bound(m_tfHist.begin(),m_tfHist.end(),stamp,[](const auto& tf,int64_t t) {
                    return rclcpp::Time(tf.header.stamp).nanoseconds()<t;
                });
                if (stamp==0 && m_allowLatestTf) T=getTransformMatrix(m_tfHist.back());
                else if (after!=m_tfHist.end() && rclcpp::Time(after->header.stamp).nanoseconds()==stamp)
                    T=getTransformMatrix(*after);
                else if (after!=m_tfHist.begin() && after!=m_tfHist.end()) {
                    const auto before=std::prev(after);
                    const auto t0=rclcpp::Time(before->header.stamp).nanoseconds();
                    const auto t1=rclcpp::Time(after->header.stamp).nanoseconds();
                    if (stamp-t0>m_maxSkew.nanoseconds() || t1-stamp>m_maxSkew.nanoseconds())
                        throw std::runtime_error("legacy TF interpolation gap exceeds legacy_max_skew");
                    const float alpha=double(stamp-t0)/double(t1-t0);
                    const auto a=getTransformMatrix(*before),b=getTransformMatrix(*after);
                    T.block<3,1>(0,3)=(1-alpha)*a.block<3,1>(0,3)+alpha*b.block<3,1>(0,3);
                    const Eigen::Quaternionf qa(a.block<3,3>(0,0)),qb(b.block<3,3>(0,0));
                    T.block<3,3>(0,0)=qa.slerp(alpha,qb).normalized().toRotationMatrix();
                } else {
                    const auto& nearest=after==m_tfHist.end()?m_tfHist.back():*after;
                    if (!m_legacyAllowNearest || std::llabs(stamp-rclcpp::Time(nearest.header.stamp).nanoseconds())>m_maxSkew.nanoseconds())
                        throw std::runtime_error("no timestamp-matched legacy TF available");
                    T=getTransformMatrix(nearest); ++m_legacyNearest;
                }
            } catch (const std::exception& error) {
                ++m_tfFailures;
                RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Legacy TF rejected: %s",error.what());
                return;
            }

        } else {
            try {
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg = m_tfBuffer->lookupTransform(
                    m_fixedFrameId,
                    cloud->header.frame_id,
                    t_query,
                    rclcpp::Duration::from_seconds(m_tfTimeout)
                );
                T = getTransformMatrix(tf_msg);
            } catch (const std::exception &ex) {
                ++m_tfFailures;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Could not transform %s to fixed frame %s: %s",
                    cloud->header.frame_id.c_str(), m_fixedFrameId.c_str(), ex.what()
                );
                return;
            }
        }
    }

    Eigen::Vector3f sensor_position=T.block<3,1>(0,3);
    if (!m_sensorFrame.empty()) {
        try {
            const auto sensor_tf=m_tfBuffer->lookupTransform(m_fixedFrameId,m_sensorFrame,t_query,
                rclcpp::Duration::from_seconds(m_tfTimeout));
            sensor_position=getTransformMatrix(sensor_tf).block<3,1>(0,3);
        } catch(const std::exception& e) {
            ++m_tfFailures;
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Missing sensor origin TF: %s",e.what());
            return;
        }
    }
    const Eigen::Vector3f sensor_in_cloud=T.block<3,3>(0,0).transpose()*(sensor_position-T.block<3,1>(0,3));
    pcl::fromROSMsg(*cloud,m_pclInput);
    m_pclFiltered.clear(); m_pclFiltered.reserve(m_pclInput.size());
    const double min_sq=m_minRange*m_minRange, max_sq=m_maxRange*m_maxRange;
    size_t count=0;
    for (const auto& p : m_pclInput) {
        const Eigen::Vector3f ray=Eigen::Vector3f(p.x,p.y,p.z)-sensor_in_cloud;
        if (!ray.allFinite()) { ++m_invalidPoints; continue; }
        const double distance=ray.cast<double>().squaredNorm();
        if (distance<=1e-12 || distance<min_sq || distance>max_sq) { ++m_rangeFiltered; continue; }
        if (count++ % m_PcDownsampling) { ++m_subsampled; continue; }
        m_pclFiltered.push_back(p);
    }
    pcl::transformPointCloud(m_pclFiltered,m_pclOutput,T);
    {
        std::lock_guard lock(m_gridMutex);
        m_grid3d.loadCloud(std::span<const pcl::PointXYZ>(m_pclOutput.points.data(),m_pclOutput.size()),sensor_position);
        ++m_integrated;
        m_lastStamp=rclcpp::Time(cloud->header.stamp).nanoseconds();
        m_outsidePoints+=m_grid3d.lastIntegration().outside_points;
        m_skippedBlocks+=m_grid3d.lastIntegration().skipped_blocks;
        if (m_grid3d.lastIntegration().skipped_blocks)
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Map capacity reached; inspect get_status skipped_blocks/evictions");
    }
    if (m_publishCloud && m_cloudPub->get_subscription_count()) {
        sensor_msgs::msg::PointCloud2 corrected;
        pcl::toROSMsg(m_pclOutput,corrected);
        corrected.header=cloud->header; corrected.header.frame_id=m_fixedFrameId;
        m_cloudPub->publish(corrected);
    }
    m_lastCallbackMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    RCLCPP_DEBUG(get_logger(),"Integrated frame; callback %.3f ms",m_lastCallbackMs.load());
}

void TSDFNode::tfCallback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg)
{
    if (msg->header.stamp.sec<0 || msg->header.stamp.nanosec>=1000000000) { ++m_invalidTransforms; return; }
    try { (void)getTransformMatrix(*msg); }
    catch(const std::exception& error) {
        ++m_invalidTransforms;
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Invalid legacy TF: %s",error.what());
        return;
    }
    std::lock_guard<std::mutex> lock(m_tfMutex);
    const auto stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
    auto position=std::lower_bound(m_tfHist.begin(),m_tfHist.end(),stamp,[](const auto& tf,int64_t t) {
        return rclcpp::Time(tf.header.stamp).nanoseconds()<t;
    });
    if (position!=m_tfHist.end() && rclcpp::Time(position->header.stamp).nanoseconds()==stamp) *position=*msg;
    else m_tfHist.insert(position,*msg);
    while (m_tfHist.size()>100) m_tfHist.pop_front();
}

void TSDFNode::gpsCallback(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    if (!isValidFix(*msg)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Ignoring invalid NavSatFix on '%s' (status=%d, lat=%.8f, lon=%.8f, alt=%.3f)",
            m_inGpsTopic.c_str(), msg->status.status, msg->latitude, msg->longitude, msg->altitude);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_gpsMutex);
        m_latestGps = *msg;
        m_hasLatestGps = true;
    }
    // The outgoing topic is a retained, valid-fix-only handoff for the
    // ATAK/TAK bridge. It is deliberately not used as the map datum.
    m_gpsPub->publish(*msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Valid GPS fix: lat=%.6f lon=%.6f ellipsoid_alt=%.2f (status=%d)",
        msg->latitude, msg->longitude, msg->altitude, msg->status.status);
}

Eigen::Matrix4f TSDFNode::getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    Eigen::Quaterniond q(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z
    );
    if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm()<1e-9)
        throw std::invalid_argument("TF quaternion must be finite and nonzero");
    q.normalize();
    Eigen::Matrix3f rotation = q.toRotationMatrix().cast<float>();

    Eigen::Vector3f translation(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
    );

    if (!translation.allFinite()) throw std::invalid_argument("TF translation must be finite");
    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = translation;

    return transform;
}

void TSDFNode::saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    queueExport("pcd",[](auto& map,const fs::path& dir){map.exportGridToPCD((dir/"grid_data.pcd").string(),1);},response);
}
void TSDFNode::saveGridPLY(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    queueExport("ply",[](auto& map,const fs::path& dir){map.exportGridToPLY((dir/"grid_data.ply").string(),1);},response);
}
void TSDFNode::saveGridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    queueExport("csv",[](auto& map,const fs::path& dir){map.exportSubgridToCSV((dir/"grid_data_csv").string(),1);},response);
}
void TSDFNode::saveGridMesh(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    queueExport("mesh",[](auto& map,const fs::path& dir){map.exportMesh((dir/"mesh.stl").string(),0);},response);
}
void TSDFNode::saveGridAtakZip(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!m_geoOrigin.configured) {
        response->success=false;
        response->message="Geo-referenced export requires an explicit WGS-84/ENU datum.";
        return;
    }
    queueExport("atak",[this](auto& map,const fs::path& dir) {
        map.exportMeshOBJ((dir/"atak_mesh").string(),0,m_colorScheme,m_colorBins);
        std::ofstream origin(dir/"atak_mesh.origin.json");
        origin.exceptions(std::ios::badbit|std::ios::failbit);
        origin << geoReferenceJson(latestGpsSnapshot()) << '\n'; origin.close();
        zipFiles(dir);
    },response);
}

// Returns the immutable WGS-84/ENU datum used for the map frame, plus the
// latest valid GPS sample if one has been received.
void TSDFNode::getGeoOrigin(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!m_geoOrigin.configured) {
        response->success = false;
        response->message =
            "No configured geo origin. Set geo_origin_configured=true to the datum used by robot_localization.";
        return;
    }

    response->success = true;
    response->message = geoReferenceJson(latestGpsSnapshot());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<TSDFNode>("db_tsdf_node");
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::cerr << "Error creating or running the node: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
