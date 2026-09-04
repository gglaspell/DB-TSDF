/*
 * TSDFNode: ROS 2 node for LiDAR->TDF mapping.
 * Subscribes to PointCloud2, looks up the transform via tf2_ros::Buffer,
 * filters, transforms, and integrates the cloud into the TSDF grid.
 * Publishes the filtered, global-frame cloud for visualization.
 */

#include <algorithm>
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
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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

class TSDFNode : public rclcpp::Node
{
private:
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
        m_inCloudTopic      = this->declare_parameter<std::string>("in_cloud", "/os_cloud_node/points");
        m_odomFrameId       = this->declare_parameter<std::string>("odom_frame_id", "odom");
        m_fixedFrameId      = this->declare_parameter<std::string>("fixed_frame_id", "");
        m_earthFrameId      = this->declare_parameter<std::string>("earth_frame_id", "earth");
        m_useTf             = this->declare_parameter<bool>("use_tf", true);
        m_useTfTopic        = this->declare_parameter<bool>("use_tf_topic", true);
        m_inTfTopic         = this->declare_parameter<std::string>("in_tf_topic", "/gt_icp/transform");
        m_inGpsTopic        = this->declare_parameter<std::string>("in_gps_topic", "/gps/fix");
        m_outGpsTopic       = this->declare_parameter<std::string>("out_gps_topic", "/current_gps_fix");

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

        m_geoOrigin.configured = this->declare_parameter<bool>("geo_origin_configured", false);
        m_geoOrigin.latitude_deg = this->declare_parameter<double>("geo_origin_latitude", 0.0);
        m_geoOrigin.longitude_deg = this->declare_parameter<double>("geo_origin_longitude", 0.0);
        m_geoOrigin.ellipsoid_height_m = this->declare_parameter<double>("geo_origin_altitude", 0.0);
        m_geoOrigin.map_yaw_rad = this->declare_parameter<double>("geo_origin_yaw", 0.0);
        m_geoOrigin.altitude_reference = this->declare_parameter<std::string>(
            "geo_origin_altitude_reference", "ellipsoid");
        if (m_geoOrigin.configured) {
            validateGeoOrigin(m_geoOrigin);
        }

        m_tdfGridSizeX_low  = this->declare_parameter<double>("tdfGridSizeX_low", -10.0);
        m_tdfGridSizeX_high = this->declare_parameter<double>("tdfGridSizeX_high", 10.0);
        m_tdfGridSizeY_low  = this->declare_parameter<double>("tdfGridSizeY_low", -10.0);
        m_tdfGridSizeY_high = this->declare_parameter<double>("tdfGridSizeY_high", 10.0);
        m_tdfGridSizeZ_low  = this->declare_parameter<double>("tdfGridSizeZ_low", -10.0);
        m_tdfGridSizeZ_high = this->declare_parameter<double>("tdfGridSizeZ_high", 10.0);
        m_tdfGridRes        = this->declare_parameter<double>("tdf_grid_res", 0.10);
        m_tdfMaxCells       = this->declare_parameter<double>("tdf_max_cells", 10000.0);
        m_minRange          = this->declare_parameter<double>("min_range", 1.0);
        m_maxRange          = this->declare_parameter<double>("max_range", 100.0);
        m_PcDownsampling    = this->declare_parameter<int>("pc_downsampling", 1);
        m_occMinHits        = this->declare_parameter<int>("occ_min_hits", 1);
        m_binsAz            = this->declare_parameter<int>("bins_az", 40);
        m_binsEl            = this->declare_parameter<int>("bins_el", 40);
        m_shadowRadius      = this->declare_parameter<int>("shadow_radius", 6);
        m_distanceMode      = this->declare_parameter<std::string>("distance_mode", "L1");
        m_kernelSize        = this->declare_parameter<int>("kernel_size", 11);
        m_verboseInit       = this->declare_parameter<bool>("verbose_init", false);

        m_colorScheme       = this->declare_parameter<std::string>("color_scheme", "grayscale");
        m_colorBins         = this->declare_parameter<int>("color_bins", 32);

        if (m_kernelSize % 2 == 0) {
            RCLCPP_WARN(this->get_logger(), "Kernel size must be odd! Forcing %d -> %d.", m_kernelSize, m_kernelSize + 1);
            m_kernelSize++;
        }

        if (m_colorBins < 1) {
            RCLCPP_WARN(this->get_logger(), "color_bins must be >= 1. Forcing %d -> 1.", m_colorBins);
            m_colorBins = 1;
            this->set_parameter(rclcpp::Parameter("color_bins", m_colorBins));
        }

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
        m_cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", 100);
        m_gpsPub = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            m_outGpsTopic, rclcpp::QoS(1).reliable().transient_local());

        // Subscriptions
        auto qos_keepall_reliable = rclcpp::QoS(rclcpp::KeepAll()).reliable().durability_volatile();

        m_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_inCloudTopic,
            qos_keepall_reliable,
            std::bind(&TSDFNode::pointcloudCallback, this, std::placeholders::_1));

        if (m_useTf && m_useTfTopic) {
            RCLCPP_INFO(this->get_logger(), "Using Legacy TF Mode: subscribing to '%s'", m_inTfTopic.c_str());
            m_tfSub = this->create_subscription<geometry_msgs::msg::TransformStamped>(
                m_inTfTopic,
                qos_keepall_reliable,
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

        save_service_pcd_ = this->create_service<std_srvs::srv::Trigger>("/save_grid_pcd",
            std::bind(&TSDFNode::saveGridPCD, this, std::placeholders::_1, std::placeholders::_2));

        save_service_ply_ = this->create_service<std_srvs::srv::Trigger>("/save_grid_ply",
            std::bind(&TSDFNode::saveGridPLY, this, std::placeholders::_1, std::placeholders::_2));

        save_service_csv_ = this->create_service<std_srvs::srv::Trigger>("/save_grid_csv",
            std::bind(&TSDFNode::saveGridCSV, this, std::placeholders::_1, std::placeholders::_2));

        save_service_mesh_ = this->create_service<std_srvs::srv::Trigger>("/save_grid_mesh",
            std::bind(&TSDFNode::saveGridMesh, this, std::placeholders::_1, std::placeholders::_2));

        save_service_atak_ = this->create_service<std_srvs::srv::Trigger>("/save_grid_atak_zip",
            std::bind(&TSDFNode::saveGridAtakZip, this, std::placeholders::_1, std::placeholders::_2));

        get_geo_origin_service_ = this->create_service<std_srvs::srv::Trigger>("/get_geo_origin",
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
                       m_verboseInit);

        RCLCPP_INFO(this->get_logger(),
            "DB-TSDF is ready! Grid: %.1f x %.1f x %.1f m @ %.3f m/voxel",
            fabs(m_tdfGridSizeX_high - m_tdfGridSizeX_low),
            fabs(m_tdfGridSizeY_high - m_tdfGridSizeY_low),
            fabs(m_tdfGridSizeZ_high - m_tdfGridSizeZ_low),
            m_tdfGridRes);
    }

    ~TSDFNode() override {
        std::lock_guard<std::mutex> lock(m_exportThreadsMutex);
        for (auto &worker : m_exportThreads) {
            if (worker.joinable()) {
                worker.join();
            }
        }
        RCLCPP_INFO(this->get_logger(), "Node closed successfully.");
    }

private:
    static bool isValidFix(const sensor_msgs::msg::NavSatFix &fix)
    {
        return fix.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
               std::isfinite(fix.latitude) && std::isfinite(fix.longitude) &&
               std::isfinite(fix.altitude) &&
               fix.latitude >= -90.0 && fix.latitude <= 90.0 &&
               fix.longitude >= -180.0 && fix.longitude <= 180.0;
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

    void startExport(std::function<void()> job)
    {
        std::lock_guard<std::mutex> lock(m_exportThreadsMutex);
        m_exportThreads.emplace_back([this, job = std::move(job)]() mutable {
            // Services use fixed output names. Serializing exports prevents
            // simultaneous requests from interleaving their OBJ/sidecar/ZIP
            // files while m_gridMutex protects the TSDF itself.
            std::lock_guard<std::mutex> export_lock(m_exportMutex);
            job();
        });
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
    std::mutex m_exportMutex;
    std::mutex m_exportThreadsMutex;
    std::vector<std::thread> m_exportThreads;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcSub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloudPub;
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
    static size_t counter = 0;
    counter++;

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    rclcpp::Time t_query = cloud->header.stamp;
    if (t_query.nanoseconds() == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Cloud timestamp is 0. Using latest available transform (rclcpp::Time(0)).");
        t_query = rclcpp::Time(0);
    }

    if (m_useTf) {
        if (m_useTfTopic) {
            std::lock_guard<std::mutex> lock(m_tfMutex);
            if (m_tfHist.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "No TF received yet on LEGACY topic %s -> discarding cloud",
                                     m_inTfTopic.c_str());
                return;
            }

            auto best_it = m_tfHist.begin();
            auto best_dt = rclcpp::Duration::from_nanoseconds(
                std::llabs((t_query - best_it->header.stamp).nanoseconds()));

            for (auto it = std::next(m_tfHist.begin()); it != m_tfHist.end(); ++it) {
                auto dt = rclcpp::Duration::from_nanoseconds(
                    std::llabs((t_query - it->header.stamp).nanoseconds()));
                if (dt < best_dt) {
                    best_dt = dt;
                    best_it = it;
                }
            }

            if (best_dt > m_maxSkew) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Best legacy TF deltaT (%.3f ms) > max_skew -> discarding cloud",
                                     best_dt.seconds() * 1e3);
                return;
            }
            T = getTransformMatrix(*best_it);

        } else {
            try {
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg = m_tfBuffer->lookupTransform(
                    m_fixedFrameId,
                    cloud->header.frame_id,
                    t_query,
                    rclcpp::Duration(0, 100000000)
                );
                T = getTransformMatrix(tf_msg);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Could not transform %s to fixed frame %s: %s",
                    cloud->header.frame_id.c_str(), m_fixedFrameId.c_str(), ex.what()
                );
                return;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_in;
    pcl::fromROSMsg(*cloud, pcl_in);

    pcl::PointCloud<pcl::PointXYZ> pcl_filtered;
    pcl_filtered.reserve(pcl_in.size());

    const double min_sq = m_minRange * m_minRange;
    const double max_sq = m_maxRange * m_maxRange;
    int cnt = 0;

    for (const auto &p : pcl_in) {
        const double d2 = p.x*p.x + p.y*p.y + p.z*p.z;
        if (d2 < min_sq || d2 > max_sq) continue;
        if (cnt++ % m_PcDownsampling) continue;
        pcl_filtered.points.push_back(p);
    }

    pcl_filtered.width = pcl_filtered.points.size();
    pcl_filtered.height = 1;
    pcl_filtered.is_dense = true;

    pcl::PointCloud<pcl::PointXYZ> pcl_out;
    pcl::transformPointCloud(pcl_filtered, pcl_out, T);

    std::vector<pcl::PointXYZ> pts_global(pcl_out.points.begin(), pcl_out.points.end());
    Eigen::Vector3f sensor_position = T.block<3,1>(0,3);
    {
        std::lock_guard<std::mutex> lock(m_gridMutex);
        m_grid3d.loadCloud(pts_global, sensor_position);
    }

    sensor_msgs::msg::PointCloud2 cloud_corrected;
    pcl::toROSMsg(pcl_out, cloud_corrected);
    cloud_corrected.header = cloud->header;
    cloud_corrected.header.frame_id = m_fixedFrameId;
    m_cloudPub->publish(cloud_corrected);

    auto end = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double,std::milli>(end - start).count();
    RCLCPP_INFO(this->get_logger(), "Received frame #%zu · time = %.3f ms", counter, ms);
}

void TSDFNode::tfCallback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_tfMutex);
    m_tfHist.push_back(*msg);
    while (m_tfHist.size() > 100)
        m_tfHist.pop_front();
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

    Eigen::Quaternionf q(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z
    );
    Eigen::Matrix3f rotation = q.toRotationMatrix();

    Eigen::Vector3f translation(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
    );

    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = translation;

    return transform;
}

// ros2 service call /save_grid_pcd std_srvs/srv/Trigger
void TSDFNode::saveGridPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Exporting grid to PCD (grid_data.pcd)...");
    startExport([this]() {
        try {
            std::lock_guard<std::mutex> lock(m_gridMutex);
            m_grid3d.exportGridToPCD("grid_data.pcd", 1);
            RCLCPP_INFO(this->get_logger(), "PCD export finished: grid_data.pcd");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "PCD export failed: %s", e.what());
        }
    });

    response->success = true;
    response->message = "PCD export started in the background.";
}

// ros2 service call /save_grid_ply std_srvs/srv/Trigger
void TSDFNode::saveGridPLY(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Exporting grid to PLY (grid_data.ply)...");
    startExport([this]() {
        try {
            std::lock_guard<std::mutex> lock(m_gridMutex);
            m_grid3d.exportGridToPLY("grid_data.ply", 1);
            RCLCPP_INFO(this->get_logger(), "PLY export finished: grid_data.ply");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "PLY export failed: %s", e.what());
        }
    });

    response->success = true;
    response->message = "PLY export started in the background.";
}

// ros2 service call /save_grid_csv std_srvs/srv/Trigger
void TSDFNode::saveGridCSV(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Exporting subgrid cells to CSV (grid_data_csv/)...");
    startExport([this]() {
        try {
            std::lock_guard<std::mutex> lock(m_gridMutex);
            m_grid3d.exportSubgridToCSV("grid_data_csv", 1);
            RCLCPP_INFO(this->get_logger(), "CSV export finished: grid_data_csv/");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CSV export failed: %s", e.what());
        }
    });

    response->success = true;
    response->message = "CSV export started in the background.";
}

// ros2 service call /save_grid_mesh std_srvs/srv/Trigger
void TSDFNode::saveGridMesh(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    constexpr float iso = 0.0f;
    RCLCPP_INFO(this->get_logger(), "Exporting grid to mesh (mesh.stl, iso=%.3f)...", iso);
    startExport([this, iso]() {
        try {
            std::lock_guard<std::mutex> lock(m_gridMutex);
            m_grid3d.exportMesh("mesh.stl", iso);
            RCLCPP_INFO(this->get_logger(), "Mesh export finished: mesh.stl");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Mesh export failed: %s", e.what());
        }
    });

    response->success = true;
    response->message = "Mesh export started in the background.";
}

// ros2 service call /save_grid_atak_zip std_srvs/srv/Trigger
void TSDFNode::saveGridAtakZip(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    constexpr float iso = 0.0f;

    if (!m_geoOrigin.configured) {
        response->success = false;
        response->message =
            "Geo-referenced export requires geo_origin_configured=true with the WGS-84/ENU datum "
            "used by robot_localization.";
        return;
    }

    std::string colorScheme = normalizeColorScheme(this->get_parameter("color_scheme").as_string());
    int colorBins = this->get_parameter("color_bins").as_int();
    if (colorBins < 1) colorBins = 1;
    const std::string geoReference = geoReferenceJson(latestGpsSnapshot());

    RCLCPP_INFO(this->get_logger(),
                "Exporting ATAK OBJ package (atak_mesh.obj/.mtl/.zip, scheme=%s, bins=%d, iso=%.3f)...",
                colorScheme.c_str(), colorBins, iso);

    startExport([this, colorScheme, colorBins, iso, geoReference]() {
        const std::string base = "atak_mesh";
        const std::string objFile = base + ".obj";
        const std::string mtlFile = base + ".mtl";
        const std::string zipFile = base + ".zip";
        const std::string originFile = base + ".origin.json";

        try {
            {
                std::lock_guard<std::mutex> lock(m_gridMutex);
                m_grid3d.exportMeshOBJ(base, iso, colorScheme, colorBins);
            }

            std::ofstream ofs(originFile);
            if (!ofs.is_open()) {
                throw std::runtime_error("could not open geo-reference sidecar for writing");
            }
            ofs << geoReference << '\n';
            ofs.close();

            const std::string zipCmd = "zip -j -q " + zipFile + " " + objFile + " " + mtlFile + " " + originFile;
            int rc = std::system(zipCmd.c_str());
            if (rc != 0) {
                throw std::runtime_error("zip command failed with exit code " + std::to_string(rc));
            }

            RCLCPP_INFO(this->get_logger(), "Geo-referenced OBJ package export finished: %s", zipFile.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Geo-referenced OBJ package export failed: %s", e.what());
        }
    });

    response->success = true;
    response->message = "Geo-referenced OBJ export queued in the background.";
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
    }

    rclcpp::shutdown();
    return 0;
}
