// Deterministic, frame-accounted input path for evaluation without ROS/DDS.
#include <db_tsdf/tsdf3d.hpp>
#include <db_tsdf/provenance.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>
#include <sys/resource.h>

namespace fs=std::filesystem;
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;

double milliseconds(Clock::time_point start) {
    return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
Json distribution(std::vector<double> values) {
    if (values.empty()) return nullptr;
    std::sort(values.begin(),values.end());
    auto percentile=[&](double fraction) {
        const double index=fraction*(values.size()-1);
        const auto lo=static_cast<size_t>(index), hi=std::min(lo+1,values.size()-1);
        return values[lo]+(values[hi]-values[lo])*(index-lo);
    };
    return {{"mean",std::accumulate(values.begin(),values.end(),0.0)/values.size()},
        {"p50",percentile(.5)},{"p95",percentile(.95)},{"p99",percentile(.99)},{"max",values.back()}};
}
void writeJson(const fs::path& path,const Json& value) {
    std::ofstream out(path);
    out.exceptions(std::ios::badbit|std::ios::failbit);
    out<<value.dump(2)<<'\n'; out.close();
}
pcl::PointCloud<pcl::PointXYZ> readCloud(const fs::path& path) {
    pcl::PCLPointCloud2 raw;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(path.string(),raw)!=0 || raw.width==0 || raw.height==0)
        throw std::runtime_error("could not read nonempty XYZ PCD: "+path.string());
    for (const char* name : {"x","y","z"}) {
        const auto field=std::find_if(raw.fields.begin(),raw.fields.end(),[&](const auto& f){return f.name==name;});
        if (field==raw.fields.end() || field->datatype!=pcl::PCLPointField::FLOAT32 || field->count!=1)
            throw std::invalid_argument("PCD must contain float32 XYZ fields: "+path.string());
    }
    pcl::fromPCLPointCloud2(raw,cloud);
    return cloud;
}
Json nearestDistances(const pcl::PointCloud<pcl::PointXYZ>& source,
                      const pcl::PointCloud<pcl::PointXYZ>& target,double tolerance) {
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target.makeShared());
    std::vector<double> distances;
    distances.reserve(source.size());
    std::vector<int> indices(1);
    std::vector<float> squared(1);
    size_t within=0;
    double sumSquared=0;
    for (const auto& point : source) {
        if (tree.nearestKSearch(point,1,indices,squared)!=1)
            throw std::runtime_error("nearest-neighbor search failed");
        const double d=std::sqrt(squared[0]);
        distances.push_back(d); sumSquared+=squared[0]; within+=d<=tolerance;
    }
    auto result=distribution(std::move(distances));
    result["rmse"]=std::sqrt(sumSquared/source.size());
    result["fraction_within_tolerance"]=double(within)/source.size();
    return result;
}
fs::path syntheticInput(const fs::path& directory) {
    fs::create_directory(directory);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // An exact, noise-free plane with fixed poses. It is a regression fixture,
    // not evidence of real-world reconstruction accuracy or speed.
    for (int z=3;z<27;++z) for (int y=3;y<27;++y)
        cloud.emplace_back(1.05f,(y+.5f)*.1f,(z+.5f)*.1f);
    if (pcl::io::savePCDFileBinary((directory/"plane.pcd").string(),cloud)!=0)
        throw std::runtime_error("could not write synthetic fixture");
    Json frames=Json::array();
    for (int i=0;i<20;++i) frames.push_back({{"pcd","plane.pcd"},
        {"stamp_ns",1000000000ll+i*100000000ll},{"sensor_origin_m",{0.,1.5,1.5}}});
    Json manifest={{"schema","db-tsdf-input/v1"},{"fixed_frame","map"},
        {"fixture","noise-free-plane-v1"},{"parameters",{{"tdf_grid_res",.1},
        {"tdfGridSizeX_low",0.},{"tdfGridSizeX_high",3.},
        {"tdfGridSizeY_low",0.},{"tdfGridSizeY_high",3.},
        {"tdfGridSizeZ_low",0.},{"tdfGridSizeZ_high",3.},
        {"tdf_max_cells",27},{"kernel_size",5},{"bins_az",16},{"bins_el",8},
        {"occ_min_hits",3},{"distance_mode","L2"},{"mesh_mode","distance"}}},
        {"frames",frames},{"reference_pcd","plane.pcd"},{"evaluation_tolerance_m",.15}};
    writeJson(directory/"manifest.json",manifest);
    return directory/"manifest.json";
}
struct Options {
    fs::path manifest,output;
    int bits{16},threads{1};
    bool synthetic{false};
};
template<class Mask> void run(const Options& options,const fs::path& staging) {
    const auto runStart=Clock::now();
    const auto manifestPath=options.synthetic ? syntheticInput(staging/"inputs") : fs::absolute(options.manifest);
    const auto parent=manifestPath.parent_path();
    const auto manifestHash=db_tsdf::sha256(manifestPath);
    std::ifstream input(manifestPath);
    const auto manifest=Json::parse(input);
    if (manifest.at("schema")!="db-tsdf-input/v1") throw std::invalid_argument("unsupported manifest schema");
    const auto& frames=manifest.at("frames");
    if (!frames.is_array() || frames.empty()) throw std::invalid_argument("manifest needs a nonempty frames array");
    Json params=manifest.value("parameters",Json::object()), effective=Json::object();
    auto value=[&]<class T>(const char* key,T fallback) -> T {
        const auto item=params.value(key,Json(fallback));
        if constexpr (std::is_integral_v<T>) {
            if (!item.is_number_integer() || item.template get<int64_t>()<std::numeric_limits<T>::min() ||
                item.template get<int64_t>()>std::numeric_limits<T>::max())
                throw std::invalid_argument(std::string(key)+" must be an integer in range");
        }
        const T result=item.template get<T>(); effective[key]=result; return result;
    };
    const double resolution=value("tdf_grid_res",.05);
    const double minX=value("tdfGridSizeX_low",-10.), maxX=value("tdfGridSizeX_high",10.);
    const double minY=value("tdfGridSizeY_low",-10.), maxY=value("tdfGridSizeY_high",10.);
    const double minZ=value("tdfGridSizeZ_low",-3.), maxZ=value("tdfGridSizeZ_high",5.);
    const int capacity=value("tdf_max_cells",1000), kernel=value("kernel_size",5), hits=value("occ_min_hits",3);
    const int az=value("bins_az",60), el=value("bins_el",60), shadow=value("shadow_radius",3);
    const auto mode=value("distance_mode",std::string("L2")), policy=value("capacity_policy",std::string("stop"));
    const auto meshMode=value("mesh_mode",std::string("occupancy"));
    const double sigma=value("mesh_smoothing_sigma",-1.), budget=value("memory_budget_mb",0.);
    const int downsampling=value("pc_downsampling",1);
    const double minRange=value("min_range",0.), maxRange=value("max_range",1000.);
    if (downsampling<1 || !std::isfinite(minRange) || !std::isfinite(maxRange) || minRange<0 || maxRange<=minRange)
        throw std::invalid_argument("invalid downsampling/range settings");
    for (const auto& [key,unused] : params.items())
        if (!effective.contains(key)) throw std::invalid_argument("unknown manifest parameter: "+key);
    TSDF3D<Mask> mapper;
    mapper.setup(minX,maxX,minY,maxY,minZ,maxZ,resolution,kernel,hits,az,el,shadow,mode,capacity,
                 false,options.threads,policy,budget);
    mapper.configureMesh(meshMode,sigma);
    const std::string frame=manifest.at("fixed_frame").get<std::string>();
    if (frame.empty()) throw std::invalid_argument("fixed_frame must not be empty");
    const double setupMs=milliseconds(runStart);
    std::map<fs::path,std::string> inputHashes;
    Json relativeHashes=Json::object();
    auto checkedPath=[&](const std::string& name) {
        const auto path=fs::weakly_canonical(parent/fs::path(name));
        if (!inputHashes.contains(path)) inputHashes[path]=db_tsdf::sha256(path);
        relativeHashes[name]=inputHashes.at(path);
        return path;
    };
    std::vector<double> integrationMs,frameMs;
    size_t inputPoints=0,invalid=0,outside=0,rangeFiltered=0,subsampled=0;
    int64_t lastStamp=0;
    for (const auto& entry : frames) {
        const auto frameStart=Clock::now();
        if (!entry.at("stamp_ns").is_number_integer()) throw std::invalid_argument("stamp_ns must be an integer");
        const int64_t stamp=entry.at("stamp_ns").get<int64_t>();
        if (stamp<=lastStamp) throw std::invalid_argument("frame stamps must be positive and strictly increasing");
        lastStamp=stamp;
        auto cloud=readCloud(checkedPath(entry.at("pcd").get<std::string>()));
        const auto origin=entry.at("sensor_origin_m").get<std::array<float,3>>();
        const Eigen::Vector3f sensor(origin[0],origin[1],origin[2]);
        if (!sensor.allFinite()) throw std::invalid_argument("sensor_origin_m must be finite");
        const auto pose=entry.value("cloud_to_fixed",std::array<float,7>{0,0,0,0,0,0,1});
        for (float v : pose) if (!std::isfinite(v)) throw std::invalid_argument("cloud_to_fixed must be finite");
        Eigen::Quaternionf q(pose[6],pose[3],pose[4],pose[5]);
        if (std::abs(q.norm()-1)>1e-3) throw std::invalid_argument("cloud_to_fixed quaternion must be normalized");
        Eigen::Matrix4f transform=Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0)=q.normalized().toRotationMatrix();
        transform.block<3,1>(0,3)=Eigen::Vector3f(pose[0],pose[1],pose[2]);
        // PCL may skip NaNs when is_dense=false; filter explicitly below.
        pcl::transformPointCloud(cloud,cloud,transform);
        pcl::PointCloud<pcl::PointXYZ> filtered;
        filtered.reserve(cloud.size());
        size_t eligible=0;
        inputPoints+=cloud.size();
        for (const auto& p : cloud) {
            const Eigen::Vector3f ray=Eigen::Vector3f(p.x,p.y,p.z)-sensor;
            if (!ray.allFinite()) { ++invalid; continue; }
            const double distance=ray.cast<double>().squaredNorm();
            if (distance<=1e-12 || distance<minRange*minRange || distance>maxRange*maxRange) { ++rangeFiltered; continue; }
            if (eligible++%downsampling) { ++subsampled; continue; }
            filtered.push_back(p);
        }
        const auto integrateStart=Clock::now();
        mapper.loadCloud(std::span<const pcl::PointXYZ>(filtered.data(),filtered.size()),sensor);
        integrationMs.push_back(milliseconds(integrateStart));
        outside+=mapper.lastIntegration().outside_points;
        invalid+=mapper.lastIntegration().invalid_points;
        if (mapper.lastIntegration().skipped_blocks || mapper.grid().evictions())
            throw std::runtime_error("map capacity exceeded; increase tdf_max_cells for evaluation");
        frameMs.push_back(milliseconds(frameStart));
    }
    const auto exportStart=Clock::now();
    mapper.exportGridToPCD((staging/"map.pcd").string(),1);
    mapper.exportMesh((staging/"mesh.stl").string(),0);
    mapper.saveCheckpoint((staging/"map.dbtsdf").string(),frame);
    const double exportMs=milliseconds(exportStart);
    Json quality=nullptr;
    if (manifest.contains("reference_pcd")) {
        const auto reference=readCloud(checkedPath(manifest.at("reference_pcd").get<std::string>()));
        for (const auto& point : reference)
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                throw std::invalid_argument("reference PCD must contain only finite XYZ points");
        const double tolerance=manifest.value("evaluation_tolerance_m",resolution);
        if (!std::isfinite(tolerance) || tolerance<=0) throw std::invalid_argument("evaluation tolerance must be positive");
        const auto surface=mapper.grid().surfaceCloud();
        const auto accuracy=nearestDistances(surface,reference,tolerance);
        const auto completeness=nearestDistances(reference,surface,tolerance);
        const double p=accuracy.at("fraction_within_tolerance"),r=completeness.at("fraction_within_tolerance");
        quality={{"method","nearest neighbors on occupied voxel centers and reference points"},
            {"tolerance_m",tolerance},{"accuracy_m",accuracy},{"completeness_m",completeness},
            {"precision",p},{"recall",r},{"f_score",p+r>0?2*p*r/(p+r):0},
            {"surface_points",surface.size()},{"reference_points",reference.size()}};
    }
    for (const auto& [path,hash] : inputHashes)
        if (db_tsdf::sha256(path)!=hash) throw std::runtime_error("input changed during run: "+path.string());
    if (db_tsdf::sha256(manifestPath)!=manifestHash) throw std::runtime_error("manifest changed during run");
    rusage usage{}; getrusage(RUSAGE_SELF,&usage);
    Json report={{"schema","db-tsdf-offline-run/v1"},{"complete",true},{"build",db_tsdf::buildInfo()},
        {"mask_bits",sizeof(Mask)*8},{"num_threads",mapper.threadCount()},{"parameters",effective},
        {"fixed_frame",frame},{"manifest_sha256",manifestHash},{"inputs_sha256",relativeHashes},
        {"fixture",manifest.value("fixture",std::string("external"))},
        {"expected_frames",frames.size()},{"integrated_frames",integrationMs.size()},{"dropped_frames",0},
        {"last_stamp_ns",lastStamp},{"input_points",inputPoints},{"invalid_points",invalid},
        {"range_filtered_points",rangeFiltered},{"subsampled_points",subsampled},{"outside_points",outside},
        {"state_hash",mapper.grid().stateHash()},{"allocated_blocks",mapper.grid().activeCount()},
        {"allocated_bytes",mapper.grid().allocatedBytes()},{"capacity_bytes",mapper.grid().estimatedBytes()},
        {"evictions",mapper.grid().evictions()},{"peak_rss_bytes",usage.ru_maxrss*1024ll},
        {"setup_ms",setupMs},{"integration_ms",distribution(integrationMs)},
        {"frame_total_ms",distribution(frameMs)},{"export_ms",exportMs},{"quality",quality}};
    report["outputs_sha256"]=db_tsdf::fileChecksums(staging);
    report["total_ms"]=milliseconds(runStart);
    writeJson(staging/"run.json",report);
    std::cout<<report.dump(2)<<'\n';
}
int main(int argc,char** argv) {
    fs::path staging;
    try {
        Options options;
        for (int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            auto value=[&]() -> std::string { if (++i==argc) throw std::invalid_argument("missing value for "+arg); return argv[i]; };
            auto number=[&]() { const auto text=value(); size_t end=0; const int n=std::stoi(text,&end);
                if (end!=text.size()) throw std::invalid_argument("invalid integer: "+text);
                return n; };
            if (arg=="--manifest") options.manifest=value();
            else if (arg=="--output") options.output=value();
            else if (arg=="--mask-bits") options.bits=number();
            else if (arg=="--threads") options.threads=number();
            else if (arg=="--synthetic") options.synthetic=true;
            else if (arg=="--help") {
                std::cout<<"db_tsdf_offline (--synthetic | --manifest inputs.json) --output NEW_DIRECTORY [--mask-bits 16|32] [--threads N]\n";
                return 0;
            } else throw std::invalid_argument("unknown option: "+arg);
        }
        if (options.output.empty() || options.synthetic==!options.manifest.empty() || (options.bits!=16 && options.bits!=32))
            throw std::invalid_argument("provide exactly one of --synthetic/--manifest, --output, and mask bits 16 or 32");
        options.output=fs::absolute(options.output).lexically_normal();
        if (fs::exists(options.output)) throw std::invalid_argument("output already exists: "+options.output.string());
        fs::create_directories(options.output.parent_path());
        staging=options.output.string()+".partial";
        if (!fs::create_directory(staging)) { staging.clear(); throw std::runtime_error("output staging directory already exists"); }
        if (options.bits==16) run<uint16_t>(options,staging); else run<uint32_t>(options,staging);
        fs::rename(staging,options.output);
        return 0;
    } catch(const std::exception& error) {
        if (!staging.empty()) { std::error_code ignored; fs::remove_all(staging,ignored); }
        std::cerr<<"DB-TSDF offline failed: "<<error.what()<<'\n'; return 1;
    }
}
