#ifndef DB_TSDF_PROVENANCE_HPP
#define DB_TSDF_PROVENANCE_HPP

#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>
#include <sys/utsname.h>
#include <openssl/evp.h>
#include <nlohmann/json.hpp>
#include <pcl/pcl_config.h>
#include <vtkVersion.h>
#include <Eigen/Core>
#if __has_include(<db_tsdf/build_info.hpp>)
#include <db_tsdf/build_info.hpp>
#endif

namespace db_tsdf {
inline std::string sha256(const std::filesystem::path& path) {
    std::ifstream in(path,std::ios::binary);
    if (!in) throw std::runtime_error("cannot hash file: "+path.string());
    std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(),EVP_MD_CTX_free);
    if (!context || EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)!=1)
        throw std::runtime_error("cannot initialize SHA-256");
    std::array<char,65536> buffer;
    while (in) {
        in.read(buffer.data(),buffer.size());
        if (EVP_DigestUpdate(context.get(),buffer.data(),in.gcount())!=1)
            throw std::runtime_error("SHA-256 update failed");
    }
    if (!in.eof()) throw std::runtime_error("read failed while hashing: "+path.string());
    std::array<unsigned char,EVP_MAX_MD_SIZE> digest;
    unsigned length=0;
    if (EVP_DigestFinal_ex(context.get(),digest.data(),&length)!=1)
        throw std::runtime_error("SHA-256 finalization failed");
    std::ostringstream out;
    for (unsigned i=0;i<length;++i) out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(digest[i]);
    return out.str();
}
inline nlohmann::json fileChecksums(const std::filesystem::path& directory) {
    nlohmann::json out=nlohmann::json::object();
    for (const auto& entry : std::filesystem::recursive_directory_iterator(directory))
        if (entry.is_regular_file()) out[entry.path().lexically_relative(directory).generic_string()]=sha256(entry.path());
    return out;
}
inline nlohmann::json buildInfo() {
    nlohmann::json info={{"compiler",__VERSION__},{"pcl",PCL_VERSION_PRETTY},
        {"vtk",vtkVersion::GetVTKVersion()},{"eigen",std::to_string(EIGEN_WORLD_VERSION)+"."+
            std::to_string(EIGEN_MAJOR_VERSION)+"."+std::to_string(EIGEN_MINOR_VERSION)},
        {"logical_cpus",std::thread::hardware_concurrency()}};
#ifdef DB_TSDF_SOURCE_SHA256
    info["source_sha256"]=DB_TSDF_SOURCE_SHA256;
    info["git_revision"]=DB_TSDF_GIT_REVISION;
    info["build_type"]=DB_TSDF_BUILD_TYPE;
#endif
#ifdef DB_TSDF_CXX_FLAGS
    info["cxx_flags"]=DB_TSDF_CXX_FLAGS;
    info["native"]=DB_TSDF_BUILD_NATIVE;
    info["sanitizers"]=DB_TSDF_BUILD_SANITIZERS;
    info["git_dirty_at_configure"]=DB_TSDF_GIT_DIRTY;
#endif
    utsname system{};
    if (uname(&system)==0) info["system"]={{"os",system.sysname},{"release",system.release},{"architecture",system.machine}};
    std::ifstream cpu("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpu,line)) {
        if (line.rfind("model name",0)==0) {
            info["cpu"]=line.substr(line.find(':')+2); break;
        }
    }
    return info;
}
}
#endif
