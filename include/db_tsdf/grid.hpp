#ifndef DB_TSDF_GRID_HPP
#define DB_TSDF_GRID_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <db_tsdf/volume.hpp>
#include <vtkCleanPolyData.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <clocale>

// Mesh
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkMarchingCubes.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkSTLWriter.h>
#include <vtkAppendPolyData.h>
#include <vtkImageGaussianSmooth.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>

inline std::array<float,3> grid_colormap_grayscale(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    return {t, t, t};
}

inline std::array<float,3> grid_colormap_warm(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    float r = std::min(1.0f, 2.0f * t);
    float g = std::max(0.0f, 2.0f * t - 1.0f);
    float b = 0.0f;
    return {r, g, b};
}

inline std::array<float,3> grid_colormap_cool(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    float r = t;
    float g = 1.0f - t;
    float b = 1.0f;
    return {r, g, b};
}

inline std::array<float,3> grid_colormap_jet(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);

    float r = std::clamp(1.5f - std::fabs(4.0f * t - 3.0f), 0.0f, 1.0f);
    float g = std::clamp(1.5f - std::fabs(4.0f * t - 2.0f), 0.0f, 1.0f);
    float b = std::clamp(1.5f - std::fabs(4.0f * t - 1.0f), 0.0f, 1.0f);
    return {r, g, b};
}

inline std::array<float,3> grid_apply_colormap(const std::string& scheme, float t)
{
    if (scheme == "warm") return grid_colormap_warm(t);
    if (scheme == "cool") return grid_colormap_cool(t);
    if (scheme == "jet")  return grid_colormap_jet(t);
    return grid_colormap_grayscale(t);
}

template <db_tsdf::SupportedMask Mask>
class Grid : public Volume<Mask> {
public:
    using Base = Volume<Mask>;
    using typename Base::VoxelData;
    using typename Base::Index;
    static constexpr int mask_bits = Base::mask_bits;

    void configureMesh(const std::string& mode, double sigma = -1) {
        if (mode != "occupancy" && mode != "distance")
            throw std::invalid_argument("mesh_mode must be occupancy or distance");
        if (!std::isfinite(sigma) || (sigma < 0 && sigma != -1) || sigma > 5)
            throw std::invalid_argument("mesh_smoothing_sigma must be -1 (auto) or 0..5 voxels");
        _meshMode = mode;
        _sigma = sigma < 0 ? (mode == "occupancy" ? 1.0 : 0.0) : sigma;
    }
    void setRankDistances(std::vector<double> distances) { _rankDistances = std::move(distances); }
    double metricDistance(Mask mask) const {
        const size_t rank = db_tsdf::maskRank(mask);
        return rank < _rankDistances.size() ? _rankDistances[rank] : rank*this->resolution();
    }
    pcl::PointCloud<pcl::PointXYZ> surfaceCloud(int subsampling = 1) const {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        const int step = std::max(1,subsampling);
        this->visit([&](const Index& p, const VoxelData& v) {
            if (p[0]%step || p[1]%step || p[2]%step || (v.s&1) || !(v.s&2) || db_tsdf::maskRank(v.d)>1)
                return;
            const auto c = this->center(p);
            cloud.push_back(pcl::PointXYZ(c[0],c[1],c[2]));
        });
        return cloud;
    }
    void exportGridToPCD(const std::string& filename, int subsampling) const {
        auto cloud = surfaceCloud(subsampling);
        if (cloud.empty()) throw std::runtime_error("PCD export: map has no occupied surface");
        if (pcl::io::savePCDFileBinary(filename,cloud) != 0)
            throw std::runtime_error("PCD writer failed: " + filename);
    }
    void exportGridToPLY(const std::string& filename, int subsampling) const {
        auto cloud = surfaceCloud(subsampling);
        if (cloud.empty()) throw std::runtime_error("PLY export: map has no occupied surface");
        if (pcl::io::savePLYFileBinary(filename,cloud) != 0)
            throw std::runtime_error("PLY writer failed: " + filename);
    }
    void exportSubgridToCSV(const std::string& directory, int subsampling) const {
        if (!this->activeCount()) throw std::runtime_error("CSV export: map is empty");
        std::filesystem::create_directories(directory);
        const int step = std::max(1,subsampling);
        for (auto id : this->activeBlocks()) {
            std::ofstream out(std::filesystem::path(directory)/("block_"+std::to_string(id)+".csv"));
            out.exceptions(std::ios::badbit | std::ios::failbit);
            out << "x,y,z,distance_rank,distance_m,free,observed,hits\n" << std::setprecision(9);
            const auto origin = this->blockOrigin(id);
            for (int z=0;z<this->blockSide();z+=step)
            for (int y=0;y<this->blockSide();y+=step)
            for (int x=0;x<this->blockSide();x+=step) {
                const Index p{origin[0]+x,origin[1]+y,origin[2]+z};
                if (!this->valid(p)) continue;
                const auto v = this->readIndex(p);
                const auto c = this->center(p);
                out << c[0]<<','<<c[1]<<','<<c[2]<<','<<db_tsdf::maskRank(v.d)<<','
                    <<metricDistance(v.d)<<','<<int(v.s&1)<<','<<int((v.s&2)!=0)<<','<<int(v.hits)<<'\n';
            }
            out.close();
        }
    }

    vtkSmartPointer<vtkPolyData> buildSurfaceMesh(float iso_level, int occ_min_hits) {
        auto appender = vtkSmartPointer<vtkAppendPolyData>::New();
        const int side = this->blockSide();
        const int halo = static_cast<int>(std::ceil(3*_sigma));
        const int dim = side+1+2*halo;
        const double res = this->resolution();
        for (auto id : this->activeBlocks()) {
            const auto origin = this->blockOrigin(id);
            auto image = vtkSmartPointer<vtkImageData>::New();
            image->SetDimensions(dim,dim,dim);
            image->SetSpacing(res,res,res);
            const auto first = this->center({origin[0]-halo,origin[1]-halo,origin[2]-halo});
            image->SetOrigin(first.data());
            image->AllocateScalars(VTK_FLOAT,1);
            auto* dest = static_cast<float*>(image->GetScalarPointer());
            bool occupied = false;
            for (int z=0;z<dim;++z)
            for (int y=0;y<dim;++y)
            for (int x=0;x<dim;++x) {
                const auto v = this->readIndex({origin[0]+x-halo,origin[1]+y-halo,origin[2]+z-halo});
                const bool inside = !(v.s&1) && (v.s&2) && v.hits>=occ_min_hits;
                occupied |= inside;
                const double magnitude = _meshMode=="distance" ? metricDistance(v.d) : 0.1*res;
                dest[x+(y+static_cast<size_t>(z)*dim)*dim] = inside ? -magnitude : magnitude;
            }
            if (!occupied) continue;
            auto mc = vtkSmartPointer<vtkMarchingCubes>::New();
            auto smoother = vtkSmartPointer<vtkImageGaussianSmooth>::New();
            if (_sigma>0) {
                smoother->SetInputData(image);
                smoother->SetStandardDeviation(_sigma);
                smoother->SetRadiusFactors(3,3,3);
                mc->SetInputConnection(smoother->GetOutputPort());
            } else mc->SetInputData(image);
            mc->SetValue(0,iso_level);
            mc->Update();
            // Each cube has one owning block. Remove halo triangles and, in
            // distance mode, cubes with unknown corners instead of inventing
            // a surface against unobserved space.
            auto part = vtkSmartPointer<vtkPolyData>::New();
            auto faces = vtkSmartPointer<vtkCellArray>::New();
            auto* mesh = mc->GetOutput();
            part->SetPoints(mesh->GetPoints());
            auto* polys = mesh->GetPolys();
            polys->InitTraversal();
            vtkIdType n; const vtkIdType* vertices;
            while (polys->GetNextCell(n,vertices)) {
                if (n!=3) continue;
                std::array<double,3> centroid{};
                for (int v=0;v<3;++v) {
                    double pt[3]; mesh->GetPoint(vertices[v],pt);
                    for (int a=0;a<3;++a) centroid[a]+=pt[a]/3;
                }
                Index cube;
                bool keep = true;
                for (int a=0;a<3;++a) {
                    cube[a] = static_cast<int>(std::floor((centroid[a]-this->_min[a])/res-0.5+1e-6));
                    keep &= cube[a]>=origin[a] && cube[a]<origin[a]+side;
                }
                if (_meshMode=="distance" && keep) {
                    for (int z=0;z<2;++z) for (int y=0;y<2;++y) for (int x=0;x<2;++x)
                        keep &= (this->readIndex({cube[0]+x,cube[1]+y,cube[2]+z}).s&2)!=0;
                }
                if (keep) faces->InsertNextCell(n,vertices);
            }
            part->SetPolys(faces);
            appender->AddInputData(part);
        }
        if (!appender->GetNumberOfInputConnections(0)) return vtkSmartPointer<vtkPolyData>::New();
        auto clean = vtkSmartPointer<vtkCleanPolyData>::New();
        clean->SetInputConnection(appender->GetOutputPort());
        clean->ToleranceIsAbsoluteOn();
        clean->SetAbsoluteTolerance(res*1e-5);
        clean->Update();
        auto out = vtkSmartPointer<vtkPolyData>::New();
        out->ShallowCopy(clean->GetOutput());
        return out;
    }
    void exportMesh(const std::string& filename, float iso_level, int occ_min_hits)
    {
        vtkSmartPointer<vtkPolyData> mesh = buildSurfaceMesh(iso_level, occ_min_hits);
        if (mesh->GetNumberOfPolys() == 0)
            throw std::runtime_error("Mesh export: map has no surface triangles");

        auto ext_pos = filename.find_last_of('.');
        std::string ext = (ext_pos == std::string::npos) ? "" : filename.substr(ext_pos + 1);

        if (ext == "stl") {
            auto writer = vtkSmartPointer<vtkSTLWriter>::New();
            writer->SetFileName(filename.c_str());
            writer->SetInputData(mesh);
            writer->SetFileTypeToBinary();
            if (!writer->Write()) {
                throw std::runtime_error("VTK STL writer failed to write mesh.");
            }
        }
        else if (ext == "vtp") {
            auto writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
            writer->SetFileName(filename.c_str());
            writer->SetInputData(mesh);
            if (!writer->Write()) {
                throw std::runtime_error("VTK XML writer failed to write mesh.");
            }
        }
        else {
            throw std::invalid_argument("Unsupported file extension: " + ext);
        }
    }

    void exportMeshOBJ(const std::string& basename,
                       float iso_level,
                       int occ_min_hits,
                       const std::string& color_scheme,
                       int color_bins)
    {
        vtkSmartPointer<vtkPolyData> mesh = buildSurfaceMesh(iso_level, occ_min_hits);
        vtkPoints* points = mesh->GetPoints();
        vtkCellArray* polys = mesh->GetPolys();

        if (points == nullptr || polys == nullptr || points->GetNumberOfPoints() == 0) {
            throw std::runtime_error("[GRID] OBJ export: mesh is empty.");
        }

        const vtkIdType nPts = points->GetNumberOfPoints();

        double zMin =  std::numeric_limits<double>::max();
        double zMax = -std::numeric_limits<double>::max();
        for (vtkIdType p = 0; p < nPts; ++p) {
            double pt[3];
            points->GetPoint(p, pt);
            zMin = std::min(zMin, pt[2]);
            zMax = std::max(zMax, pt[2]);
        }

        const double zRange = ((zMax - zMin) > 1e-12) ? (zMax - zMin) : 1.0;
        const int bins = std::max(1, color_bins);

        struct Face {
            vtkIdType v0, v1, v2;
            int bin;
        };

        std::vector<Face> faces;
        faces.reserve(static_cast<size_t>(polys->GetNumberOfCells()));

        polys->InitTraversal();
        vtkIdType npts_cell = 0;
        const vtkIdType* pts_cell = nullptr;

        while (polys->GetNextCell(npts_cell, pts_cell)) {
            if (npts_cell != 3) continue;

            double p0[3], p1[3], p2[3];
            points->GetPoint(pts_cell[0], p0);
            points->GetPoint(pts_cell[1], p1);
            points->GetPoint(pts_cell[2], p2);

            const double centroidZ = (p0[2] + p1[2] + p2[2]) / 3.0;
            const double t = (centroidZ - zMin) / zRange;

            int bin = static_cast<int>(std::floor(t * static_cast<double>(bins)));
            bin = std::clamp(bin, 0, bins - 1);

            faces.push_back(Face{pts_cell[0], pts_cell[1], pts_cell[2], bin});
        }

        if (faces.empty()) {
            throw std::runtime_error("[GRID] OBJ export: no triangle faces were produced.");
        }

        const std::string objFilename = basename + ".obj";
        const std::string mtlFilename = basename + ".mtl";
        const std::string mtlBaseName = std::filesystem::path(mtlFilename).filename().string();

        std::set<int> usedBins;
        for (const auto& f : faces) usedBins.insert(f.bin);

        std::ofstream mtl(mtlFilename);
        if (!mtl.is_open()) {
            throw std::runtime_error("[GRID] Could not open " + mtlFilename + " for writing.");
        }

        mtl << "# Auto-generated material library\n";
        mtl << "# scheme=" << color_scheme << " bins=" << bins << "\n\n";
        mtl << std::fixed << std::setprecision(6);

        for (int b : usedBins) {
            const float t = (bins > 1) ? static_cast<float>(b) / static_cast<float>(bins - 1) : 0.0f;
            const auto rgb = grid_apply_colormap(color_scheme, t);

            mtl << "newmtl bin_" << b << "\n";
            mtl << "Ka 0.000000 0.000000 0.000000\n";
            mtl << "Kd " << rgb[0] << " " << rgb[1] << " " << rgb[2] << "\n";
            mtl << "Ks 0.000000 0.000000 0.000000\n";
            mtl << "d 1.000000\n";
            mtl << "illum 1\n\n";
        }
        mtl.close();
        if (!mtl) throw std::runtime_error("MTL write failed");

        std::ofstream obj(objFilename);
        if (!obj.is_open()) {
            throw std::runtime_error("[GRID] Could not open " + objFilename + " for writing.");
        }

        obj << "# Auto-generated OBJ mesh with centroid-Z false coloring\n";
        obj << "mtllib " << mtlBaseName << "\n\n";
        obj << std::fixed << std::setprecision(6);

        for (vtkIdType p = 0; p < nPts; ++p) {
            double pt[3];
            points->GetPoint(p, pt);
            obj << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
        }
        obj << "\n";

        std::stable_sort(faces.begin(), faces.end(),
                         [](const Face& a, const Face& b) { return a.bin < b.bin; });

        int currentBin = -1;
        for (const auto& f : faces) {
            if (f.bin != currentBin) {
                obj << "usemtl bin_" << f.bin << "\n";
                currentBin = f.bin;
            }
            obj << "f " << (f.v0 + 1) << " " << (f.v1 + 1) << " " << (f.v2 + 1) << "\n";
        }

        obj.close();
        if (!obj) throw std::runtime_error("OBJ write failed");

        std::cout << "[GRID" << mask_bits << "] OBJ export finished: " << objFilename
                  << " and " << mtlFilename
                  << " (" << faces.size() << " faces)\n";
    }

private:
    std::string _meshMode{"occupancy"};
    double _sigma{1};
    std::vector<double> _rankDistances;
};
using GRID16 = Grid<uint16_t>;
using GRID32 = Grid<uint32_t>;
#endif
