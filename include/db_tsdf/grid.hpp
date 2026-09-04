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

#include <db_tsdf/mask.hpp>

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

template <db_tsdf::SupportedMask Mask>
struct VoxelDataT
{
    Mask d;          // Bit-count encodes the truncated distance rank
    uint8_t s;       // bit0: sign (0 occ / 1 free)
    uint8_t hits;    // hit counter
};
static_assert(sizeof(VoxelDataT<uint16_t>) == 4, "16-bit voxel layout must occupy 4 bytes");
static_assert(sizeof(VoxelDataT<uint32_t>) == 8, "32-bit voxel layout must occupy 8 bytes");

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
class Grid
{
public:

    using VoxelData = VoxelDataT<Mask>;
    static constexpr int mask_bits = db_tsdf::mask_bits_v<Mask>;

    struct Iterator
    {
        Iterator(Grid* parent, VoxelData **grid, uint32_t i, uint32_t base, uint32_t j, uint32_t cellSizeX)
        {
            _parent = parent;
            _grid = grid;
            _i = i;
            _j = j;
            _base = base;
            _cellSizeX = cellSizeX;
            _curr = _grid[_i];
        }

        Iterator& operator=(const Iterator &it)
        {
            _grid = it._grid;
            _i = it._i;
            _j = it._j;
            _base = it._base;
            _cellSizeX = it._cellSizeX;
            _curr = it._curr;
            return *this;
        }

        VoxelData &operator*() {
            if (_curr == _parent->_dummy) {
                return _parent->_garbage;
            }
            return _curr[_j+_base];
        }

        VoxelData *operator->() {
            if (_curr == _parent->_dummy) {
                return &(_parent->_garbage);
            }
            return _curr + _j + _base;
        }

        Iterator& operator++()
        {
            _j++;
            if(_j >= _cellSizeX)
            {
                _j = 0;
                _i++;
                _curr = _grid[_i];
            }
            return *this;
        }

    protected:
        Grid* _parent;
        VoxelData **_grid;
        VoxelData *_curr;
        uint32_t _i, _j, _base, _cellSizeX;
    };

    Grid(void)
    {
        _grid = NULL;
        _buffer = NULL;
        _garbage = VoxelData{db_tsdf::full_mask_v<Mask>, 0xFF, 0xFF};
        _dummy = NULL;
    }

    void setup(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, float cellRes = 0.05, int maxCells = 100000)
    {
        if(_grid != NULL)
            free(_grid);

        if(_buffer != NULL)
            free(_buffer);

        if(_dummy != NULL)
            free(_dummy);

        _maxX = (int)ceil(maxX);
        _maxY = (int)ceil(maxY);
        _maxZ = (int)ceil(maxZ);
        _minX = (int)floor(minX);
        _minY = (int)floor(minY);
        _minZ = (int)floor(minZ);

        _gridSizeX = abs(_maxX-_minX);
        _gridSizeY = abs(_maxY-_minY);
        _gridSizeZ = abs(_maxZ-_minZ);
        _gridStepY = _gridSizeX;
        _gridStepZ = _gridSizeX*_gridSizeY;
        _gridSize = _gridSizeX*_gridSizeY*_gridSizeZ;

        _maxCells = (uint64_t)maxCells;
        _cellRes = cellRes;
        _oneDivRes = 1.0/_cellRes;
        _cellSizeX = (uint32_t)_oneDivRes;
        _cellSizeY = (uint32_t)_oneDivRes;
        _cellSizeZ = (uint32_t)_oneDivRes;
        _cellStepY = _cellSizeX;
        _cellStepZ = _cellSizeX*_cellSizeY;
        _cellSize = 1 + static_cast<uint64_t>(_cellSizeX)*_cellSizeY*_cellSizeZ;
        _buffer = (VoxelData *)malloc(_maxCells*_cellSize*sizeof(VoxelData));
        std::memset(_buffer, -1, _maxCells*_cellSize*sizeof(VoxelData));
        for(int i=0; i<_maxCells; i++)
        {
            writeControlIndex(&_buffer[i*_cellSize], _gridSize);
        }
        _cellIndex = 0;

        _dummy = (VoxelData*)malloc(_cellSize * sizeof(VoxelData));
        std::memset(_dummy, -1, _cellSize * sizeof(VoxelData));
        writeControlIndex(_dummy, _gridSize);
        _grid = (VoxelData**)malloc(_gridSize * sizeof(VoxelData*));
        for (uint32_t k = 0; k < _gridSize; ++k) _grid[k] = _dummy;
    }

    ~Grid(void)
    {
        if(_grid != NULL)
            free(_grid);

        if(_buffer != NULL)
            free(_buffer);

        if(_dummy != NULL)
            free(_dummy);
    }

    void clear(void)
    {
        for (uint32_t k = 0; k < _gridSize; ++k) _grid[k] = _dummy;
        std::memset(_buffer, -1, _maxCells*_cellSize*sizeof(VoxelData));
        for(uint64_t i=0; i<_maxCells; i++)
        {
            writeControlIndex(&_buffer[i*_cellSize], _gridSize);
        }
        _cellIndex = 0;
    }

    void allocCell(float x, float y, float z)
    {
        x -= _minX;
        y -= _minY;
        z -= _minZ;
        uint32_t int_x = (uint32_t)x, int_y = (uint32_t)y, int_z = (uint32_t)z;
        uint32_t i = int_x + int_y*_gridStepY + int_z*_gridStepZ;
        if( _grid[i] == _dummy)
        {
            _grid[i] = _buffer + (_cellIndex % _maxCells)*_cellSize;
            const uint32_t old_index = readControlIndex(_grid[i]);

            if (old_index != _gridSize) {
                _grid[old_index] = _dummy;
            }

            VoxelData* cell = _grid[i];
            for (uint64_t j = 1; j < _cellSize; ++j) {
                cell[j].d    = db_tsdf::full_mask_v<Mask>;
                cell[j].s    = 1u;
                cell[j].hits = 0u;
            }

            writeControlIndex(cell, i);
            _cellIndex++;
        }
    }

    VoxelData &operator()(float x, float y, float z)
    {
        x -= _minX;
        y -= _minY;
        z -= _minZ;
        uint32_t int_x = (uint32_t)x, int_y = (uint32_t)y, int_z = (uint32_t)z;
        uint32_t i = int_x + int_y*_gridStepY + int_z*_gridStepZ;
        if(_grid[i] == _dummy) { return _garbage; }
        uint32_t j = 1 + (uint32_t)((x-int_x)*_oneDivRes) + (uint32_t)((y-int_y)*_oneDivRes)*_cellStepY + (uint32_t)((z-int_z)*_oneDivRes)*_cellStepZ;
        return _grid[i][j];
    }

    VoxelData read(float x, float y, float z)
    {
        x -= _minX;
        y -= _minY;
        z -= _minZ;
        uint32_t int_x = (uint32_t)x, int_y = (uint32_t)y, int_z = (uint32_t)z;
        uint32_t i = int_x + int_y*_gridStepY + int_z*_gridStepZ;
        if(_grid[i] == _dummy) { return _garbage; }

        uint32_t j = 1 + (uint32_t)((x-int_x)*_oneDivRes) + (uint32_t)((y-int_y)*_oneDivRes)*_cellStepY + (uint32_t)((z-int_z)*_oneDivRes)*_cellStepZ;
        return _grid[i][j];
    }

    Iterator getIterator(float x, float y, float z)
    {
        x -= _minX;
        y -= _minY;
        z -= _minZ;
        uint32_t int_x = (uint32_t)x, int_y = (uint32_t)y, int_z = (uint32_t)z;
        uint32_t i = int_x + int_y*_gridStepY + int_z*_gridStepZ;

        return Iterator(this, _grid, i, 1 + (uint32_t)((y-int_y)*_oneDivRes)*_cellStepY + (uint32_t)((z-int_z)*_oneDivRes)*_cellStepZ, (uint32_t)((x-int_x)*_oneDivRes), _cellSizeX);
    }

    void exportGridToPCD(const std::string& filename, int subsampling_factor)
    {
        using PointT = pcl::PointXYZ;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        const uint32_t step = std::max(1, subsampling_factor);

        for (uint32_t cz = 0; cz < _gridSizeZ; ++cz)
        {
            const float z0 = _minZ + static_cast<float>(cz);
            for (uint32_t cy = 0; cy < _gridSizeY; ++cy)
            {
                const float y0 = _minY + static_cast<float>(cy);
                for (uint32_t cx = 0; cx < _gridSizeX; ++cx)
                {
                    const float x0 = _minX + static_cast<float>(cx);
                    const uint32_t i = cx + cy * _gridStepY + cz * _gridStepZ;
                    VoxelData* cell = _grid[i];
                    if (cell == _dummy) continue;
                    for (uint32_t vz = 0; vz < _cellSizeZ; vz += step) {
                        for (uint32_t vy = 0; vy < _cellSizeY; vy += step) {
                            for (uint32_t vx = 0; vx < _cellSizeX; vx += step) {
                                const uint32_t j = 1u + vx + vy * _cellStepY + vz * _cellStepZ;
                                const int dist = db_tsdf::maskRank(cell[j].d);
                                if (dist > 1u) continue;
                                if ((cell[j].s & 0x01u) != 0u) continue;

                                PointT pt;
                                pt.x = x0 + (vx + 0.5f) * _cellRes;
                                pt.y = y0 + (vy + 0.5f) * _cellRes;
                                pt.z = z0 + (vz + 0.5f) * _cellRes;
                                cloud->push_back(pt);
                            }
                        }
                    }
                }
            }
        }

        if (cloud->empty())
        {
            std::cerr << "[GRID" << mask_bits << "] Warning: Empty Cloud (no mask==0 found).\n";
            return;
        }

        pcl::io::savePCDFileBinary(filename, *cloud);
    }

    void exportGridToPLY(const std::string& filename, int subsampling_factor)
    {
        using PointT = pcl::PointXYZ;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        const uint32_t step = std::max(1, subsampling_factor);

        for (uint32_t cz = 0; cz < _gridSizeZ; ++cz)
        {
            const float z0 = _minZ + static_cast<float>(cz);
            for (uint32_t cy = 0; cy < _gridSizeY; ++cy)
            {
                const float y0 = _minY + static_cast<float>(cy);
                for (uint32_t cx = 0; cx < _gridSizeX; ++cx)
                {
                    const float x0 = _minX + static_cast<float>(cx);
                    const uint32_t i = cx + cy * _gridStepY + cz * _gridStepZ;
                    VoxelData* cell = _grid[i];
                    if (cell == _dummy) continue;
                    for (uint32_t vz = 0; vz < _cellSizeZ; vz += step) {
                        for (uint32_t vy = 0; vy < _cellSizeY; vy += step) {
                            for (uint32_t vx = 0; vx < _cellSizeX; vx += step) {
                                const uint32_t j = 1u + vx + vy * _cellStepY + vz * _cellStepZ;
                                const int dist = db_tsdf::maskRank(cell[j].d);
                                if (dist > 1u) continue;
                                if ((cell[j].s & 0x01u) != 0u) continue;

                                PointT pt;
                                pt.x = x0 + (vx + 0.5f) * _cellRes;
                                pt.y = y0 + (vy + 0.5f) * _cellRes;
                                pt.z = z0 + (vz + 0.5f) * _cellRes;
                                cloud->push_back(pt);
                            }
                        }
                    }
                }
            }
        }

        if (cloud->empty())
        {
            std::cerr << "[GRID" << mask_bits << "] Warning: Empty Cloud (no mask==0 found).\n";
            return;
        }

        pcl::io::savePLYFileBinary(filename, *cloud);
    }

    void exportSubgridToCSV(const std::string& out_dir, int subsampling_factor)
    {
        (void)subsampling_factor;

        std::filesystem::create_directories(out_dir);

        for (uint32_t cz = 0; cz < _gridSizeZ; ++cz)
        {
            const float z0 = _minZ + static_cast<float>(cz);
            for (uint32_t cy = 0; cy < _gridSizeY; ++cy)
            {
                const float y0 = _minY + static_cast<float>(cy);
                for (uint32_t cx = 0; cx < _gridSizeX; ++cx)
                {
                    const float x0 = _minX + static_cast<float>(cx);

                    const uint32_t i = cx + cy * _gridStepY + cz * _gridStepZ;
                    VoxelData* cell = _grid[i];
                    if (cell == _dummy) continue;

                    bool has_occupied = false;
                    for (uint32_t vz = 0; vz < _cellSizeZ && !has_occupied; ++vz)
                    for (uint32_t vy = 0; vy < _cellSizeY && !has_occupied; ++vy)
                    for (uint32_t vx = 0; vx < _cellSizeX; ++vx)
                    {
                        const uint32_t j = 1u + vx + vy * _cellStepY + vz * _cellStepZ;
                        if ( (cell[j].s & 0x01u) == 0u ) { has_occupied = true; break; }
                    }
                    if (!has_occupied) continue;

                    const int X = static_cast<int>(std::floor(x0 + 1e-6f));
                    const int Y = static_cast<int>(std::floor(y0 + 1e-6f));
                    const int Z = static_cast<int>(std::floor(z0 + 1e-6f));

                    std::ostringstream base;
                    base << out_dir << "/" << X << "_" << Y << "_" << Z;

                    std::ofstream f_csv(base.str() + ".csv");
                    if (!f_csv.is_open()) {
                        std::cerr << "[GRID" << mask_bits << "] Could not open " << (base.str()+".csv") << "\n";
                        continue;
                    }
                    f_csv << "x,y,z,d_manhattan,s,hits\n";
                    f_csv << std::fixed << std::setprecision(6);

                    std::vector<std::array<float,3>> pts;
                    pts.reserve(1024);

                    const float half = 0.5f * _cellRes;
                    for (uint32_t vz = 0; vz < _cellSizeZ; ++vz)
                    {
                        const float z_voxel_min = z0 + static_cast<float>(vz) * _cellRes;
                        for (uint32_t vy = 0; vy < _cellSizeY; ++vy)
                        {
                            const float y_voxel_min = y0 + static_cast<float>(vy) * _cellRes;
                            for (uint32_t vx = 0; vx < _cellSizeX; ++vx)
                            {
                                const float x_voxel_min = x0 + static_cast<float>(vx) * _cellRes;
                                const uint32_t j = 1u + vx + vy * _cellStepY + vz * _cellStepZ;

                                const int distance_rank = db_tsdf::maskRank(cell[j].d);
                                const uint32_t s = (cell[j].s & 0x01u);
                                const uint32_t hits = static_cast<uint32_t>(cell[j].hits);

                                const float cxp = x_voxel_min + half;
                                const float cyp = y_voxel_min + half;
                                const float czp = z_voxel_min + half;
                                f_csv << cxp << "," << cyp << "," << czp << ","
                                      << distance_rank << "," << s << "," << hits << "\n";

                                if ( (s & 0x01u) == 0u ) {
                                    pts.push_back({cxp, cyp, czp});
                                }
                            }
                        }
                    }
                    f_csv.close();

                    std::ofstream f_ply(base.str() + ".ply");
                    if (!f_ply.is_open()) {
                        std::cerr << "[GRID" << mask_bits << "] Could not open " << (base.str()+".ply") << "\n";
                        continue;
                    }
                    f_ply << "ply\nformat ascii 1.0\n";
                    f_ply << "element vertex " << pts.size() << "\n";
                    f_ply << "property float x\nproperty float y\nproperty float z\n";
                    f_ply << "end_header\n";
                    f_ply << std::fixed << std::setprecision(6);
                    for (const auto& p : pts) {
                        f_ply << p[0] << " " << p[1] << " " << p[2] << "\n";
                    }
                    f_ply.close();
                }
            }
        }
    }

    vtkSmartPointer<vtkPolyData> buildSurfaceMesh(float iso_level, int occ_min_hits)
    {
        vtkSmartPointer<vtkAppendPolyData> appender =
            vtkSmartPointer<vtkAppendPolyData>::New();

        const float BAND = 0.1f * _cellRes;

        for (uint32_t cz = 0; cz < _gridSizeZ; ++cz)
        for (uint32_t cy = 0; cy < _gridSizeY; ++cy)
        for (uint32_t cx = 0; cx < _gridSizeX; ++cx)
        {
            const uint32_t i = cx + cy * _gridStepY + cz * _gridStepZ;
            VoxelData* cell = _grid[i];
            if (cell == _dummy) continue;

            vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New();
            image->SetDimensions(_cellSizeX + 1, _cellSizeY + 1, _cellSizeZ + 1);
            image->SetSpacing(_cellRes, _cellRes, _cellRes);

            const float x0 = _minX + static_cast<float>(cx);
            const float y0 = _minY + static_cast<float>(cy);
            const float z0 = _minZ + static_cast<float>(cz);

            image->SetOrigin(x0, y0, z0);
            image->AllocateScalars(VTK_FLOAT, 1);

            float *dest = static_cast<float*>(image->GetScalarPointer());
            bool has_occupied_voxels = false;

            for (uint32_t vz = 0; vz < _cellSizeZ + 1; ++vz)
            for (uint32_t vy = 0; vy < _cellSizeY + 1; ++vy)
            for (uint32_t vx = 0; vx < _cellSizeX + 1; ++vx)
            {
                VoxelData vox = this->read(x0 + vx * _cellRes,
                                           y0 + vy * _cellRes,
                                           z0 + vz * _cellRes);

                const bool enough_hits = (vox.hits >= occ_min_hits);
                const bool occupied = ((vox.s & 0x01u) == 0);

                float sdf_value;
                if (!enough_hits) {
                    sdf_value = +BAND;
                } else if (occupied) {
                    sdf_value = -BAND;
                    has_occupied_voxels = true;
                } else {
                    sdf_value = +BAND;
                }

                dest[vx + vy * (_cellSizeX + 1) + vz * (_cellSizeX + 1) * (_cellSizeY + 1)] = sdf_value;
            }

            if (has_occupied_voxels)
            {
                auto smoother = vtkSmartPointer<vtkImageGaussianSmooth>::New();
                smoother->SetInputData(image);
                smoother->SetStandardDeviation(1.0);
                smoother->Update();

                auto mc = vtkSmartPointer<vtkMarchingCubes>::New();
                mc->SetInputConnection(smoother->GetOutputPort());
                mc->SetValue(0, iso_level);
                mc->Update();

                appender->AddInputData(mc->GetOutput());
            }
        }

        std::cout << "[GRID" << mask_bits << "] Joining cell meshes...\n";
        appender->Update();

        auto out = vtkSmartPointer<vtkPolyData>::New();
        out->ShallowCopy(appender->GetOutput());
        return out;
    }

    void exportMesh(const std::string& filename, float iso_level, int occ_min_hits)
    {
        vtkSmartPointer<vtkPolyData> mesh = buildSurfaceMesh(iso_level, occ_min_hits);

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

        std::cout << "[GRID" << mask_bits << "] OBJ export finished: " << objFilename
                  << " and " << mtlFilename
                  << " (" << faces.size() << " faces)\n";
    }

protected:

    static uint32_t readControlIndex(const VoxelData* cell)
    {
        uint32_t index;
        std::memcpy(&index, cell, sizeof(index));
        return index;
    }

    static void writeControlIndex(VoxelData* cell, uint32_t index)
    {
        std::memcpy(cell, &index, sizeof(index));
    }

    VoxelData **_grid;
    float _maxX, _maxY, _maxZ, _minX, _minY, _minZ;
    uint32_t _gridSizeX, _gridSizeY, _gridSizeZ, _gridStepY, _gridStepZ, _gridSize;
    float _cellRes, _oneDivRes;
    uint32_t _cellSizeX, _cellSizeY, _cellSizeZ, _cellStepY, _cellStepZ;
    uint64_t _maxCells, _cellSize, _cellIndex;
    VoxelData *_buffer;
    VoxelData *_dummy;
    VoxelData _garbage;
};

using GRID16 = Grid<uint16_t>;
using GRID32 = Grid<uint32_t>;

#endif
