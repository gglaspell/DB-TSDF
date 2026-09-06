#ifndef DB_TSDF_TSDF3D_HPP
#define DB_TSDF_TSDF3D_HPP

#include <algorithm>
#include <memory>
#include <span>
#include <omp.h>
#include <db_tsdf/trilinear_params.hpp>
#include <db_tsdf/grid.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <map>
#include <set>

template <db_tsdf::SupportedMask Mask>
struct DirectionalKernel
{
    std::vector<uint8_t>  signs;        // 0 = occ, 1 = free
};

template <db_tsdf::SupportedMask Mask>
class TSDF3D
{

public:

using GridType = Grid<Mask>;
using Kernel = DirectionalKernel<Mask>;
static constexpr int mask_bits = db_tsdf::mask_bits_v<Mask>;

TSDF3D(void)
{
    m_maxX = 40;
    m_maxY = 40;
    m_maxZ = 40;
    m_minX = -40;
    m_minY = -40;
    m_minZ = -10;
    m_resolution = 0.05;
    m_oneDivRes = 1/m_resolution;
}

virtual ~TSDF3D() = default;
TSDF3D(const TSDF3D&) = default;
TSDF3D& operator=(const TSDF3D&) = default;
TSDF3D(TSDF3D&&) noexcept = default;
TSDF3D& operator=(TSDF3D&&) noexcept = default;

void setup(double minX, double maxX,
   double minY, double maxY,
   double minZ, double maxZ,
   double resolution,
   int kernelSize,
   int occMinHits,
   int binsAz,
               int binsEl,
               int shadowRadius,
               std::string distanceMode,
   int maxCells = 50000,
   bool verboseInit = false,
   int threads = 0, const std::string& capacityPolicy = "stop",
   double memoryBudgetMiB = 0
   )
{
    TSDF3D next;
    next.configure(minX,maxX,minY,maxY,minZ,maxZ,resolution,kernelSize,occMinHits,
                   binsAz,binsEl,shadowRadius,distanceMode,maxCells,verboseInit,
                   threads,capacityPolicy,memoryBudgetMiB);
    *this = std::move(next);
}

private:
void configure(double minX,double maxX,double minY,double maxY,double minZ,double maxZ,
               double resolution,int kernelSize,int occMinHits,int binsAz,int binsEl,
               int shadowRadius,const std::string& distanceMode,int maxCells,bool verboseInit,
               int threads,const std::string& capacityPolicy,double memoryBudgetMiB)
{
    if (kernelSize < 1 || kernelSize > 63 || kernelSize % 2 == 0)
        throw std::invalid_argument("kernel_size must be odd and between 1 and 63");
    if (occMinHits < 1 || occMinHits > 255)
        throw std::invalid_argument("occ_min_hits must be between 1 and 255");
    if (binsAz <= 0 || binsEl <= 0 || binsAz > 720 || binsEl > 360)
        throw std::invalid_argument("bins_az must be 1..720 and bins_el 1..360");
    if (shadowRadius < 0 || shadowRadius > 1024)
        throw std::invalid_argument("shadow_radius must be 0..1024 voxels");
    if (distanceMode != "L1" && distanceMode != "L2")
        throw std::invalid_argument("distance_mode must be L1 or L2");
    const size_t kernelVoxels = static_cast<size_t>(kernelSize)*kernelSize*kernelSize;
    const size_t kernelBytes = kernelVoxels*(static_cast<size_t>(binsAz)*binsEl+sizeof(Mask));
    if (kernelBytes > 512u*1024u*1024u)
        throw std::invalid_argument("directional kernels exceed 512 MiB; reduce kernel size or bins");
    setThreadCount(threads);
    m_grid.setup(minX,maxX,minY,maxY,minZ,maxZ,resolution,maxCells,capacityPolicy,memoryBudgetMiB);
    if (memoryBudgetMiB > 0 && (m_grid.estimatedBytes()+kernelBytes)/1048576.0 > memoryBudgetMiB)
        throw std::invalid_argument("grid and kernels exceed memory_budget_mb");
    m_bank = std::make_shared<KernelBank>();
    m_r_squared_to_mask.clear();
    m_maxX = maxX;
    m_maxY = maxY;
    m_maxZ = maxZ;
    m_minX = minX;
    m_minY = minY;
    m_minZ = minZ;
    m_resolution = resolution;
    m_oneDivRes = 1/m_resolution;
    m_occMinHits = occMinHits;
    m_binsAz = binsAz;
    m_binsEl = binsEl;
    m_numBins = m_binsAz * m_binsEl;
    m_shadowRadiusMd = shadowRadius;
    m_distanceMode = distanceMode;
    m_kernelSize = kernelSize;
    m_kernelRadius = (kernelSize - 1) / 2;
    m_verboseInit = verboseInit;

    initDirectionalKernels();

    std::vector<double> distances(mask_bits+1);
    for (int rank=0;rank<=mask_bits;++rank) distances[rank]=rank*resolution;
    if (distanceMode=="L2") {
        std::vector<bool> assigned(mask_bits+1,false);
        for (const auto& [r2, mask] : m_r_squared_to_mask) {
            const int rank=db_tsdf::maskRank(mask);
            if (!assigned[rank]) { distances[rank]=std::sqrt(r2)*resolution; assigned[rank]=true; }
        }
    }
    m_grid.setRankDistances(std::move(distances));
}

public:
void clear(void)
{
    m_grid.clear();
    m_lastIntegration = {};
}

void exportGridToPCD(const std::string& filename, int subsampling_factor)
{
    m_grid.exportGridToPCD(filename, subsampling_factor);
}

void exportGridToPLY(const std::string& filename, int subsampling_factor)
{
    m_grid.exportGridToPLY(filename, subsampling_factor);
}

void exportSubgridToCSV(const std::string& filename, int subsampling_factor)
{
    m_grid.exportSubgridToCSV(filename, subsampling_factor);
}

void exportMesh(const std::string& filename, float iso_level)
{
    m_grid.exportMesh(filename, iso_level, m_occMinHits);
}

// Exports the marching-cubes surface as OBJ + MTL, coloring each
// triangle by a discrete Z-bin lookup table (centroid-Z based).
// basename should be given WITHOUT extension, e.g. "atak_mesh" ->
// produces "atak_mesh.obj" and "atak_mesh.mtl".
void exportMeshOBJ(const std::string& basename,
                   float iso_level,
                   const std::string& color_scheme,
                   int color_bins)
{
    m_grid.exportMeshOBJ(basename, iso_level, m_occMinHits, color_scheme, color_bins);
}

virtual inline bool isIntoGrid(const float &x, const float &y, const float &z)
{
    return m_grid.contains(x,y,z);
}


void loadCloud(std::vector<pcl::PointXYZ> &cloud, float tx, float ty, float tz, float yaw)
{
    std::vector<pcl::PointXYZ> out;

    float c = cos(yaw);
    float s = sin(yaw);
    out.resize(cloud.size());
    for(uint32_t i=0; i<out.size(); i++)
    {
        out[i].x = c*cloud[i].x - s*cloud[i].y + tx;
        out[i].y = s*cloud[i].x + c*cloud[i].y + ty;
        out[i].z = cloud[i].z + tz;
    }
    loadCloud(out, Eigen::Vector3f(tx, ty, tz));
}

void loadCloud(std::vector<pcl::PointXYZ> &cloud, float tx, float ty, float tz, float roll, float pitch, float yaw)
{
    std::vector<pcl::PointXYZ> out;

    // Get rotation matrix
    float cr, sr, cp, sp, cy, sy;
    float r00, r01, r02, r10, r11, r12, r20, r21, r22;
    sr = sin(roll);
    cr = cos(roll);
    sp = sin(pitch);
    cp = cos(pitch);
    sy = sin(yaw);
    cy = cos(yaw);
    r00 = cy*cp; r01 = cy*sp*sr-sy*cr; r02 = cy*sp*cr+sy*sr;
    r10 = sy*cp; r11 = sy*sp*sr+cy*cr;r12 = sy*sp*cr-cy*sr;
    r20 = -sp;r21 = cp*sr;r22 = cp*cr;

    // Tiltcompensate points
    out.resize(cloud.size());
    for(uint i=0; i<cloud.size(); i++)
    {
        out[i].x = cloud[i].x*r00 + cloud[i].y*r01 + cloud[i].z*r02 + tx;
        out[i].y = cloud[i].x*r10 + cloud[i].y*r11 + cloud[i].z*r12 + ty;
        out[i].z = cloud[i].x*r20 + cloud[i].y*r21 + cloud[i].z*r22 + tz;
    }
    loadCloud(out, Eigen::Vector3f(tx, ty, tz));
}

void loadCloudFiltered(std::vector<pcl::PointXYZ> &cloud,
                       float tx, float ty, float tz,
                       float roll, float pitch, float yaw,
                       float maxAbs)
{
    std::vector<pcl::PointXYZ> out;
    out.reserve(cloud.size());

    // Matriz de rotación precalculada (radianes)
    const float sr = std::sin(roll),  cr = std::cos(roll);
    const float sp = std::sin(pitch), cp = std::cos(pitch);
    const float sy = std::sin(yaw),   cy = std::cos(yaw);

    const float r00 =  cy*cp;           const float r01 =  cy*sp*sr - sy*cr;  const float r02 =  cy*sp*cr + sy*sr;
    const float r10 =  sy*cp;           const float r11 =  sy*sp*sr + cy*cr;  const float r12 =  sy*sp*cr - cy*sr;
    const float r20 = -sp;              const float r21 =  cp*sr;              const float r22 =  cp*cr;

    for (std::size_t i = 0; i < cloud.size(); ++i)
    {
        const float ix = cloud[i].x;
        const float iy = cloud[i].y;
        const float iz = cloud[i].z;

        // Sólo calcular x', y' para el filtro rápido
        const float xw = ix*r00 + iy*r01 + iz*r02 + tx;
        const float yw = ix*r10 + iy*r11 + iz*r12 + ty;

        // Filtro por ventana rectangular en XY (Chebyshev / AABB)
        if (std::fabs(xw) <= maxAbs && std::fabs(yw) <= maxAbs)
        {
            // Calcula z' sólo si el punto pasa el filtro
            const float zw = ix*r20 + iy*r21 + iz*r22 + tz;

            pcl::PointXYZ p;
            p.x = xw; p.y = yw; p.z = zw;
            out.push_back(p);
        }
    }
    loadCloud(out, Eigen::Vector3f(tx, ty, tz));
}


struct IntegrationStats {
    size_t input_points{0}, valid_points{0}, invalid_points{0}, outside_points{0}, skipped_blocks{0};
};
void setThreadCount(int threads) {
    if (threads < 0 || threads > 1024) throw std::invalid_argument("num_threads must be 0..1024");
    m_threads = threads ? threads : omp_get_max_threads();
}
int threadCount() const { return m_threads; }
const IntegrationStats& lastIntegration() const { return m_lastIntegration; }
const GridType& grid() const { return m_grid; }
GridType& grid() { return m_grid; }
void configureMesh(const std::string& mode, double sigma=-1) { m_grid.configureMesh(mode,sigma); }
std::string checkpointSignature(const std::string& frame) const {
    return std::to_string(m_kernelSize)+" "+std::to_string(m_occMinHits)+" "+
        std::to_string(m_binsAz)+" "+std::to_string(m_binsEl)+" "+
        std::to_string(m_shadowRadiusMd)+" "+m_distanceMode+"\n"+frame;
}
void saveCheckpoint(const std::string& path,const std::string& frame={}) const {
    m_grid.saveCheckpoint(path,checkpointSignature(frame));
}
void loadCheckpoint(const std::string& path,const std::string& frame={}) {
    m_grid.loadCheckpoint(path,checkpointSignature(frame));
}

void loadCloud(std::span<const pcl::PointXYZ> cloud, const Eigen::Vector3f &sensor_pos_world)
{
    if (!sensor_pos_world.allFinite()) throw std::invalid_argument("sensor origin must be finite");
    if (!m_bank) throw std::logic_error("call setup before integrating");
    using Index = typename GridType::Index;
    struct Point { Index center; int bin; };
    std::vector<Point> points;
    points.reserve(cloud.size());
    std::map<uint32_t,std::vector<size_t>> work;
    m_lastIntegration = {};
    m_lastIntegration.input_points = cloud.size();
    const int side = m_grid.blockSide();
    const auto dims = m_grid.dimensions();
    for (const auto& p : cloud) {
        const Eigen::Vector3f direction = Eigen::Vector3f(p.x,p.y,p.z)-sensor_pos_world;
        if (!direction.allFinite() || direction.squaredNorm() <= 1e-12f) {
            ++m_lastIntegration.invalid_points; continue;
        }
        if (!m_grid.contains(p.x,p.y,p.z)) { ++m_lastIntegration.outside_points; continue; }
        const auto center = m_grid.index(p.x,p.y,p.z);
        const size_t pointIndex = points.size();
        points.push_back({center,dirToBin(direction)});
        ++m_lastIntegration.valid_points;
        Index lo,hi;
        for (int a=0;a<3;++a) {
            lo[a] = std::max(0,center[a]-m_kernelRadius)/side*side;
            hi[a] = std::min(dims[a]-1,center[a]+m_kernelRadius);
        }
        for (int z=lo[2];z<=hi[2];z+=side)
        for (int y=lo[1];y<=hi[1];y+=side)
        for (int x=lo[0];x<=hi[0];x+=side)
            work[m_grid.blockId({x,y,z})].push_back(pointIndex);
    }
    // Sorted, unique allocation order makes capacity handling reproducible.
    for (const auto& [id,unused] : work) m_grid.allocateBlock(id);
    struct Job { uint32_t id; const std::vector<size_t>* points; typename GridType::VoxelData* data; };
    std::vector<Job> jobs;
    jobs.reserve(work.size());
    for (const auto& [id,indices] : work) {
        auto* data = m_grid.blockData(id);
        if (data) jobs.push_back({id,&indices,data});
        else ++m_lastIntegration.skipped_blocks;
    }
    // A destination block has exactly one owner, including all kernels whose
    // centers lie in neighboring blocks. No atomics or shared dummy voxel.
    const int workers=static_cast<int>(std::min(jobs.size(),static_cast<size_t>(m_threads)));
    #pragma omp parallel for num_threads(std::max(1,workers)) schedule(static) if(workers>1)
    for (size_t j=0;j<jobs.size();++j) {
        const auto& job = jobs[j];
        const auto origin = m_grid.blockOrigin(job.id);
        for (size_t pi : *job.points) {
            const auto& point = points[pi];
            const auto& signs = m_bank->kernels[point.bin].signs;
            Index lo,hi;
            for (int a=0;a<3;++a) {
                lo[a] = std::max(origin[a],point.center[a]-m_kernelRadius);
                hi[a] = std::min({origin[a]+side-1,dims[a]-1,point.center[a]+m_kernelRadius});
            }
            for (int z=lo[2];z<=hi[2];++z)
            for (int y=lo[1];y<=hi[1];++y) {
                size_t dst = lo[0]-origin[0]+(y-origin[1]+static_cast<size_t>(z-origin[2])*side)*side;
                size_t k = lo[0]-point.center[0]+m_kernelRadius+
                    (y-point.center[1]+m_kernelRadius+
                     static_cast<size_t>(z-point.center[2]+m_kernelRadius)*m_kernelSize)*m_kernelSize;
                for (int x=lo[0];x<=hi[0];++x,++dst,++k) {
                    auto& v = job.data[dst];
                    v.d &= m_bank->distances[k];
                    v.s |= 2;
                    if (!signs[k] && v.hits<m_occMinHits) {
                        ++v.hits;
                        if (v.hits==m_occMinHits) v.s &= uint8_t(~1u);
                    }
                }
            }
        }
    }
}

// Signed metric interpolation between voxel centers. Unknown support is
// explicitly invalid; it must not look like a zero-distance surface.
inline TrilinearParams computeDistInterpolation(const double& x,const double& y,const double& z) const
{
    TrilinearParams r;
    if (!m_grid.contains(x,y,z)) return r;
    auto base=m_grid.index(x,y,z);
    const std::array<double,3> query{x,y,z};
    const auto center=m_grid.center(base);
    for (int a=0;a<3;++a) if (query[a]<center[a]) --base[a];
    std::array<double,8> c;
    for (int dz=0;dz<2;++dz) for (int dy=0;dy<2;++dy) for (int dx=0;dx<2;++dx) {
        const auto v=m_grid.readIndex({base[0]+dx,base[1]+dy,base[2]+dz});
        if (!(v.s&2)) return r;
        c[dx+2*dy+4*dz]=m_grid.metricDistance(v.d)*((v.s&1)?1:-1);
    }
    r.origin=m_grid.center(base);
    r.inverse_resolution=1/m_grid.resolution();
    r.a0=c[0];
    r.a1=c[1]-c[0]; r.a2=c[2]-c[0]; r.a3=c[4]-c[0];
    r.a4=c[3]-c[2]-c[1]+c[0];
    r.a5=c[5]-c[4]-c[1]+c[0];
    r.a6=c[6]-c[4]-c[2]+c[0];
    r.a7=c[7]-c[6]-c[5]-c[3]+c[4]+c[2]+c[1]-c[0];
    r.valid=true;
    return r;
}

protected:

// Grid parameters
GridType m_grid;
double m_maxX, m_maxY, m_maxZ;
double m_minX, m_minY, m_minZ;
double m_resolution, m_oneDivRes;
int m_occMinHits{1};
int m_kernelSize{1};
    int m_kernelRadius{0};
int m_binsAz{1};
    int m_binsEl{1};
    int m_numBins{1};
    int m_shadowRadiusMd{0};
    std::string m_distanceMode;
    bool m_verboseInit{false};

// Directional kernels
struct KernelBank { std::vector<Kernel> kernels; std::vector<Mask> distances; };
std::shared_ptr<KernelBank> m_bank;
int m_threads{1};
IntegrationStats m_lastIntegration;
inline int dirToBin(const Eigen::Vector3f &v) const;
void initDirectionalKernels();

std::map<int, Mask> m_r_squared_to_mask;

};

// Map direction to bin index (azimuth × elevation)
template <db_tsdf::SupportedMask Mask>
inline int TSDF3D<Mask>::dirToBin(const Eigen::Vector3f &v) const
{
    float az = std::atan2(v.y(), v.x());
    if (az < 0)
    {
        az += 2.0f*M_PI;
    }

    const double el = std::asin(std::clamp(v.z() / v.cast<double>().norm(), -1.0, 1.0));
    int   baz = int(az * m_binsAz / (2.0f*M_PI));
    int   bel = int((el + M_PI/2) * m_binsEl / M_PI);

    bel = std::clamp(bel, 0, m_binsEl - 1);
    baz = std::clamp(baz, 0, m_binsAz - 1);

    return bel*m_binsAz + baz;
}


template <db_tsdf::SupportedMask Mask>
inline void TSDF3D<Mask>::initDirectionalKernels()
{
    if (m_distanceMode == "L2")
    {
        if (m_verboseInit) std::cout << "--- [TSDF3D" << mask_bits << "] Generating "
                                     << mask_bits << "-bit L2 (Euclidean) distance LUT..." << std::endl;
        if (m_r_squared_to_mask.empty())
        {
            std::set<int> unique_r_squared;
            for (int z = -m_kernelRadius; z <= m_kernelRadius; ++z)
                for (int y = -m_kernelRadius; y <= m_kernelRadius; ++y)
                    for (int x = -m_kernelRadius; x <= m_kernelRadius; ++x)
                    {
                        unique_r_squared.insert(x*x + y*y + z*z);
                    }

            m_r_squared_to_mask.clear();
            int rank = 0;

            if (m_verboseInit) std::cout << "---Rank -> L2 Distance (Voxels)" << std::endl;
            for (int r2 : unique_r_squared)
            {
                const Mask mask = db_tsdf::rankMask<Mask>(rank);
                m_r_squared_to_mask[r2] = mask;
                if (m_verboseInit)
                {
                    float l2_dist = std::sqrt(static_cast<float>(r2));
                    std::cout << std::setw(5) << rank << " -> " << l2_dist;
                    if (rank >= mask_bits) { std::cout << " (Truncated to " << mask_bits << " bits)"; }
                    std::cout << std::endl;
                }
                rank++;
            }
        }
    }
    else if (m_distanceMode == "L1")
    {
        if (m_verboseInit) std::cout << "--- [TSDF3D" << mask_bits << "] Using "
                                     << mask_bits << "-bit L1 (Manhattan) distance masks." << std::endl;
    }
    else
    {
        std::cerr << "[TSDF3D" << mask_bits << "] Warning: distance_mode '" << m_distanceMode
                  << "' not recognized. Defaulting to 'L1'." << std::endl;
        m_distanceMode = "L1";
    }

    // --- Kernel Generation ---
    m_bank->kernels.resize(m_numBins);

    const int kernel_total_voxels = m_kernelSize * m_kernelSize * m_kernelSize;
    m_bank->distances.resize(kernel_total_voxels);

    auto binToDir = [&](int az,int el) // <-- Añadido [&] para capturar 'this'
    {
        float azr = (az+0.5f)*2.0f*M_PI / m_binsAz;
        float elr = (-M_PI/2)+ (el+0.5f)*M_PI / m_binsEl;
        float c   = cosf(elr);
        return Eigen::Vector3f(c*cosf(azr), c*sinf(azr), sinf(elr));
    };

    for(int el=0; el < m_binsEl; ++el)
    for(int az=0; az < m_binsAz; ++az)
    {
        Kernel& DK = m_bank->kernels[el*m_binsAz+az];
        Eigen::Vector3f dir = binToDir(az,el).normalized();

        DK.signs.resize(kernel_total_voxels);

        int k=0;
        for(int z=-m_kernelRadius;z<=m_kernelRadius;++z)
        for(int y=-m_kernelRadius;y<=m_kernelRadius;++y)
        for(int x=-m_kernelRadius;x<=m_kernelRadius;++x,++k)
        {
            const float R_E = float(m_shadowRadiusMd);
            float re2 = float(x*x + y*y + z*z);

            // The Distance Mode "switch"
            if (el == 0 && az == 0 && m_distanceMode == "L1")
            {
                int l1_dist = std::abs(x) + std::abs(y) + std::abs(z);
                m_bank->distances[k] = db_tsdf::rankMask<Mask>(l1_dist);
            }
            else if (el == 0 && az == 0) // "L2"
            {
                int r2_int = x*x + y*y + z*z;
                m_bank->distances[k] = m_r_squared_to_mask[r2_int];
            }

            bool behind   = dir.dot(Eigen::Vector3f(x,y,z)) >= 0.0f;
            const bool at_hit = (x == 0 && y == 0 && z == 0);
            bool inShadow = behind && (re2 <= R_E*R_E);
            DK.signs[k]   = (at_hit || inShadow) ? 0 : 1;
        }

        // --- Kernel preview (only with verbose_init) ---
        int az_ray_from_left = 0; // Azimuth bin 0 corresponde a +X
        int el_horizontal = m_binsEl / 2; // Bin de elevación central (horizontal)

        if (m_verboseInit && el == el_horizontal && az == az_ray_from_left)
        {
            std::cout << "\n--- Kernel preview (+X Ray) [Z=0 Slice] ---" << std::endl;
            std::cout << "--- Rank " << m_distanceMode << " | Shadow Radius: " << m_shadowRadiusMd << " ---" << std::endl;
            std::cout << "Format: [Rank][Sign] (#=Occupied/Shadow, .=Free)\n" << std::endl;
            std::cout << " (Y+) \n" << "  ^ \n" << "  | \n";

            int z_central = 0;
            int iz = z_central + m_kernelRadius;

            for(int y = m_kernelRadius; y >= -m_kernelRadius; --y) {
                int iy = y + m_kernelRadius;
                std::cout << std::setw(2) << y << " | ";
                for(int x = -m_kernelRadius; x <= m_kernelRadius; ++x) {
                    int ix = x + m_kernelRadius;
                    int k_idx = (iz * m_kernelSize * m_kernelSize) + (iy * m_kernelSize) + ix;
                    Mask mask = m_bank->distances[k_idx];
                    int rank = db_tsdf::maskRank(mask);
                    uint8_t sign = DK.signs[k_idx];
                    char sign_char = (sign == 0) ? '#' : '.';
                    std::cout << " " << std::setw(2) << rank << sign_char;
                }
                std::cout << std::endl;
            }

            std::cout << "   " << std::string(60, '-') << "> (X+)" << std::endl;
            std::cout << "     ";
            for(int x = -m_kernelRadius; x <= m_kernelRadius; ++x) {
                std::cout << " " << std::setw(3) << x;
            }
            std::cout << "\n---------------------------------------------------------------------\n" << std::endl;
        }
    }
}

using TSDF3D16 = TSDF3D<uint16_t>;
using TSDF3D32 = TSDF3D<uint32_t>;

#endif
