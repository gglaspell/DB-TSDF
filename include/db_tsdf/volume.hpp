#ifndef DB_TSDF_VOLUME_HPP
#define DB_TSDF_VOLUME_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <db_tsdf/mask.hpp>

template <db_tsdf::SupportedMask Mask>
struct VoxelDataT {
    Mask d{db_tsdf::full_mask_v<Mask>};
    uint8_t s{1};  // bit 0: free; bit 1: observed by a kernel
    uint8_t hits{0};
};
static_assert(sizeof(VoxelDataT<uint16_t>) == 4);
static_assert(sizeof(VoxelDataT<uint32_t>) == 8);

// Value semantics deliberately provide a deep, independent map snapshot.
// Allocation and mutation must be externally serialized. Distinct blocks can
// be updated concurrently after allocation has finished.
template <db_tsdf::SupportedMask Mask>
class Volume {
public:
    using VoxelData = VoxelDataT<Mask>;
    using Index = std::array<int, 3>;
    static constexpr int mask_bits = db_tsdf::mask_bits_v<Mask>;

    static size_t checkedProduct(size_t a, size_t b) {
        if (b && a > std::numeric_limits<size_t>::max() / b)
            throw std::invalid_argument("grid size overflows addressable memory");
        return a * b;
    }

    void setup(double minX, double maxX, double minY, double maxY,
               double minZ, double maxZ, double resolution = 0.05,
               int maxCells = 100000, const std::string& policy = "stop",
               double memoryBudgetMiB = 0) {
        if (!std::isfinite(resolution) || resolution <= 0 || maxCells <= 0)
            throw std::invalid_argument("resolution and max_cells must be positive");
        if (!std::isfinite(memoryBudgetMiB) || memoryBudgetMiB < 0)
            throw std::invalid_argument("memory_budget_mb must be finite and nonnegative");
        if (policy != "stop" && policy != "rolling")
            throw std::invalid_argument("capacity_policy must be stop or rolling");
        Volume next;
        next._min = {minX, minY, minZ};
        next._max = {maxX, maxY, maxZ};
        next._resolution = resolution;
        // Preserve approximately 1 m blocks, including exactly 20 voxels at
        // 5 cm. Block extent is always an integer multiple of resolution.
        const double inverse = 1.0 / resolution;
        const double nearest = std::round(inverse);
        const double side = std::max(1.0, std::abs(inverse-nearest) <= 1e-6*inverse ? nearest : std::ceil(inverse));
        if (side > 1024)
            throw std::invalid_argument("resolution requires more than 1024 voxels per block edge");
        next._side = static_cast<int>(side);
        size_t count = 1;
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(next._min[a]) || !std::isfinite(next._max[a]) ||
                next._min[a] >= next._max[a])
                throw std::invalid_argument("grid bounds must be finite and ordered");
            const double cells = std::ceil((next._max[a] - next._min[a]) / resolution);
            if (!std::isfinite(cells) || cells > std::numeric_limits<int>::max() - 1024)
                throw std::invalid_argument("grid axis exceeds integer coordinate range");
            next._dims[a] = std::max(1,static_cast<int>(cells));
            next._blockDims[a] = (next._dims[a] + next._side - 1) / next._side;
            count = checkedProduct(count, next._blockDims[a]);
        }
        if (count > static_cast<size_t>(std::numeric_limits<int32_t>::max()))
            throw std::invalid_argument("too many block addresses; reduce grid bounds");
        next._blockVoxels = checkedProduct(checkedProduct(next._side, next._side), next._side);
        next._capacity = std::min(count, static_cast<size_t>(maxCells));
        const size_t tableBytes = checkedProduct(count, sizeof(int32_t));
        const size_t poolBytes = checkedProduct(next._capacity,
            checkedProduct(next._blockVoxels, sizeof(VoxelData)));
        if (poolBytes > std::numeric_limits<size_t>::max() - tableBytes)
            throw std::invalid_argument("grid byte estimate overflows");
        next._estimatedBytes = poolBytes + tableBytes;
        if (memoryBudgetMiB > 0 && next._estimatedBytes / 1048576.0 > memoryBudgetMiB)
            throw std::invalid_argument("grid capacity exceeds memory_budget_mb (estimated " +
                std::to_string(next._estimatedBytes / 1048576.0) + " MiB)");
        next._policy = policy;
        next._table.assign(count, -1);
        *this = std::move(next);
    }

    void clear() {
        std::fill(_table.begin(), _table.end(), -1);
        _blocks.clear();
        _ids.clear();
        _nextEviction = _evictions = _rejected = 0;
    }

    bool contains(double x, double y, double z) const {
        return x >= _min[0] && x < _max[0] && y >= _min[1] && y < _max[1] &&
               z >= _min[2] && z < _max[2];
    }
    Index index(double x, double y, double z) const {
        if (!contains(x, y, z)) throw std::out_of_range("point outside grid bounds");
        const std::array<double,3> p{x,y,z};
        Index out;
        for (int a=0; a<3; ++a)
            out[a] = std::min(_dims[a]-1, static_cast<int>(std::floor((p[a]-_min[a])/_resolution)));
        return out;
    }
    bool valid(const Index& p) const {
        return p[0]>=0 && p[1]>=0 && p[2]>=0 &&
               p[0]<_dims[0] && p[1]<_dims[1] && p[2]<_dims[2];
    }
    std::array<double,3> center(const Index& p) const {
        return {_min[0]+(p[0]+0.5)*_resolution,
                _min[1]+(p[1]+0.5)*_resolution,
                _min[2]+(p[2]+0.5)*_resolution};
    }
    uint32_t blockId(const Index& p) const {
        return static_cast<uint32_t>(p[0]/_side +
            (p[1]/_side + static_cast<size_t>(p[2]/_side)*_blockDims[1])*_blockDims[0]);
    }
    Index blockOrigin(uint32_t id) const {
        return {static_cast<int>(id % _blockDims[0])*_side,
                static_cast<int>((id / _blockDims[0]) % _blockDims[1])*_side,
                static_cast<int>(id / (_blockDims[0]*static_cast<size_t>(_blockDims[1])))*_side};
    }
    size_t offset(const Index& p) const {
        return p[0]%_side + (p[1]%_side + static_cast<size_t>(p[2]%_side)*_side)*_side;
    }
    VoxelData* blockData(uint32_t id) {
        return id < _table.size() && _table[id] >= 0 ? _blocks[_table[id]].data() : nullptr;
    }
    const VoxelData* blockData(uint32_t id) const {
        return id < _table.size() && _table[id] >= 0 ? _blocks[_table[id]].data() : nullptr;
    }
    bool allocateBlock(uint32_t id) {
        if (id >= _table.size()) throw std::out_of_range("block outside grid");
        if (_table[id] >= 0) return true;
        size_t slot = _blocks.size();
        if (slot == _capacity) {
            if (_policy == "stop") { ++_rejected; return false; }
            slot = _nextEviction++ % _capacity;
            _table[_ids[slot]] = -1;
            ++_evictions;
            std::fill(_blocks[slot].begin(), _blocks[slot].end(), VoxelData{});
            _ids[slot] = id;
        } else {
            _blocks.emplace_back(_blockVoxels);
            try { _ids.push_back(id); }
            catch (...) { _blocks.pop_back(); throw; }
        }
        _table[id] = static_cast<int32_t>(slot);
        return true;
    }
    void allocCell(float x, float y, float z) { allocateBlock(blockId(index(x,y,z))); }
    VoxelData& at(const Index& p) {
        if (!valid(p)) throw std::out_of_range("voxel outside grid");
        auto* block = blockData(blockId(p));
        if (!block) throw std::out_of_range("voxel block is not allocated");
        return block[offset(p)];
    }
    VoxelData& operator()(float x, float y, float z) { return at(index(x,y,z)); }
    VoxelData readIndex(const Index& p) const {
        if (!valid(p)) return {};
        const auto* block = blockData(blockId(p));
        return block ? block[offset(p)] : VoxelData{};
    }
    VoxelData read(double x, double y, double z) const {
        return contains(x,y,z) ? readIndex(index(x,y,z)) : VoxelData{};
    }
    struct Iterator {
        Volume* parent;
        Index p;
        VoxelData& operator*() { return parent->at(p); }
        VoxelData* operator->() { return &parent->at(p); }
        Iterator& operator++() { ++p[0]; return *this; }
    };
    Iterator getIterator(float x,float y,float z) { return {this,index(x,y,z)}; }

    std::vector<uint32_t> activeBlocks() const {
        auto out = _ids;
        std::sort(out.begin(),out.end());
        return out;
    }
    template <class Visitor> void visit(Visitor&& visitor) const {
        for (auto id : activeBlocks()) {
            const auto origin = blockOrigin(id);
            const auto* data = blockData(id);
            for (int z=0; z<_side; ++z)
            for (int y=0; y<_side; ++y)
            for (int x=0; x<_side; ++x) {
                const Index p{origin[0]+x,origin[1]+y,origin[2]+z};
                if (valid(p)) visitor(p,data[x+(y+static_cast<size_t>(z)*_side)*_side]);
            }
        }
    }
    uint64_t stateHash() const {
        uint64_t hash = 14695981039346656037ull;
        auto add = [&](uint64_t v, int bytes) {
            for (int i=0;i<bytes;++i) { hash ^= (v >> (8*i)) & 255; hash *= 1099511628211ull; }
        };
        // Canonical ordering and explicit fields exclude padding and allocation order.
        visit([&](const Index& p,const VoxelData& v) {
            if (!(v.s & 2)) return;
            for (int a : p) add(a,4);
            add(v.d,sizeof(Mask)); add(v.s,1); add(v.hits,1);
        });
        return hash;
    }
    int blockSide() const { return _side; }
    const Index& dimensions() const { return _dims; }
    double resolution() const { return _resolution; }
    size_t activeCount() const { return _blocks.size(); }
    size_t capacity() const { return _capacity; }
    size_t estimatedBytes() const { return _estimatedBytes; }
    size_t allocatedBytes() const { return _table.size()*sizeof(int32_t)+_blocks.size()*_blockVoxels*sizeof(VoxelData); }
    uint64_t evictions() const { return _evictions; }
    uint64_t rejectedBlocks() const { return _rejected; }

    std::string checkpointSignature(const std::string& extra) const {
        std::ostringstream out;
        out << std::setprecision(17) << _resolution << ' ' << _side << ' ' << _capacity << ' ' << _policy;
        for (double v : _min) out << ' ' << v;
        for (double v : _max) out << ' ' << v;
        return out.str()+"\n"+extra;
    }
    uint64_t checkpointHash(const std::string& extra) const {
        uint64_t hash=14695981039346656037ull;
        auto add=[&](uint64_t value,int bytes) {
            for (int i=0;i<bytes;++i) { hash^=(value>>(8*i))&255; hash*=1099511628211ull; }
        };
        add(mask_bits,1);
        for (unsigned char c : checkpointSignature(extra)) add(c,1);
        add(_blocks.size(),8); add(_nextEviction,8); add(_evictions,8); add(_rejected,8);
        for (size_t b=0;b<_blocks.size();++b) {
            add(_ids[b],4);
            for (const auto& v : _blocks[b]) { add(v.d,sizeof(Mask)); add(v.s,1); add(v.hits,1); }
        }
        return hash;
    }
    void saveCheckpoint(const std::string& path,const std::string& extra={}) const {
        std::ofstream out(path,std::ios::binary);
        out.exceptions(std::ios::badbit | std::ios::failbit);
        const auto signature=checkpointSignature(extra);
        out << "DBTSDF2\n" << mask_bits << ' ' << signature.size() << ' ' << _blocks.size()
            << ' ' << _nextEviction << ' ' << _evictions << ' ' << _rejected << ' ' << checkpointHash(extra) << '\n';
        out.write(signature.data(),signature.size());
        auto integer = [&](uint32_t value,int bytes) {
            for (int i=0;i<bytes;++i) out.put(static_cast<char>((value>>(8*i))&255));
        };
        // Explicit little-endian fields exclude compiler padding. Slot order
        // and eviction cursor are preserved so resumed rolling maps agree.
        for (size_t b=0;b<_blocks.size();++b) {
            integer(_ids[b],4);
            for (const auto& v : _blocks[b]) {
                integer(v.d,sizeof(Mask)); integer(v.s,1); integer(v.hits,1);
            }
        }
        out.close();
    }
    void loadCheckpoint(const std::string& path,const std::string& extra={}) {
        std::ifstream in(path,std::ios::binary);
        in.exceptions(std::ios::badbit | std::ios::failbit);
        std::string magic;
        std::getline(in,magic);
        unsigned bits;
        size_t signatureBytes,count;
        uint64_t cursor,evictions,rejected,hash;
        in >> bits >> signatureBytes >> count >> cursor >> evictions >> rejected >> hash;
        if (in.get()!='\n' || magic!="DBTSDF2" || bits!=mask_bits || count>_capacity || signatureBytes>16384)
            throw std::runtime_error("invalid checkpoint header or backend width");
        std::string signature(signatureBytes,'\0');
        in.read(signature.data(),signature.size());
        if (signature!=checkpointSignature(extra))
            throw std::runtime_error("checkpoint configuration/frame does not match this mapper");
        Volume next;
        next.setup(_min[0],_max[0],_min[1],_max[1],_min[2],_max[2],_resolution,
                   static_cast<int>(_capacity),_policy);
        auto integer = [&](int bytes) {
            uint32_t value=0;
            for (int i=0;i<bytes;++i) value |= static_cast<uint32_t>(static_cast<unsigned char>(in.get()))<<(8*i);
            return value;
        };
        for (size_t b=0;b<count;++b) {
            const auto id=integer(4);
            if (id>=_table.size() || next.blockData(id)) throw std::runtime_error("invalid checkpoint block ID");
            next.allocateBlock(id);
            auto* block=next.blockData(id);
            for (size_t j=0;j<_blockVoxels;++j) {
                auto& v=block[j];
                v.d=static_cast<Mask>(integer(sizeof(Mask)));
                v.s=integer(1); v.hits=integer(1);
                if (v.s>3 || (!(v.s&2) && (v.s!=1 || v.hits!=0 || v.d!=db_tsdf::full_mask_v<Mask>)) ||
                    db_tsdf::rankMask<Mask>(db_tsdf::maskRank(v.d))!=v.d)
                    throw std::runtime_error("invalid checkpoint voxel encoding");
            }
        }
        if (in.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("checkpoint has trailing data");
        next._nextEviction=cursor; next._evictions=evictions; next._rejected=rejected;
        if (next.checkpointHash(extra)!=hash) throw std::runtime_error("checkpoint checksum mismatch");
        *this=std::move(next);
    }

protected:
    std::array<double,3> _min{}, _max{};
    double _resolution{0.05};
    Index _dims{}, _blockDims{};
    int _side{20};
    size_t _blockVoxels{8000}, _capacity{0}, _estimatedBytes{0};
    std::string _policy{"stop"};
    std::vector<int32_t> _table;
    std::vector<std::vector<VoxelData>> _blocks;
    std::vector<uint32_t> _ids;
    uint64_t _nextEviction{0}, _evictions{0}, _rejected{0};
};

#endif
