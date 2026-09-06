#include <db_tsdf/tsdf3d.hpp>
#include <iostream>
#include <limits>

void require(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template<class Mask> struct SerialReference : TSDF3D<Mask> {
    void integrate(const std::vector<pcl::PointXYZ>& points,const Eigen::Vector3f& sensor) {
        for (const auto& point : points) {
            const Eigen::Vector3f ray=Eigen::Vector3f(point.x,point.y,point.z)-sensor;
            if (!ray.allFinite() || ray.squaredNorm()<=1e-12f || !this->m_grid.contains(point.x,point.y,point.z)) continue;
            const auto center=this->m_grid.index(point.x,point.y,point.z);
            const auto& signs=this->m_bank->kernels[this->dirToBin(ray)].signs;
            int k=0;
            for (int z=-this->m_kernelRadius;z<=this->m_kernelRadius;++z)
            for (int y=-this->m_kernelRadius;y<=this->m_kernelRadius;++y)
            for (int x=-this->m_kernelRadius;x<=this->m_kernelRadius;++x,++k) {
                const typename Grid<Mask>::Index p{center[0]+x,center[1]+y,center[2]+z};
                if (!this->m_grid.valid(p)) continue;
                require(this->m_grid.allocateBlock(this->m_grid.blockId(p)),"reference grid is full");
                auto& voxel=this->m_grid.at(p);
                voxel.d&=this->m_bank->distances[k]; voxel.s|=2;
                if (!signs[k]) {
                    voxel.hits=std::min(int(voxel.hits)+1,this->m_occMinHits);
                    if (voxel.hits==this->m_occMinHits) voxel.s&=uint8_t(~1u);
                }
            }
        }
    }
};
template<class Mask> void check() {
    TSDF3D<Mask> mapper;
    mapper.setup(-3,3,-3,3,-3,3,0.1,3,64,8,8,3,"L1",216,false,1);
    std::vector<pcl::PointXYZ> points(128,pcl::PointXYZ(0.25,0.25,0.25));
    const Eigen::Vector3f sensor(-1,0.25,0.25);
    mapper.loadCloud(points,sensor);
    auto v=mapper.grid().read(0.25,0.25,0.25);
    require(v.hits==64 && !(v.s&1) && v.d==0,"duplicate returns did not saturate occupied voxel");
    // Deliberate overlaps across block boundaries, with different ray bins.
    for(int z=-6;z<=6;++z) for(int y=-8;y<=8;++y)
        points.emplace_back(0.98f+0.017f*(y%3),y*0.17f,z*0.13f);
    points.emplace_back(std::numeric_limits<float>::quiet_NaN(),0,0);
    points.emplace_back(100,0,0);
    points.emplace_back(sensor.x(),sensor.y(),sensor.z());
    mapper.clear(); mapper.loadCloud(points,sensor);
    const auto reference=mapper.grid().stateHash();
    SerialReference<Mask> oracle;
    oracle.setup(-3,3,-3,3,-3,3,0.1,3,64,8,8,3,"L1",216,false,1);
    oracle.integrate(points,sensor);
    require(oracle.grid().stateHash()==reference,"block integration disagrees with serial point reference");
    require(mapper.lastIntegration().invalid_points==2,"invalid returns not counted");
    require(mapper.lastIntegration().outside_points==1,"outside returns not counted");
    for (int threads : {1,2,4,16}) {
        mapper.setThreadCount(threads);
        for(int run=0;run<5;++run) {
            mapper.clear(); mapper.loadCloud(points,sensor);
            require(mapper.grid().stateHash()==reference,"map depends on worker count or scheduling");
        }
    }
    const auto snapshot=mapper;
    bool rejected=false;
    try { mapper.setup(-3,3,-3,3,-3,3,.03,63,64,720,360,3,"L2",216); }
    catch(const std::invalid_argument&) { rejected=true; }
    require(rejected && mapper.grid().stateHash()==reference,"failed setup changed the live map");
    rejected=false;
    try { mapper.setup(-1,1,-1,1,-1,1,.1,5,1,60,60,3,"L2",8,false,1,"stop",.1); }
    catch(const std::invalid_argument&) { rejected=true; }
    require(rejected && mapper.grid().stateHash()==reference,"kernel memory budget failure destroyed the live map");
    mapper.clear();
    require(snapshot.grid().stateHash()==reference,"map snapshot was mutated by clear");
    mapper.setup(-1,1,-1,1,-1,1,0.03,5,1,8,8,3,"L2",8,false,2);
    points={pcl::PointXYZ(0.99,0.99,0.99),pcl::PointXYZ(-0.99,-0.99,-0.99)};
    mapper.loadCloud(points,Eigen::Vector3f(0,0,0));
    require(mapper.grid().read(0.99,0.99,0.99).d==0,"kernel lost near upper boundary");
    require(mapper.grid().read(-0.99,-0.99,-0.99).d==0,"kernel lost near lower boundary");
    require(std::abs(mapper.grid().metricDistance(db_tsdf::rankMask<Mask>(2))-std::sqrt(2)*0.03)<1e-8,
        "L2 rank was treated as metric distance");
    mapper.setup(-1,1,-1,1,-1,1,0.03,3,1,8,8,3,"L2",8,false,1);
    require(mapper.grid().activeCount()==0,"repeated setup retained old map");

    // Analytic signed plane at negative world coordinates, including samples
    // on either side of zero and support spanning a block boundary.
    mapper.setup(-2,2,-2,2,-2,2,.25,3,1,8,8,3,"L1",64,false,1);
    for (int z=0;z<16;++z) for (int y=0;y<16;++y) for (int x=0;x<16;++x) {
        const typename Grid<Mask>::Index p{x,y,z};
        mapper.grid().allocateBlock(mapper.grid().blockId(p));
        auto& v=mapper.grid().at(p);
        v.d=db_tsdf::rankMask<Mask>(x<4?4-x:x-4); v.s=x<4?3:2; v.hits=1;
    }
    for (double x : {-1.2,-1.0,-.875,-.6,.1}) {
        const auto interpolation=mapper.computeDistInterpolation(x,-.9,-.1);
        require(interpolation.valid && std::abs(interpolation.interpolate(x,-.9,-.1)-(-.875-x))<1e-8,
            "metric interpolation biased a negative-coordinate plane");
    }
    require(!mapper.computeDistInterpolation(-2,-1,0).valid,"unknown support produced a valid distance");
}
void compareBackends() {
    TSDF3D<uint16_t> narrow;
    TSDF3D<uint32_t> wide;
    narrow.setup(-2,2,-2,2,-2,2,.03,5,2,16,8,3,"L2",64,false,4);
    wide.setup(-2,2,-2,2,-2,2,.03,5,2,16,8,3,"L2",64,false,4);
    const std::vector<pcl::PointXYZ> points{{.99f,.25f,.13f},{1.01f,.24f,.15f},{-.99f,-.25f,-.13f}};
    for (int i=0;i<3;++i) {
        narrow.loadCloud(points,Eigen::Vector3f(0,0,0));
        wide.loadCloud(points,Eigen::Vector3f(0,0,0));
    }
    require(narrow.grid().activeBlocks()==wide.grid().activeBlocks(),"backend widths allocated different blocks");
    narrow.grid().visit([&](const auto& p,const auto& v) {
        const auto other=wide.grid().readIndex(p);
        require(v.s==other.s && v.hits==other.hits,"backend widths disagree on occupancy");
        if (v.s&2) require(db_tsdf::maskRank(v.d)==db_tsdf::maskRank(other.d),"unsaturated backend distances disagree");
    });
}
int main() {
    try { check<uint16_t>(); check<uint32_t>(); compareBackends(); }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
