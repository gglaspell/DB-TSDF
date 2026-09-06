#include <db_tsdf/volume.hpp>
#include <iostream>
#include <functional>

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void throws(const std::function<void()>& fn) {
    try { fn(); } catch (const std::exception&) { return; }
    throw std::runtime_error("invalid operation was accepted");
}
template<class Mask> void check() {
    Volume<Mask> grid;
    grid.setup(0,1,0,1,0,1,0.03,1);
    require(grid.activeCount()==0,"setup eagerly allocated voxels");
    grid.allocCell(0.5,0.5,0.5);
    require(&grid(0.999,0.001,0.001)!=&grid(0.001,0.031,0.001),"3 cm voxels alias");
    grid(0.999,0.999,0.999).hits=42;
    require(grid.read(0.999,0.999,0.999).hits==42,"3 cm upper corner read failed");
    require(grid.read(1,1,1).hits==0,"outside read is not unknown");
    require(grid.read(-0.001,0,0).hits==0,"negative outside read is not unknown");
    throws([&]{grid(1,0,0).hits=1;});
    const auto copy=grid;
    grid.clear();
    require(copy.read(0.999,0.999,0.999).hits==42,"snapshot shared storage");
    require(grid.activeCount()==0,"clear retained active blocks");
    grid.setup(-2,2,-2,2,-2,2,0.05,1);
    grid.allocCell(-0.01,-0.01,-0.01);
    grid(-0.01,-0.01,-0.01).hits=7;
    require(grid.read(-0.01,-0.01,-0.01).hits==7,"negative coordinates broken");
    grid.allocCell(1,1,1);
    require(grid.rejectedBlocks()==1 && grid.evictions()==0,"stop policy lost old map");
    require(grid.read(-0.01,-0.01,-0.01).hits==7,"stop policy evicted block");
    grid.setup(-2,2,-2,2,-2,2,0.05,1,"rolling");
    grid.allocCell(-1,-1,-1); grid(-1,-1,-1).hits=7;
    grid.allocCell(1,1,1);
    require(grid.evictions()==1 && grid.read(-1,-1,-1).hits==0,"rolling policy did not evict");
    throws([&]{grid.setup(0,1,0,1,0,1,0,1);});
    throws([&]{grid.setup(0,1,0,1,0,1,0.05,0);});
    throws([&]{grid.setup(1,0,0,1,0,1,0.05,1);});
    throws([&]{grid.setup(0,1,0,1,0,1,0.05,1,"unknown");});
    throws([&]{grid.setup(0,10,0,10,0,10,0.01,1000,"stop",1);});
}
int main() {
    try { check<uint16_t>(); check<uint32_t>(); }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
