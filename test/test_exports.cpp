#include <db_tsdf/tsdf3d.hpp>
#include <db_tsdf/export_worker.hpp>
#include <chrono>
#include <future>
#include <iostream>

void require(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template<class Mask> void check(const std::filesystem::path& dir) {
    Grid<Mask> grid;
    grid.setup(0,3,0,3,0,3,0.1,27);
    for(int z=0;z<30;++z) for(int y=0;y<30;++y) for(int x=0;x<30;++x) {
        const typename Grid<Mask>::Index p{x,y,z};
        grid.allocateBlock(grid.blockId(p));
        auto& v=grid.at(p);
        v.d=db_tsdf::rankMask<Mask>(x>=15 ? x-14 : 15-x);
        v.s=x>=15 ? 2 : 3;
        v.hits=1;
    }
    grid.configureMesh("distance",0);
    auto mesh=grid.buildSurfaceMesh(0,1);
    require(mesh->GetNumberOfPolys()==2*29*29,"mesh has missing/duplicate triangles at block seams");
    double bounds[6]; mesh->GetBounds(bounds);
    require(std::abs(bounds[0]-1.5)<1e-5 && std::abs(bounds[1]-1.5)<1e-5,"distance mesh moved the plane");
    const auto path=(dir/(std::to_string(sizeof(Mask))+".dbtsdf")).string();
    const auto hash=grid.stateHash();
    grid.saveCheckpoint(path,"frame=test");
    grid.clear(); grid.loadCheckpoint(path,"frame=test");
    require(grid.stateHash()==hash,"checkpoint round trip changed voxel state");
    bool rejected=false;
    try { grid.loadCheckpoint(path,"frame=wrong"); } catch(const std::exception&) { rejected=true; }
    require(rejected && grid.stateHash()==hash,"incompatible checkpoint changed the live map");
    // Alter an otherwise valid field, without truncating the file.
    {
        std::fstream stream(path,std::ios::in|std::ios::out|std::ios::binary);
        stream.seekp(-1,std::ios::end); stream.put(char(2));
    }
    rejected=false;
    try { grid.loadCheckpoint(path,"frame=test"); } catch(const std::exception&) { rejected=true; }
    require(rejected && grid.stateHash()==hash,"corrupt checkpoint changed the live map");
    grid.saveCheckpoint(path,"frame=test");
    std::filesystem::resize_file(path,std::filesystem::file_size(path)-3);
    rejected=false;
    try { grid.loadCheckpoint(path,"frame=test"); } catch(const std::exception&) { rejected=true; }
    require(rejected && grid.stateHash()==hash,"truncated checkpoint changed the live map");
    grid.clear(); rejected=false;
    try { grid.exportGridToPCD((dir/"empty.pcd").string(),1); } catch(const std::exception&) { rejected=true; }
    require(rejected && !std::filesystem::exists(dir/"empty.pcd"),"empty export reported success");
    rejected=false;
    try { grid.exportMesh((dir/"empty.stl").string(),0,1); } catch(const std::exception&) { rejected=true; }
    require(rejected && !std::filesystem::exists(dir/"empty.stl"),"empty mesh export reported success");

    // A rolling checkpoint must preserve which slot gets evicted next.
    grid.setup(0,3,0,1,0,1,.1,2,"rolling");
    grid.allocateBlock(0); grid.allocateBlock(1); grid.allocateBlock(2);
    grid.saveCheckpoint(path,"rolling");
    auto restored=grid; restored.clear(); restored.loadCheckpoint(path,"rolling");
    grid.allocateBlock(0); restored.allocateBlock(0);
    require(grid.activeBlocks()==restored.activeBlocks() && grid.evictions()==restored.evictions(),
        "checkpoint changed rolling eviction order");
}
void workerCheck() {
    db_tsdf::ExportWorker worker(2);
    std::promise<void> started, release;
    auto gate=release.get_future().share();
    require(worker.enqueue("one",[&]{started.set_value();gate.wait();})!=0,"first job rejected");
    started.get_future().wait();
    require(worker.enqueue("two",[]{throw std::runtime_error("expected failure");})!=0,"second job rejected");
    require(worker.enqueue("three",[]{})==0,"queue was not bounded");
    release.set_value(); worker.shutdown();
    const auto jobs=worker.statuses();
    require(jobs.size()==2 && jobs[0].state=="complete" && jobs[1].state=="failed" &&
        jobs[1].error=="expected failure","worker completion/error reporting failed");
}
int main() {
    const auto dir=std::filesystem::temp_directory_path()/
        ("db-tsdf-exports-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(dir);
    try { check<uint16_t>(dir); check<uint32_t>(dir); workerCheck(); }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; std::filesystem::remove_all(dir); return 1; }
    std::filesystem::remove_all(dir);
}
