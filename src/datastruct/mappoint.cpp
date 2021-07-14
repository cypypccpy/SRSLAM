#include "srslam/datastruct/mappoint.h"
#include "srslam/datastruct/feature.h"

mappoint::mappoint(long id, Eigen::Vector3d position) : id_(id), pos_(position) {}

std::shared_ptr<mappoint> mappoint::CreateNewMappoint() {
    static long factory_id = 0;
    std::shared_ptr<mappoint> new_mappoint(new mappoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}

void mappoint::RemoveObservation(std::shared_ptr<feature> feat) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    for (auto iter = observations_.begin(); iter != observations_.end();
         iter++) {
        if (iter->lock() == feat) {
            observations_.erase(iter);
            feat->map_point_.reset();
            observed_times_--;
            break;
        }
    }
}

