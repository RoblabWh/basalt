/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <basalt/calibration/calibration_helper.h>

#include <map>
#include <string>
#include <vector>

namespace basalt {

// Corner and init-pose caches are stored keyed by camera NAME (see
// VioDataset::get_cam_names), so one cache file per dataset works for every
// --include/--exclude sensor selection. In memory the maps stay keyed by the
// current camera index; entries of currently deselected cameras are parked in
// the dormant stores below and merged back into the file on save, so the
// cache accumulates the union of everything ever detected for the dataset.

enum class CacheLoadResult {
  Loaded,        // new (name-keyed) format
  LoadedLegacy,  // old index-keyed format; valid because no filter is active
  Missing,
  Corrupt,                // unreadable or inconsistent; ignored
  IgnoredLegacyFiltered,  // old format + active sensor filter; ignored
};

// Cache entries belonging to cameras that are not currently selected,
// keyed by camera name, then frame timestamp.
struct DormantCorners {
  std::map<std::string, std::map<int64_t, CalibCornerData>> corners;
  std::map<std::string, std::map<int64_t, CalibCornerData>> rejected;
};

struct DormantPoses {
  std::map<std::string, std::map<int64_t, CalibInitPoseData>> poses;
};

CacheLoadResult loadCornerCache(const std::string &path,
                                const std::vector<std::string> &cam_names,
                                bool sensor_filter_active,
                                CalibCornerMap &calib_corners,
                                CalibCornerMap &calib_corners_rejected,
                                DormantCorners &dormant);

void saveCornerCache(const std::string &path,
                     const std::vector<std::string> &cam_names,
                     const CalibCornerMap &calib_corners,
                     const CalibCornerMap &calib_corners_rejected,
                     const DormantCorners &dormant);

CacheLoadResult loadPoseCache(const std::string &path,
                              const std::vector<std::string> &cam_names,
                              bool sensor_filter_active,
                              CalibInitPoseMap &calib_init_poses,
                              DormantPoses &dormant);

void savePoseCache(const std::string &path,
                   const std::vector<std::string> &cam_names,
                   const CalibInitPoseMap &calib_init_poses,
                   const DormantPoses &dormant);

}  // namespace basalt
