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

#include <basalt/calibration/calib_cache.h>

#include <basalt/serialization/headers_serialization.h>

#include <fstream>
#include <iostream>

namespace basalt {

namespace {

// On-disk format (cereal binary):
//   v2: magic, version, cam_names (union list; TimeCamId.cam_id indexes it),
//       then the payload maps.
//   legacy (before sensor filtering): payload maps only, cam_id is the
//   positional camera index of the run that wrote the file.
const std::string CACHE_MAGIC = "basalt_calib_cache";
constexpr uint32_t CACHE_VERSION = 2;

enum class Envelope { V2, Legacy, UnsupportedVersion };

// Legacy files start with the u64 entry count of the first map, which cereal
// reads as a string length: this yields either a garbage string or an
// exception when the "length" exceeds the file size. Both mean legacy.
Envelope readEnvelope(cereal::BinaryInputArchive &archive,
                      std::vector<std::string> &file_cam_names) {
  std::string magic;
  try {
    archive(magic);
  } catch (const std::exception &) {
    return Envelope::Legacy;
  }
  if (magic != CACHE_MAGIC) return Envelope::Legacy;

  uint32_t version = 0;
  archive(version);
  if (version != CACHE_VERSION) return Envelope::UnsupportedVersion;

  archive(file_cam_names);
  return Envelope::V2;
}

std::map<std::string, size_t> buildIndex(
    const std::vector<std::string> &names) {
  std::map<std::string, size_t> index;
  for (size_t i = 0; i < names.size(); i++) index[names[i]] = i;
  return index;
}

// Route the entries of a map read from file either into the live map
// (cameras of the current selection, remapped to current indices) or into
// the dormant store (cameras not currently selected). Returns false if an
// entry references a camera not in the file's name list (corrupt file).
template <typename Map, typename DormantStore>
bool distribute(const Map &file_map, const std::vector<std::string> &file_names,
                const std::map<std::string, size_t> &current_index, Map &live,
                DormantStore &dormant) {
  for (const auto &kv : file_map) {
    if (kv.first.cam_id >= file_names.size()) return false;
    const std::string &name = file_names[kv.first.cam_id];
    auto it = current_index.find(name);
    if (it != current_index.end()) {
      live.emplace(TimeCamId(kv.first.frame_id, it->second), kv.second);
    } else {
      dormant[name][kv.first.frame_id] = kv.second;
    }
  }
  return true;
}

// Append dormant entries to an output map using the union name index.
// Names that are currently selected are skipped: the live map wins.
template <typename DormantStore, typename Map>
void appendDormant(const DormantStore &store,
                   const std::map<std::string, size_t> &union_index,
                   size_t num_current, Map &out) {
  for (const auto &per_name : store) {
    const size_t idx = union_index.at(per_name.first);
    if (idx < num_current) continue;
    for (const auto &fkv : per_name.second) {
      out.emplace(TimeCamId(fkv.first, idx), fkv.second);
    }
  }
}

// Union name list: current names first (so live entries keep their indices),
// then dormant-only names in deterministic (sorted) order.
template <typename... DormantStores>
std::vector<std::string> buildUnionNames(
    const std::vector<std::string> &cam_names,
    std::map<std::string, size_t> &union_index,
    const DormantStores &...stores) {
  std::vector<std::string> union_names = cam_names;
  union_index = buildIndex(cam_names);
  for (const auto *store : {&stores...}) {
    for (const auto &per_name : *store) {
      if (union_index.count(per_name.first) == 0) {
        union_index[per_name.first] = union_names.size();
        union_names.push_back(per_name.first);
      }
    }
  }
  return union_names;
}

void writeEnvelope(cereal::BinaryOutputArchive &archive,
                   const std::vector<std::string> &union_names) {
  std::string magic = CACHE_MAGIC;
  uint32_t version = CACHE_VERSION;
  archive(magic);
  archive(version);
  archive(union_names);
}

template <typename Map>
bool legacyIndicesValid(const Map &map, size_t num_cams) {
  for (const auto &kv : map) {
    if (kv.first.cam_id >= num_cams) return false;
  }
  return true;
}

}  // namespace

CacheLoadResult loadCornerCache(const std::string &path,
                                const std::vector<std::string> &cam_names,
                                bool sensor_filter_active,
                                CalibCornerMap &calib_corners,
                                CalibCornerMap &calib_corners_rejected,
                                DormantCorners &dormant) {
  calib_corners.clear();
  calib_corners_rejected.clear();
  dormant = DormantCorners();

  std::ifstream is(path, std::ios::binary);
  if (!is.good()) return CacheLoadResult::Missing;

  try {
    Envelope env;
    {
      cereal::BinaryInputArchive archive(is);
      std::vector<std::string> file_names;
      env = readEnvelope(archive, file_names);

      if (env == Envelope::V2) {
        CalibCornerMap corners_in, rejected_in;
        archive(corners_in);
        archive(rejected_in);

        const auto index = buildIndex(cam_names);
        if (!distribute(corners_in, file_names, index, calib_corners,
                        dormant.corners) ||
            !distribute(rejected_in, file_names, index, calib_corners_rejected,
                        dormant.rejected)) {
          throw std::runtime_error("camera id out of range");
        }
        return CacheLoadResult::Loaded;
      }
    }

    if (env == Envelope::UnsupportedVersion) {
      std::cerr << "Warning: corner cache " << path
                << " has an unsupported version, ignoring it." << std::endl;
      return CacheLoadResult::Corrupt;
    }

    // legacy format: index-keyed, no camera names stored
    if (sensor_filter_active) {
      std::cerr << "Warning: corner cache " << path
                << " predates sensor filtering (no camera names stored); "
                   "ignoring it because --include/--exclude is active. "
                   "Corners will be re-detected and the cache rewritten with "
                   "camera names."
                << std::endl;
      return CacheLoadResult::IgnoredLegacyFiltered;
    }

    // the envelope probe consumed bytes and cereal cannot seek: reopen
    std::ifstream is_legacy(path, std::ios::binary);
    cereal::BinaryInputArchive archive(is_legacy);
    archive(calib_corners);
    archive(calib_corners_rejected);

    if (!legacyIndicesValid(calib_corners, cam_names.size()) ||
        !legacyIndicesValid(calib_corners_rejected, cam_names.size())) {
      throw std::runtime_error(
          "camera id out of range (cache from a different camera setup?)");
    }
    return CacheLoadResult::LoadedLegacy;
  } catch (const std::exception &e) {
    calib_corners.clear();
    calib_corners_rejected.clear();
    dormant = DormantCorners();
    std::cerr << "Warning: corner cache " << path
              << " is corrupt or has an unknown format, ignoring it ("
              << e.what() << ")." << std::endl;
    return CacheLoadResult::Corrupt;
  }
}

void saveCornerCache(const std::string &path,
                     const std::vector<std::string> &cam_names,
                     const CalibCornerMap &calib_corners,
                     const CalibCornerMap &calib_corners_rejected,
                     const DormantCorners &dormant) {
  std::map<std::string, size_t> union_index;
  const std::vector<std::string> union_names = buildUnionNames(
      cam_names, union_index, dormant.corners, dormant.rejected);

  CalibCornerMap corners_out = calib_corners;
  CalibCornerMap rejected_out = calib_corners_rejected;
  appendDormant(dormant.corners, union_index, cam_names.size(), corners_out);
  appendDormant(dormant.rejected, union_index, cam_names.size(), rejected_out);

  std::ofstream os(path, std::ios::binary);
  if (!os.is_open()) {
    std::cerr << "Warning: could not write corner cache " << path << std::endl;
    return;
  }
  cereal::BinaryOutputArchive archive(os);
  writeEnvelope(archive, union_names);
  archive(corners_out);
  archive(rejected_out);
}

CacheLoadResult loadPoseCache(const std::string &path,
                              const std::vector<std::string> &cam_names,
                              bool sensor_filter_active,
                              CalibInitPoseMap &calib_init_poses,
                              DormantPoses &dormant) {
  calib_init_poses.clear();
  dormant = DormantPoses();

  std::ifstream is(path, std::ios::binary);
  if (!is.good()) return CacheLoadResult::Missing;

  try {
    Envelope env;
    {
      cereal::BinaryInputArchive archive(is);
      std::vector<std::string> file_names;
      env = readEnvelope(archive, file_names);

      if (env == Envelope::V2) {
        CalibInitPoseMap poses_in;
        archive(poses_in);

        const auto index = buildIndex(cam_names);
        if (!distribute(poses_in, file_names, index, calib_init_poses,
                        dormant.poses)) {
          throw std::runtime_error("camera id out of range");
        }
        return CacheLoadResult::Loaded;
      }
    }

    if (env == Envelope::UnsupportedVersion) {
      std::cerr << "Warning: init-pose cache " << path
                << " has an unsupported version, ignoring it." << std::endl;
      return CacheLoadResult::Corrupt;
    }

    if (sensor_filter_active) {
      std::cerr << "Warning: init-pose cache " << path
                << " predates sensor filtering (no camera names stored); "
                   "ignoring it because --include/--exclude is active. "
                   "Poses will be re-initialized and the cache rewritten "
                   "with camera names."
                << std::endl;
      return CacheLoadResult::IgnoredLegacyFiltered;
    }

    std::ifstream is_legacy(path, std::ios::binary);
    cereal::BinaryInputArchive archive(is_legacy);
    archive(calib_init_poses);

    if (!legacyIndicesValid(calib_init_poses, cam_names.size())) {
      throw std::runtime_error(
          "camera id out of range (cache from a different camera setup?)");
    }
    return CacheLoadResult::LoadedLegacy;
  } catch (const std::exception &e) {
    calib_init_poses.clear();
    dormant = DormantPoses();
    std::cerr << "Warning: init-pose cache " << path
              << " is corrupt or has an unknown format, ignoring it ("
              << e.what() << ")." << std::endl;
    return CacheLoadResult::Corrupt;
  }
}

void savePoseCache(const std::string &path,
                   const std::vector<std::string> &cam_names,
                   const CalibInitPoseMap &calib_init_poses,
                   const DormantPoses &dormant) {
  std::map<std::string, size_t> union_index;
  const std::vector<std::string> union_names =
      buildUnionNames(cam_names, union_index, dormant.poses);

  CalibInitPoseMap poses_out = calib_init_poses;
  appendDormant(dormant.poses, union_index, cam_names.size(), poses_out);

  std::ofstream os(path, std::ios::binary);
  if (!os.is_open()) {
    std::cerr << "Warning: could not write init-pose cache " << path
              << std::endl;
    return;
  }
  cereal::BinaryOutputArchive archive(os);
  writeEnvelope(archive, union_names);
  archive(poses_out);
}

}  // namespace basalt
