## ORB\_SLAM3 Calling Hierarchies

This document summarizes the **high‑level calling and dependency hierarchies** of the main modules in this ORB\_SLAM3 clone.  
It is organized top‑down from external entry points to core back‑end components.

---

## 1. Top‑Level Entry Points

- **Application / Evaluation Code (outside `src/`)**
  - **calls**: `ORB_SLAM3::System` constructor
  - **then calls** (per sensor type):
    - `System::TrackStereo`
    - `System::TrackRGBD`
    - `System::TrackMonocular`
  - **then calls for shutdown / results**:
    - `System::Shutdown`
    - `System::SaveTrajectoryTUM`
    - `System::SaveKeyFrameTrajectoryTUM`
    - `System::SaveTrajectoryEuRoC`
    - `System::SaveKeyFrameTrajectoryEuRoC`
    - `System::SaveTrajectoryKITTI`
    - `System::SaveAtlas` / `System::LoadAtlas`

- **`System` constructor**
  - **creates / calls**:
    - `Settings` (if new config format) or legacy OpenCV `cv::FileStorage` access
    - `ORBVocabulary::loadFromTextFile`
    - `KeyFrameDatabase` constructor
    - `Atlas` constructor or `System::LoadAtlas`
    - `FrameDrawer` / `MapDrawer` constructors
    - `Tracking` constructor
    - `LocalMapping` constructor
    - `LoopClosing` constructor
    - optional `Viewer` constructor
  - **starts threads**:
    - `std::thread(&LocalMapping::Run, mpLocalMapper)`
    - `std::thread(&LoopClosing::Run, mpLoopCloser)`
    - optional `std::thread(&Viewer::Run, mpViewer)`
  - **wires cross‑module references**:
    - `Tracking::SetLocalMapper`
    - `Tracking::SetLoopClosing`
    - `LocalMapping::SetTracker`
    - `LocalMapping::SetLoopCloser`
    - `LoopClosing::SetTracker`
    - `LoopClosing::SetLocalMapper`
    - `Tracking::SetViewer` (if viewer enabled)

---

## 2. Online Tracking Path

- **`System::TrackStereo` / `TrackRGBD` / `TrackMonocular`**
  - **pre‑processing calls**:
    - `Settings::needToRectify`, `Settings::needToResize`
    - OpenCV image ops: `cv::remap`, `cv::resize`
  - **mode / reset management**:
    - `LocalMapping::RequestStop`, `LocalMapping::Release`
    - `Tracking::InformOnlyTracking`
    - `Tracking::Reset`
    - `Tracking::ResetActiveMap`
  - **IMU handling**:
    - loops over `vector<IMU::Point>`:
      - calls `Tracking::GrabImuData`
  - **main tracking calls**:
    - `Tracking::GrabImageStereo`
    - `Tracking::GrabImageRGBD`
    - `Tracking::GrabImageMonocular`
  - **state exposure**:
    - reads `Tracking::mState`
    - reads `Tracking::mCurrentFrame` (map points and keypoints)

- **`Tracking` main loop (`Tracking::GrabImage*` → internal pipeline)**
  - **depends on / calls**:
    - `Frame` constructor (builds current frame, features, etc.)
    - `ORBextractor` for feature extraction
    - `ORBmatcher` for feature matching
    - `GeometricTools` for epipolar / geometric checks
    - `TwoViewReconstruction` for stereo / initialization
    - `ImuTypes` for IMU preintegration and inertial state
    - `Optimizer` for motion‑only BA / pose refinement
    - `Atlas::GetCurrentMap` and operations on:
      - `Map`, `KeyFrame`, `MapPoint`
    - `KeyFrameDatabase` for place recognition / relocalization
    - `LocalMapping` (via flags and queues) to insert new keyframes and map points
    - `LoopClosing` (implicitly through `KeyFrameDatabase` and map state)
    - `FrameDrawer` to update live frame visualization data
  - **creates / updates**:
    - new `KeyFrame` objects from `Frame`
    - `MapPoint` insertion / culling
    - `Atlas` active `Map`

- **`Frame`**
  - **constructed by**: `Tracking`
  - **uses / calls**:
    - `Converter` utilities for pose / matrix conversions
    - `ORBextractor` for building multiscale ORB pyramids
    - camera calibration and distortion through `Settings` / config
  - **provides to callers**:
    - features (`mvKeys`, `mvKeysUn`, descriptors)
    - projection / back‑projection helpers for `Tracking`, `LocalMapping`, `Optimizer`

---

## 3. Local Mapping Path

- **`LocalMapping::Run` thread**
  - **called by**: `std::thread` created in `System` constructor
  - **main responsibilities / calls**:
    - consumes new keyframes from `Tracking`
    - **calls**:
      - `LocalMapping::ProcessNewKeyFrame`
      - `LocalMapping::CreateNewMapPoints`
      - `LocalMapping::SearchInNeighbors`
      - `Optimizer::LocalBundleAdjustment`
      - `Optimizer::OptimizeSim3` (in some relocalization / merging paths)
    - interacts with:
      - `Atlas` and active `Map`
      - `KeyFrame`, `MapPoint` objects (creation, fusion, culling)
      - `LoopClosing` (signals when maps are ready or GBA is running)
  - **control from `System` and `Tracking`**:
    - `LocalMapping::RequestStop` / `Release`
    - `LocalMapping::RequestFinish`

- **`Map` / `Atlas`**
  - **called by**:
    - `Tracking`, `LocalMapping`, `LoopClosing`, `Viewer`, `System`
  - **Atlas responsibilities**:
    - manages multiple `Map` instances
    - provides:
      - `Atlas::CreateNewMap`
      - `Atlas::GetAllMaps`
      - `Atlas::GetCurrentMap`
      - inertial flags via `Atlas::SetInertialSensor`
    - serialization:
      - `Atlas::PreSave` / `Atlas::PostLoad`
      - used by `System::SaveAtlas` / `System::LoadAtlas`
  - **Map responsibilities**:
    - storage and queries for:
      - `KeyFrame` objects
      - `MapPoint` objects
    - exposes:
      - `GetAllKeyFrames`, `KeyFramesInMap`
      - `GetAllMapPoints`
      - pose graph and spanning tree relations for `Optimizer` and `LoopClosing`

---

## 4. Loop Closing and Place Recognition

- **`LoopClosing::Run` thread**
  - **called by**: `std::thread` from `System` constructor
  - **high‑level call flow**:
    - consumes candidate keyframes from `LocalMapping` / `Tracking`
    - **uses**:
      - `KeyFrameDatabase` for place recognition (BoW queries)
      - `ORBVocabulary` for BoW scoring
      - `ORBmatcher` for loop feature matching
      - `Sim3Solver` for similarity transform estimation
      - `Optimizer` for:
        - pose graph optimization
        - global bundle adjustment
    - **updates**:
      - `Atlas` and `Map` topology
      - `KeyFrame` poses
      - `MapPoint` positions (loop fusion)
  - **cooperates with**:
    - `Tracking` (to pause tracking when GBA runs)
    - `LocalMapping` (map consistency during loop correction)
    - optional `Viewer` (to show loop closures in UI)

- **`KeyFrameDatabase`**
  - **built by**: `System` constructor using `ORBVocabulary`
  - **called by**:
    - `Tracking` for relocalization queries
    - `LoopClosing` for loop detection
  - **internal calls**:
    - BoW scoring via `ORBVocabulary`
    - access to `KeyFrame` BoW vectors and feature data

---

## 5. Visualization Path

- **`Viewer::Run` thread** (optional)
  - **created and started by**: `System` constructor (if `bUseViewer == true`)
  - **calls / depends on**:
    - `FrameDrawer` for current frame overlays
    - `MapDrawer` for map / trajectory visualization
    - Pangolin UI functions (`pangolin::CreateWindowAndBind`, draw loops, etc.)
    - `System` / `Tracking` to query:
      - tracking state
      - active map, keyframes, and map points
  - **called by**:
    - `System::Shutdown` (indirectly via finish requests)

- **`FrameDrawer`**
  - **updated by**: `Tracking` on each frame
  - **used by**: `Viewer`
  - **depends on**:
    - `Atlas` (for current map)
    - `Frame` / `Tracking` state (feature tracks, matches, etc.)

- **`MapDrawer`**
  - **constructed by**: `System` with `Atlas` and settings
  - **used by**: `Viewer`
  - **reads from**:
    - `Atlas`, `Map`, `KeyFrame`, `MapPoint`

---

## 6. Core Data Structures and Utilities

- **`KeyFrame`**
  - **created by**:
    - `Tracking` (when promoting a `Frame` to keyframe)
    - `LocalMapping` (during map initialization / duplication)
  - **used by**:
    - `Map`, `Atlas`
    - `KeyFrameDatabase`
    - `LoopClosing`
    - `Optimizer`
    - `System` trajectory saving functions
  - **calls / depends on**:
    - `MapPoint` (observations)
    - `Converter` for representation conversions
    - `ImuTypes` (for IMU pose / bias when inertial)

- **`MapPoint`**
  - **created / managed by**:
    - `LocalMapping` (triangulation, fusion)
    - `Tracking` (initialization and temporary points)
  - **used by**:
    - `Tracking` (projection / matching)
    - `Optimizer` (BA)
    - `LoopClosing` (fusion in loop areas)
    - `MapDrawer` / `Viewer` (visualization)

- **`ORBextractor`**
  - **called by**:
    - `Frame` / `Tracking` during frame construction
    - sometimes `KeyFrame` building when recomputing features
  - **provides**:
    - ORB keypoints and descriptors at multiple pyramid levels

- **`ORBmatcher`**
  - **called by**:
    - `Tracking` (frame‑to‑frame, frame‑to‑map, relocalization)
    - `LocalMapping` (searches around new keyframes)
    - `LoopClosing` (loop candidate matching)

- **`Optimizer`**
  - **called by**:
    - `Tracking`:
      - motion‑only bundle adjustment
      - pose refinement during initialization and relocalization
    - `LocalMapping`:
      - local bundle adjustment
    - `LoopClosing`:
      - pose graph optimization
      - global bundle adjustment
  - **operates on**:
    - `KeyFrame` graph
    - `MapPoint` set
    - IMU states (`ImuTypes`) when inertial
  - **builds on**:
    - `G2oTypes` (custom g2o vertices / edges)
    - external g2o optimization library

- **`GeometricTools`**
  - **called by**:
    - `Tracking`, `LocalMapping`, `LoopClosing` for:
      - epipolar geometry checks
      - triangulation helpers

- **`TwoViewReconstruction`**
  - **called by**:
    - `Tracking` during stereo / monocular initialization
  - **depends on**:
    - `GeometricTools`, `ORBmatcher`, `Optimizer`

- **`Sim3Solver`**
  - **called by**:
    - `LoopClosing` and map merging / alignment code
  - **operates on**:
    - matched `KeyFrame` feature sets
    - produces Sim(3) transformation hypotheses refined by `Optimizer`

- **`Converter`**
  - **called throughout** the codebase to convert between:
    - OpenCV `cv::Mat`
    - Eigen types
    - Sophus `SE3f`
    - quaternions and rotation matrices

- **`ImuTypes`**
  - **used by**:
    - `Tracking` (preintegrated IMU factors)
    - `LocalMapping` (inertial initialization)
    - `Optimizer` (IMU constraints in BA)

- **`Config` / `Settings`**
  - **called by**:
    - `System` (initialization)
    - `Tracking`, `Frame`, `MapDrawer`, `Viewer` (camera / visualization params)

---

## 7. Serialization and Atlas I/O

- **`System::SaveAtlas`**
  - **calls**:
    - `Atlas::PreSave`
    - Boost serialization:
      - `boost::archive::text_oarchive` or `boost::archive::binary_oarchive`
    - `System::CalculateCheckSum` to hash vocabulary file
  - **outputs**:
    - `*.osa` serialized atlas file with vocabulary checksum

- **`System::LoadAtlas`**
  - **calls**:
    - Boost deserialization (`text_iarchive` / `binary_iarchive`)
    - `System::CalculateCheckSum` to verify vocabulary consistency
    - `Atlas::SetKeyFrameDababase`
    - `Atlas::SetORBVocabulary`
    - `Atlas::PostLoad`

- **`System::CalculateCheckSum`**
  - **depends on**:
    - OpenSSL MD5 (`MD5_Init`, `MD5_Update`, `MD5_Final`)
    - standard streams for file access

---

## 8. Debugging and Diagnostics

- **Time and state queries (called by external tools / evaluation code)**:
  - `System::GetTrackingState`
  - `System::GetTrackedMapPoints`
  - `System::GetTrackedKeyPointsUn`
  - `System::GetTimeFromIMUInit`
  - `System::isLost`
  - `System::isFinished`
  - `System::GetImageScale`

- **Initialization debugging (`System::SaveDebugData`)**
  - **calls**:
    - `SaveTrajectoryEuRoC` (frame trajectory for initialization)
    - writes diagnostic files using:
      - `LocalMapping::mScale`
      - `LocalMapping::mRwg` (gravity direction)
      - `LocalMapping::mCostTime`
      - `LocalMapping::mbg`, `LocalMapping::mba` (IMU bias estimates)
      - `LocalMapping::mcovInertial` (covariance)
      - `LocalMapping::mInitTime`

---

## 9. Summary of Major Call Chains

- **Main tracking loop (stereo / RGB‑D / monocular)**  
  - Application → `System::Track*`  
  - `System::Track*` → `Tracking::GrabImage*`  
  - `Tracking::GrabImage*` → `Frame` / `ORBextractor` / `ORBmatcher` / `GeometricTools` / `Optimizer`  
  - `Tracking` ↔ `LocalMapping` (keyframe and map point updates)  
  - `Tracking` ↔ `KeyFrameDatabase` / `LoopClosing` (relocalization and loops)  
  - `Tracking` → `FrameDrawer` / `MapDrawer` → `Viewer` (visualization)

- **Mapping and loop closing**  
  - `Tracking` → `LocalMapping::Run` (via keyframe queue)  
  - `LocalMapping` → `Optimizer::LocalBundleAdjustment` → g2o / `G2oTypes`  
  - `LocalMapping` / `Tracking` → `KeyFrameDatabase` → `LoopClosing::Run`  
  - `LoopClosing` → `Sim3Solver` / `ORBmatcher` / `Optimizer` → updates `Atlas` / `Map` / `KeyFrame` / `MapPoint`

- **Serialization and dataset management**  
  - Application → `System::SaveAtlas` / `System::LoadAtlas`  
  - `System::*Trajectory*` → `Atlas` / `Map` / `KeyFrame` / `Tracking` trajectory lists  
  - `System::ChangeDataset` → `Tracking::ResetActiveMap` or `Tracking::CreateMapInAtlas` → new `Map` in `Atlas`

This overview is intended as a **navigational map** for the repository: for any function or class you encounter, you can locate its place in one of the layers above and reason about who calls it and what downstream modules it influences.

