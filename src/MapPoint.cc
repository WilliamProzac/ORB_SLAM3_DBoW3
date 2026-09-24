/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include "MapPoint.h"
#include <stdexcept>
#include "FeatureDescriptor.h"
#include "ORBmatcher.h"

#include <mutex>

namespace ORB_SLAM3 {
namespace { std::mutex replacementMutex; }

long unsigned int MapPoint::nNextId = 0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint()
    : mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0),
      mnLoopPointForKF(0), mnCorrectedByKF(0), mnCorrectedReference(0),
      mnBAGlobalForKF(0), mnVisible(1), mnFound(1), mbBad(false),
      mpReplaced(MapPointPtr()) {
  mpReplaced = MapPointPtr();
}

MapPoint::MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map *pMap)
    : mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0),
      mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mnBALocalForKF(0),
      mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
      mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF),
      mnVisible(1), mnFound(1), mbBad(false),
      mpReplaced(MapPointPtr()), mfMinDistance(0),
      mfMaxDistance(0), mpMap(pMap), mnOriginMapId(pMap->GetId()) {
  SetWorldPos(Pos);

  mNormalVector.setZero();

  mbTrackInViewR = false;
  mbTrackInView = false;

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame *pRefKF,
                   KeyFrame *pHostKF, Map *pMap)
    : mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0),
      mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mnBALocalForKF(0),
      mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
      mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF),
      mnVisible(1), mnFound(1), mbBad(false),
      mpReplaced(MapPointPtr()), mfMinDistance(0),
      mfMaxDistance(0), mpMap(pMap), mnOriginMapId(pMap->GetId()) {
  mInvDepth = invDepth;
  mInitU = (double)uv_init.x;
  mInitV = (double)uv_init.y;
  mpHostKF = pHostKF;

  mNormalVector.setZero();

  // Worldpos is not set
  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map *pMap, Frame *pFrame,
                   const int &idxF)
    : mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0),
      mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mnBALocalForKF(0),
      mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
      mnCorrectedReference(0), mnBAGlobalForKF(0),
      mpRefKF(static_cast<KeyFrame *>(NULL)), mnVisible(1), mnFound(1),
      mbBad(false), mpReplaced(NULL), mpMap(pMap),
      mnOriginMapId(pMap->GetId()) {
  SetWorldPos(Pos);

  Eigen::Vector3f Ow;
  if (pFrame->Nleft == -1 || idxF < pFrame->Nleft) {
    Ow = pFrame->GetCameraCenter();
  } else {
    Eigen::Matrix3f Rwl = pFrame->GetRwc();
    Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
    Eigen::Vector3f twl = pFrame->GetOw();

    Ow = Rwl * tlr + twl;
  }
  mNormalVector = mWorldPos - Ow;
  mNormalVector = mNormalVector / mNormalVector.norm();

  Eigen::Vector3f PC = mWorldPos - Ow;
  const float dist = PC.norm();
  const int level = (pFrame->Nleft == -1)    ? pFrame->mvKeysUn[idxF].octave
                    : (idxF < pFrame->Nleft) ? pFrame->mvKeys[idxF].octave
                                             : pFrame->mvKeysRight[idxF].octave;
  const float levelScaleFactor = pFrame->mvScaleFactors[level];
  const int nLevels = pFrame->mnScaleLevels;

  mfMaxDistance = dist * levelScaleFactor;
  mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

  pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  unique_lock<mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

void MapPoint::SetWorldPos(const Eigen::Vector3f &Pos) {
  unique_lock<mutex> lock2(mGlobalMutex);
  unique_lock<mutex> lock(mMutexPos);
  mWorldPos = Pos;
}

Eigen::Vector3f MapPoint::GetWorldPos() {
  unique_lock<mutex> lock(mMutexPos);
  return mWorldPos;
}

Eigen::Vector3f MapPoint::GetNormal() {
  unique_lock<mutex> lock(mMutexPos);
  return mNormalVector;
}

KeyFrame *MapPoint::GetReferenceKeyFrame() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame *pKF, int idx) {
  unique_lock<mutex> lock(mMutexFeatures);
  tuple<int, int> indexes;

  if (mObservations.count(pKF)) {
    indexes = mObservations[pKF];
  } else {
    indexes = tuple<int, int>(-1, -1);
  }

  if (pKF->NLeft != -1 && idx >= pKF->NLeft) {
    get<1>(indexes) = idx;
  } else {
    get<0>(indexes) = idx;
  }

  mObservations[pKF] = indexes;

  if (!pKF->mpCamera2 && pKF->mvuRight[idx] >= 0)
    nObs += 2;
  else
    nObs++;
}

void MapPoint::EraseObservation(KeyFrame *pKF) {
  const auto keepAlive = shared_from_this();
  bool bBad = false;
  {
    unique_lock<mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF)) {
      tuple<int, int> indexes = mObservations[pKF];
      int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

      if (leftIndex != -1) {
        if (!pKF->mpCamera2 && pKF->mvuRight[leftIndex] >= 0)
          nObs -= 2;
        else
          nObs--;
      }
      if (rightIndex != -1) {
        nObs--;
      }

      mObservations.erase(pKF);

      if (mpRefKF == pKF)
        mpRefKF = mObservations.empty() ? nullptr : mObservations.begin()->first;

      // If only 2 observations or less, discard point
      if (nObs <= 2)
        bBad = true;
    }
  }

  if (bBad)
    SetBadFlag();
}

std::map<KeyFrame *, std::tuple<int, int>> MapPoint::GetObservations() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mObservations;
}

int MapPoint::Observations() {
  unique_lock<mutex> lock(mMutexFeatures);
  return nObs;
}

void MapPoint::SetBadFlag() {
  const auto keepAlive = shared_from_this();
  map<KeyFrame *, tuple<int, int>> obs;
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    if (mbBad) return;
    mMemoryLifetime.Retire();
    mbBad = true;
    obs = mObservations;
    mObservations.clear();
  }
  for (map<KeyFrame *, tuple<int, int>>::iterator mit = obs.begin(),
                                                  mend = obs.end();
       mit != mend; mit++) {
    KeyFrame *pKF = mit->first;
    int leftIndex = get<0>(mit->second), rightIndex = get<1>(mit->second);
    if (leftIndex != -1) {
      pKF->EraseMapPointMatch(leftIndex);
    }
    if (rightIndex != -1) {
      pKF->EraseMapPointMatch(rightIndex);
    }
  }

  mpMap->EraseMapPoint(keepAlive);
}

MapPointPtr MapPoint::GetReplaced() {
  unique_lock<mutex> lock1(mMutexFeatures);
  unique_lock<mutex> lock2(mMutexPos);
  return mpReplaced;
}

void MapPoint::Replace(MapPointPtr pMP) {
  const auto keepAlive = shared_from_this();
  // Serialize replacement edges so opposite concurrent replacements cannot
  // create a strong cycle. A retired source never changes its published edge.
  std::lock_guard<std::mutex> transition(replacementMutex);
  if (!pMP) return;
  std::set<MapPoint*> seen;
  while (pMP) {
    if (pMP.get() == this || pMP->mnId == mnId || !seen.insert(pMP.get()).second) return;
    auto next = pMP->GetReplaced();
    if (!next) break;
    pMP = next;
  }
  if (pMP->isBad()) return;

  int nvisible, nfound;
  map<KeyFrame *, tuple<int, int>> obs;
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    if (mbBad) return;
    obs = mObservations;
    mObservations.clear();
    mMemoryLifetime.Retire();
    mbBad = true;
    nvisible = mnVisible;
    nfound = mnFound;
    mpReplaced = pMP;
  }

  for (map<KeyFrame *, tuple<int, int>>::iterator mit = obs.begin(),
                                                  mend = obs.end();
       mit != mend; mit++) {
    // Replace measurement in keyframe
    KeyFrame *pKF = mit->first;

    tuple<int, int> indexes = mit->second;
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

    if (!pMP->IsInKeyFrame(pKF)) {
      if (leftIndex != -1) {
        pKF->ReplaceMapPointMatch(leftIndex, pMP);
        pMP->AddObservation(pKF, leftIndex);
      }
      if (rightIndex != -1) {
        pKF->ReplaceMapPointMatch(rightIndex, pMP);
        pMP->AddObservation(pKF, rightIndex);
      }
    } else {
      if (leftIndex != -1) {
        pKF->EraseMapPointMatch(leftIndex);
      }
      if (rightIndex != -1) {
        pKF->EraseMapPointMatch(rightIndex);
      }
    }
  }
  pMP->IncreaseFound(nfound);
  pMP->IncreaseVisible(nvisible);
  pMP->ComputeDistinctiveDescriptors();

  mpMap->EraseMapPoint(keepAlive);
}

bool MapPoint::isBad() {
  // Both published writes (SetBadFlag/Replace) hold mMutexFeatures. Position
  // reads/writes do not modify mbBad, so taking mMutexPos adds no protection.
  unique_lock<mutex> lock(mMutexFeatures);
  return mbBad;
}

void MapPoint::IncreaseVisible(int n) {
  unique_lock<mutex> lock(mMutexFeatures);
  mnVisible += n;
}

void MapPoint::IncreaseFound(int n) {
  unique_lock<mutex> lock(mMutexFeatures);
  mnFound += n;
}

float MapPoint::GetFoundRatio() {
  unique_lock<mutex> lock(mMutexFeatures);
  return static_cast<float>(mnFound) / mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors() {
  // Retrieve all observed descriptors
  vector<cv::Mat> vDescriptors;

  map<KeyFrame *, tuple<int, int>> observations;

  {
    unique_lock<mutex> lock1(mMutexFeatures);
    if (mbBad)
      return;
    observations = mObservations;
  }

  if (observations.empty())
    return;

  vDescriptors.reserve(observations.size());

  for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(),
                                                  mend = observations.end();
       mit != mend; mit++) {
    KeyFrame *pKF = mit->first;

    if (!pKF->isBad()) {
      tuple<int, int> indexes = mit->second;
      int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

      if (leftIndex != -1) {
        vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
      }
      if (rightIndex != -1) {
        vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
      }
    }
  }

  if (vDescriptors.empty())
    return;

  const cv::Mat representative = FeatureDescriptorMedoid(vDescriptors);

  {
    unique_lock<mutex> lock(mMutexFeatures);
    mDescriptor = representative;
  }
}

cv::Mat MapPoint::GetDescriptor() {
  unique_lock<mutex> lock(mMutexFeatures);
  return mDescriptor.clone();
}

bool MapPoint::CopyOrbDescriptor(unsigned char (&output)[32]) {
  unique_lock<mutex> lock(mMutexFeatures);
  if (mDescriptor.type() != CV_8UC1 || mDescriptor.rows != 1 ||
      mDescriptor.cols != 32 || mDescriptor.empty())
    return false;
  std::copy_n(mDescriptor.ptr<unsigned char>(), 32, output);
  return true;
}

tuple<int, int> MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexFeatures);
  if (mObservations.count(pKF))
    return mObservations[pKF];
  else
    return tuple<int, int>(-1, -1);
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF) {
  unique_lock<mutex> lock(mMutexFeatures);
  return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth() {
  map<KeyFrame *, tuple<int, int>> observations;
  KeyFrame *pRefKF;
  Eigen::Vector3f Pos;
  {
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    if (mbBad)
      return;
    observations = mObservations;
    pRefKF = mpRefKF;
    Pos = mWorldPos;
  }

  if (observations.empty())
    return;

  Eigen::Vector3f normal;
  normal.setZero();
  int n = 0;
  for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(),
                                                  mend = observations.end();
       mit != mend; mit++) {
    KeyFrame *pKF = mit->first;

    tuple<int, int> indexes = mit->second;
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

    if (leftIndex != -1) {
      Eigen::Vector3f Owi = pKF->GetCameraCenter();
      Eigen::Vector3f normali = Pos - Owi;
      normal = normal + normali / normali.norm();
      n++;
    }
    if (rightIndex != -1) {
      Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
      Eigen::Vector3f normali = Pos - Owi;
      normal = normal + normali / normali.norm();
      n++;
    }
  }

  Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
  const float dist = PC.norm();

  tuple<int, int> indexes = observations[pRefKF];
  int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
  int level;
  if (pRefKF->NLeft == -1) {
    level = pRefKF->mvKeysUn[leftIndex].octave;
  } else if (leftIndex != -1) {
    level = pRefKF->mvKeys[leftIndex].octave;
  } else {
    level = pRefKF->mvKeysRight[rightIndex - pRefKF->NLeft].octave;
  }

  // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
  const float levelScaleFactor = pRefKF->mvScaleFactors[level];
  const int nLevels = pRefKF->mnScaleLevels;

  {
    unique_lock<mutex> lock3(mMutexPos);
    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
    mNormalVector = normal / n;
  }
}

void MapPoint::SetNormalVector(const Eigen::Vector3f &normal) {
  unique_lock<mutex> lock3(mMutexPos);
  mNormalVector = normal;
}

float MapPoint::GetMinDistanceInvariance() {
  unique_lock<mutex> lock(mMutexPos);
  return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance() {
  unique_lock<mutex> lock(mMutexPos);
  return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame *pKF) {
  float ratio;
  {
    unique_lock<mutex> lock(mMutexPos);
    ratio = mfMaxDistance / currentDist;
  }

  int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
  if (nScale < 0)
    nScale = 0;
  else if (nScale >= pKF->mnScaleLevels)
    nScale = pKF->mnScaleLevels - 1;

  return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame *pF) {
  float ratio;
  {
    unique_lock<mutex> lock(mMutexPos);
    ratio = mfMaxDistance / currentDist;
  }

  int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
  if (nScale < 0)
    nScale = 0;
  else if (nScale >= pF->mnScaleLevels)
    nScale = pF->mnScaleLevels - 1;

  return nScale;
}

void MapPoint::PrintObservations() {
  cout << "MP_OBS: MP " << mnId << endl;
  for (map<KeyFrame *, tuple<int, int>>::iterator mit = mObservations.begin(),
                                                  mend = mObservations.end();
       mit != mend; mit++) {
    KeyFrame *pKFi = mit->first;
    tuple<int, int> indexes = mit->second;
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    cout << "--OBS in KF " << pKFi->mnId << " in map "
         << pKFi->GetMap()->GetId() << endl;
  }
}

Map *MapPoint::GetMap() {
  unique_lock<mutex> lock(mMutexMap);
  return mpMap;
}

void MapPoint::UpdateMap(Map *pMap) {
  unique_lock<mutex> lock(mMutexMap);
  mpMap = pMap;
}

void MapPoint::PreSave(set<KeyFrame *> &spKF, set<MapPointPtr > &spMP) {
  mBackupReplacedId = -1;
  const auto replaced = GetReplaced();
  if (replaced && spMP.find(replaced) != spMP.end())
    mBackupReplacedId = replaced->mnId;

  mBackupObservationsId1.clear();
  mBackupObservationsId2.clear();
  // Save the id and position in each KF who view it
  const auto observations = GetObservations();
  vector<KeyFrame *> vToErase;
  for (std::map<KeyFrame *, std::tuple<int, int>>::const_iterator
           it = observations.begin(),
           end = observations.end();
       it != end; ++it) {
    KeyFrame *pKFi = it->first;
    if (spKF.find(pKFi) != spKF.end()) {
      mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
      mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
    } else {
      vToErase.push_back(pKFi);
    }
  }

  for (KeyFrame *pKFi : vToErase) {
    EraseObservation(pKFi);
  }

  // Save the id of the reference KF
  const auto refKF = GetReferenceKeyFrame();
  mBackupRefKFId = -1;
  if (refKF && spKF.find(refKF) != spKF.end()) {
    mBackupRefKFId = refKF->mnId;
  }
}

void MapPoint::PostLoad(map<long unsigned int, KeyFrame *> &mpKFid,
                        map<long unsigned int, MapPointPtr > &mpMPid) {
  mpRefKF = mpKFid[mBackupRefKFId];
  if (!mpRefKF) {
    cout << "ERROR: MP without KF reference " << mBackupRefKFId
         << "; Num obs: " << nObs << endl;
  }
  mpReplaced = MapPointPtr();
  if (mBackupReplacedId >= 0) {
    map<long unsigned int, MapPointPtr >::iterator it =
        mpMPid.find(mBackupReplacedId);
    if (it != mpMPid.end()) {
      // Load is unpublished and sequential. Reject the edge that would close
      // a cycle before installing a strong reference.
      std::set<MapPoint*> seen;
      for (auto target = it->second; target; target = target->GetReplaced())
        if (target.get() == this || !seen.insert(target.get()).second)
          throw std::runtime_error("cyclic MapPoint replacement in archive");
      mpReplaced = it->second;
    }
  }

  mObservations.clear();

  for (map<long unsigned int, int>::const_iterator
           it = mBackupObservationsId1.begin(),
           end = mBackupObservationsId1.end();
       it != end; ++it) {
    KeyFrame *pKFi = mpKFid[it->first];
    map<long unsigned int, int>::const_iterator it2 =
        mBackupObservationsId2.find(it->first);
    std::tuple<int, int> indexes = tuple<int, int>(it->second, it2->second);
    if (pKFi) {
      mObservations[pKFi] = indexes;
    }
  }

  mBackupObservationsId1.clear();
  mBackupObservationsId2.clear();
}

} // namespace ORB_SLAM3
