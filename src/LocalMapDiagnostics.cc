#include "LocalMapDiagnostics.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
namespace ORB_SLAM3 {
LocalGeometryDiagnostics DiagnoseLocalGeometry(const LocalTrackingAttempt& attempt) {
  LocalGeometryDiagnostics result;
  Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Zero();
  std::vector<double> residuals;
  double sum=0;
  const auto& frame=attempt.frame;
  for(int i=0;i<frame.N;++i) {
    const auto& point=frame.mvpMapPoints[i];
    if(!point || frame.mvbOutlier[i]) continue;
    const auto& snapshot=attempt.ExistingPoint(point.get()).snapshot;
    const Eigen::Vector3d pc=(frame.GetPose()*snapshot.geometry.position).cast<double>();
    if(!pc.allFinite() || pc.z()<=0) continue;
    const auto uv=frame.mpCamera->project(pc);
    const auto& key=frame.mvKeysUn[i];
    const double weight=frame.mvInvLevelSigma2.at(key.octave);
    double squared=(uv-Eigen::Vector2d(key.pt.x,key.pt.y)).squaredNorm();
    Eigen::Matrix<double,3,6> transform;
    transform<<0,pc.z(),-pc.y(),1,0,0,-pc.z(),0,pc.x(),0,1,0,pc.y(),-pc.x(),0,0,0,1;
    const Eigen::Matrix<double,2,3> projection=frame.mpCamera->projectJac(pc);
    const Eigen::Matrix<double,2,6> jacobian=projection*transform;
    information.noalias()+=weight*jacobian.transpose()*jacobian;
    if(frame.mvuRight[i]>=0) {
      const double error=uv.x()-frame.mbf/pc.z()-frame.mvuRight[i];
      squared+=error*error;
      Eigen::Matrix<double,1,3> right=projection.row(0);
      right(2)+=frame.mbf/(pc.z()*pc.z());
      const Eigen::Matrix<double,1,6> jr=right*transform;
      information.noalias()+=weight*jr.transpose()*jr;
    }
    sum+=squared;residuals.push_back(std::sqrt(squared));
  }
  result.constraints=residuals.size();
  if(residuals.empty()) return result;
  result.residual_rms=std::sqrt(sum/residuals.size());
  const size_t index=size_t(std::ceil(.95*residuals.size()))-1;
  std::nth_element(residuals.begin(),residuals.begin()+index,residuals.end());
  result.residual_p95=residuals[index];
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eigen(information,Eigen::EigenvaluesOnly);
  if(eigen.info()==Eigen::Success) {
    result.information_min=eigen.eigenvalues()(0);result.information_max=eigen.eigenvalues()(5);
    const double threshold=std::max(1e-9,result.information_max*1e-8);
    for(int i=0;i<6;++i) if(eigen.eigenvalues()(i)>threshold) ++result.rank;
  }
  return result;
}
}
