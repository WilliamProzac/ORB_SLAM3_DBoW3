#pragma once
#include <atomic>
#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"

namespace ORB_SLAM3 {
// Only the optimizer thread writes localStop_. External cancellation uses the
// atomic token. Existing g2o iteration hooks avoid changing its bool* ABI.
class OptimizerStopToken : public g2o::HyperGraphAction {
public:
  OptimizerStopToken(g2o::SparseOptimizer& optimizer,
                     const std::atomic<bool>* stop)
      : optimizer_(optimizer), stop_(stop) {
    if (stop_) {
      Requested();
      optimizer_.setForceStopFlag(&localStop_);
      optimizer_.addPreIterationAction(this);
      optimizer_.addPostIterationAction(this);
    }
  }
  ~OptimizerStopToken() {
    if (stop_) {
      optimizer_.removePreIterationAction(this);
      optimizer_.removePostIterationAction(this);
      optimizer_.setForceStopFlag(nullptr);
    }
  }
  bool Requested() {
    if (stop_ && stop_->load()) localStop_ = true;
    return localStop_;
  }
  HyperGraphAction* operator()(const g2o::HyperGraph*, Parameters*) override {
    Requested();
    return this;
  }
private:
  g2o::SparseOptimizer& optimizer_;
  const std::atomic<bool>* stop_;
  bool localStop_ = false;
};
}
