#pragma once
#include "LocalTrackingAttempt.h"
namespace ORB_SLAM3 {
struct LocalGeometryDiagnostics {
  int constraints=0, rank=0;
  double residual_rms=0, residual_p95=0, information_min=0, information_max=0;
};
// Optional frozen diagnostic only. Costs must be included in callback timing.
LocalGeometryDiagnostics DiagnoseLocalGeometry(const LocalTrackingAttempt& attempt);
}
