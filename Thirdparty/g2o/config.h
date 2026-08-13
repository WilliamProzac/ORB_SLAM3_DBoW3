#ifndef G2O_CONFIG_H
#define G2O_CONFIG_H

/* #undef G2O_OPENMP */
/* #undef G2O_SHARED_LIBS */

// Give a warning if Eigen defaults to row-major matrices. g2o internally
// assumes column-major matrices throughout the code.
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#error "g2o requires column major Eigen matrices"
#endif

#endif
