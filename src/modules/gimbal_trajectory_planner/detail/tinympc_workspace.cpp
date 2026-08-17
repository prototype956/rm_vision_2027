#include "modules/gimbal_trajectory_planner/detail/tinympc_workspace.hpp"

#include "tinympc/admm.hpp"
#include "tinympc/tiny_api.hpp"

#include <stdexcept>

namespace mv::modules::detail {
namespace {

void ValidateSolver(const TinySolver& solver) {
  if (solver.solution == nullptr || solver.work == nullptr) {
    throw std::runtime_error("TinyMPC solver workspace is not initialized");
  }
}

void ShiftHorizon(tinyMatrix& value) {
  if (value.cols() <= 1)
    return;
  value.leftCols(value.cols() - 1) = value.rightCols(value.cols() - 1).eval();
}

void ClearSolveStatus(TinySolver& solver) noexcept {
  TinyWorkspace& work = *solver.work;
  work.primal_residual_state = 0;
  work.primal_residual_input = 0;
  work.dual_residual_state = 0;
  work.dual_residual_input = 0;
  work.status = 11;
  work.iter = 0;
  solver.solution->iter = 0;
  solver.solution->solved = 0;
}

}  // namespace

void ShiftTinyMpcWarmStart(TinySolver& solver) {
  ValidateSolver(solver);
  TinyWorkspace& work = *solver.work;
  ShiftHorizon(work.x);
  ShiftHorizon(work.u);
  ShiftHorizon(work.v);
  ShiftHorizon(work.vnew);
  ShiftHorizon(work.z);
  ShiftHorizon(work.znew);
  ShiftHorizon(work.g);
  ShiftHorizon(work.y);
  ShiftHorizon(work.vc);
  ShiftHorizon(work.vcnew);
  ShiftHorizon(work.zc);
  ShiftHorizon(work.zcnew);
  ShiftHorizon(work.gc);
  ShiftHorizon(work.yc);
  ShiftHorizon(work.vl);
  ShiftHorizon(work.vlnew);
  ShiftHorizon(work.zl);
  ShiftHorizon(work.zlnew);
  ShiftHorizon(work.gl);
  ShiftHorizon(work.yl);
  ShiftHorizon(solver.solution->x);
  ShiftHorizon(solver.solution->u);
  ClearSolveStatus(solver);
}

void RebaseTinyMpcWarmStart(TinySolver& solver, const tinyVector& x0) {
  ValidateSolver(solver);
  TinyWorkspace& work = *solver.work;
  if (x0.rows() != work.nx || work.Xref.rows() != work.nx || work.Xref.cols() != work.N ||
      work.Uref.rows() != work.nu || work.Uref.cols() != work.N - 1) {
    throw std::runtime_error("TinyMPC warm-start dimensions do not match the solver workspace");
  }

  work.x = work.Xref;
  work.x.col(0) = x0;
  if (work.x_min.rows() == work.nx && work.x_min.cols() == work.N && work.x_max.rows() == work.nx &&
      work.x_max.cols() == work.N) {
    work.x = work.x_max.cwiseMin(work.x_min.cwiseMax(work.x));
    work.x.col(0) = x0;
  }
  work.u = work.Uref;
  if (work.u_min.rows() == work.nu && work.u_min.cols() == work.N - 1 &&
      work.u_max.rows() == work.nu && work.u_max.cols() == work.N - 1) {
    work.u = work.u_max.cwiseMin(work.u_min.cwiseMax(work.u));
  }

  work.q.setZero();
  work.r.setZero();
  work.p.setZero();
  work.d.setZero();
  work.v = work.x;
  work.vnew = work.x;
  work.z = work.u;
  work.znew = work.u;
  work.g.setZero();
  work.y.setZero();
  work.vc = work.x;
  work.vcnew = work.x;
  work.zc = work.u;
  work.zcnew = work.u;
  work.gc.setZero();
  work.yc.setZero();
  work.vl = work.x;
  work.vlnew = work.x;
  work.zl = work.u;
  work.zlnew = work.u;
  work.gl.setZero();
  work.yl.setZero();
  ClearSolveStatus(solver);
  solver.solution->x = work.x;
  solver.solution->u = work.u;
}

int SolveTinyMpcWithFreshReference(TinySolver& solver) {
  ValidateSolver(solver);
  update_linear_cost(&solver);
  return tiny_solve(&solver);
}

}  // namespace mv::modules::detail
