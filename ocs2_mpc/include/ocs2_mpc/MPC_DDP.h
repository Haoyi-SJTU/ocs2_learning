

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_ddp/GaussNewtonDDP.h>

#include "ocs2_mpc/MPC_BASE.h"

namespace ocs2 {

/**
 * This is an MPC implementation with DDP (SLQ or ILQR) optimal control solvers.
 */
class MPC_DDP : public MPC_BASE {
 public:
  /**
   * Constructor
   *
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   * @param [in] ddpSettings: Structure containing the settings for the DDP algorithm.
   * @param [in] rollout: The rollout class used for simulating the system dynamics.
   * @param [in] optimalControlProblem: The optimal control problem definition.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  MPC_DDP(mpc::Settings mpcSettings, ddp::Settings ddpSettings, const RolloutBase& rollout,
          const OptimalControlProblem& optimalControlProblem, const Initializer& initializer);

  /** Default destructor. */
  ~MPC_DDP() override = default;

  GaussNewtonDDP* getSolverPtr() override { return ddpPtr_.get(); }

  const GaussNewtonDDP* getSolverPtr() const override { return ddpPtr_.get(); }

 protected:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

 private:
  std::unique_ptr<GaussNewtonDDP> ddpPtr_;
};

}  // namespace ocs2
