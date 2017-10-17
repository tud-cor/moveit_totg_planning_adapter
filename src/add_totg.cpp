#include <class_loader/class_loader.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <trajectories/Path.h>
#include <trajectories/Trajectory.h>

using namespace Eigen;
using namespace std;

namespace totg_planning_adapter {

class AddTOTG : public planning_request_adapter::PlanningRequestAdapter {
public:
  AddTOTG() : planning_request_adapter::PlanningRequestAdapter(), nh_("~") {}

  ~AddTOTG(){};

  virtual std::string getDescription() const {
    return "TOTG Planning Request Adapter";
  }

  virtual bool
  adaptAndPlan(const PlannerFn &planner,
               const planning_scene::PlanningSceneConstPtr &planning_scene,
               const planning_interface::MotionPlanRequest &req,
               planning_interface::MotionPlanResponse &res,
               std::vector<std::size_t> &added_path_index) const {
    bool result = planner(planning_scene, req, res);
    bool totg_result = false;

    if (result && res.trajectory_) {

      list<VectorXd> waypoints;
      double acceleration_scaling_factor = 1.0;
      double velocity_scaling_factor = 1.0;

      robot_trajectory::RobotTrajectoryPtr rt = res.trajectory_;

      int vars_count = rt->getGroup()->getVariableCount();

      const robot_model::JointModelGroup *jmg = rt->getGroup();
      const robot_model::RobotModel &rmodel = jmg->getParentModel();
      const std::vector<std::string> &vars = jmg->getVariableNames();

      VectorXd waypoint(vars_count);
      VectorXd maxAcc(vars_count);
      VectorXd maxVel(vars_count);

      for (size_t i = 0; i < rt->getWayPointCount(); i++) {
        rt->getWayPoint(i).copyJointGroupPositions(jmg, waypoint);
        waypoints.push_back(waypoint);
      }

      for (int i = 0; i < vars_count; i++) {
        // Get Acceleration limit
        double a_max = 1.0;
        const robot_model::VariableBounds &a =
            rmodel.getVariableBounds(vars[i]);
        if (a.acceleration_bounded_)
          a_max =
              std::min(fabs(a.max_acceleration_ * acceleration_scaling_factor),
                       fabs(a.min_acceleration_ * acceleration_scaling_factor));

        // Get Velocity limit
        double v_max = 1.0;
        const robot_model::VariableBounds &b =
            rmodel.getVariableBounds(vars[i]);
        if (b.velocity_bounded_)
          v_max = std::min(fabs(b.max_velocity_ * velocity_scaling_factor),
                           fabs(b.min_velocity_ * velocity_scaling_factor));
        maxAcc[i] = a_max;
        maxVel[i] = v_max;
      }

      double timestep = 0.001; // Decreasing this should provide better paths,
                               // but the parameterization takes longer

      Trajectory trajectory(Path(waypoints, 0.1), maxVel, maxAcc, timestep);

      totg_result = trajectory.isValid();

      robot_state::RobotState rs(rt->getRobotModel());

      if (totg_result) {
        double step_size = 0.025;
        double duration = trajectory.getDuration();
        rt->clear();

        Eigen::VectorXd vel_vec = trajectory.getVelocity(0.0);
        std::vector<double> velocities(
            vel_vec.data(), vel_vec.data() + vel_vec.rows() * vel_vec.cols());

        rs.setJointGroupPositions(jmg, trajectory.getPosition(0.0));
        rs.setVariableVelocities(vars, velocities);
        rt->addSuffixWayPoint(rs, 0.0);

        for (double t = 0.0; t < duration; t += step_size) {
          Eigen::VectorXd vel_vec = trajectory.getVelocity(t);
          std::vector<double> velocities(
              vel_vec.data(), vel_vec.data() + vel_vec.rows() * vel_vec.cols());

          rs.setJointGroupPositions(jmg, trajectory.getPosition(t));
          rs.setVariableVelocities(vars, velocities);
          rt->addSuffixWayPoint(rs, step_size);
        }
      } else {
        ROS_WARN_STREAM("TOTG failed!");
      }
    }
    return result && totg_result;
  }

private:
  ros::NodeHandle nh_;
};
}

CLASS_LOADER_REGISTER_CLASS(totg_planning_adapter::AddTOTG,
                            planning_request_adapter::PlanningRequestAdapter);
