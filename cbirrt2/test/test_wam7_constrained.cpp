/* File: test_wam7_con.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>
#include <openrave-core.h>
#include "../stdafx.h"

#include <gtest/gtest.h>

#define TSR_DISTANCE_TOLERANCE (1e-3)

namespace {
// from Johannes Schaub - litb
// http://bloglitb.blogspot.de/2011/12/access-to-private-members-safer.html
template<typename Tag, typename Tag::type M> struct Rob { friend typename Tag::type get(Tag) { return M; } };
#define ROBBER(c,t,m) \
   struct c ## _ ## m { typedef t c::*type; friend t c::*get(c ## _ ## m); }; \
   template struct Rob<c ## _ ## m, &c::m>; \
   t & rob_ ## m(c & i) { return i.*get(c ## _ ## m()); } \
   const t & rob_ ## m(const c & i) { return i.*get(c ## _ ## m()); }
ROBBER(TaskSpaceRegionChain, bool, bSampleGoalFromChain)
ROBBER(TaskSpaceRegionChain, bool, bSampleStartFromChain)
ROBBER(TaskSpaceRegionChain, bool, bConstrainToChain)
ROBBER(TaskSpaceRegionChain, std::string, mimicbodyname)
} // anonymous namespace

TEST(Wam7ConstrainedTestCase, SimpleTest)
{
   // initialize with wam7
   OpenRAVE::RaveInitialize(true, OpenRAVE::Level_Info);
   OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
   OpenRAVE::RobotBasePtr robot = env->ReadRobotXMLFile(OpenRAVE::RobotBasePtr(), "robots/barrettwam.robot.xml");
   ASSERT_TRUE(robot);
   env->Add(robot);
   std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manips = robot->GetManipulators();
   robot->SetActiveDOFs(manips[0]->GetArmIndices());
   ASSERT_EQ(robot->GetActiveDOF(), 7);
   // construct planner parameters
   boost::shared_ptr<CBirrtParameters> params(new CBirrtParameters());
   params->SetRobotActiveJoints(robot);
   params->SetConfigurationSpecification(env, robot->GetActiveConfigurationSpecification());
   // move from one side to the other
   params->vinitialconfig.clear();
   params->vinitialconfig.resize(7, 0.0);
   params->vinitialconfig[0] = -0.3;
   params->vinitialconfig[1] = M_PI / 3.0;
   params->vinitialconfig[3] = M_PI / 3.0;
   params->vinitialconfig[5] = M_PI / 3.0;
   params->vinitialconfig[6] = -0.3;
   params->vgoalconfig.clear();
   params->vgoalconfig.resize(7, 0.0);
   params->vgoalconfig[0] = 0.3;
   params->vgoalconfig[1] = M_PI / 3.0;
   params->vgoalconfig[3] = M_PI / 3.0;
   params->vgoalconfig[5] = M_PI / 3.0;
   params->vgoalconfig[6] = 0.3;
   // while constrained to a line in task space
   TaskSpaceRegion tsr;
   tsr.manipind = 0;
   tsr.relativebodyname = "NULL";
   tsr.relativelinkname = "NULL";
   robot->SetActiveDOFValues(params->vinitialconfig);
   tsr.T0_w = robot->GetActiveManipulator()->GetEndEffectorTransform();
   tsr.Tw_e.identity();
   tsr.Bw[0][0] =  0.; tsr.Bw[0][1] =  0.;
   tsr.Bw[1][0] = -3.; tsr.Bw[1][1] =  3.;
   tsr.Bw[2][0] =  0.; tsr.Bw[2][1] =  0.;
   tsr.Bw[3][0] =  0.; tsr.Bw[3][1] =  0.;
   tsr.Bw[4][0] =  0.; tsr.Bw[4][1] =  0.;
   tsr.Bw[5][0] =  0.; tsr.Bw[5][1] =  0.;
   TaskSpaceRegionChain chain;
   chain.TSRChain.push_back(tsr);
   rob_bSampleGoalFromChain(chain) = false;
   rob_bSampleStartFromChain(chain) = false;
   rob_bConstrainToChain(chain) = true;
   rob_mimicbodyname(chain) = "NULL";
   chain.Initialize(env);
   params->vTSRChains.push_back(chain);
   params->Validate();
   // call planner
   OpenRAVE::PlannerBasePtr planner = OpenRAVE::RaveCreatePlanner(env, "CBiRRT");
   ASSERT_TRUE(planner);
   bool success = planner->InitPlan(robot, params);
   ASSERT_TRUE(success);
   OpenRAVE::TrajectoryBasePtr traj = OpenRAVE::RaveCreateTrajectory(env);
   ASSERT_TRUE(traj);
   OpenRAVE::PlannerStatus status = planner->PlanPath(traj);
   ASSERT_EQ(status, OpenRAVE::PS_HasSolution);
   // validate trajectory
   const OpenRAVE::ConfigurationSpecification & tracj_cspec = traj->GetConfigurationSpecification();
   unsigned int gidx;
   for (gidx=0; gidx<tracj_cspec._vgroups.size(); gidx++)
      if (tracj_cspec._vgroups[gidx].name.substr(0,13) == "joint_values ")
         break;
   ASSERT_LT(gidx, tracj_cspec._vgroups.size());
   const OpenRAVE::ConfigurationSpecification::Group & group = tracj_cspec._vgroups[gidx];
   ASSERT_EQ(group.dof, 7);
   // validate endpoints
   ASSERT_GE(traj->GetNumWaypoints(), 2);
   std::vector<OpenRAVE::dReal> traj_start;
   std::vector<OpenRAVE::dReal> traj_end;
   traj->GetWaypoint(0, traj_start);
   traj->GetWaypoint(traj->GetNumWaypoints()-1, traj_end);
   for (unsigned int i=0; i<7; i++)
   {
      ASSERT_DOUBLE_EQ(params->vinitialconfig[i], traj_start[group.offset+i]);
      ASSERT_DOUBLE_EQ(params->vgoalconfig[i], traj_end[group.offset+i]);
   }
   // validate constraint along trajectory
   OpenRAVE::RobotBasePtr chainrobot;
   chain.RobotizeTSRChain(env, chainrobot);
   std::vector<OpenRAVE::dReal> chainjointvals(chain.GetNumDOF());
   Transform chaintxclosest;
   for (unsigned int wpi=0; wpi<traj->GetNumWaypoints(); wpi++)
   {
      std::vector<OpenRAVE::dReal> wp;
      traj->GetWaypoint(wpi, wp);
      std::vector<OpenRAVE::dReal> q(&wp[group.offset], &wp[group.offset+group.dof]);
      robot->SetActiveDOFValues(q);
      EXPECT_NEAR(chain.GetClosestTransform(
         robot->GetActiveManipulator()->GetEndEffectorTransform(),
         &chainjointvals[0], chaintxclosest), 0., TSR_DISTANCE_TOLERANCE);
   }
   env->Destroy();
   OpenRAVE::RaveDestroy();
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
