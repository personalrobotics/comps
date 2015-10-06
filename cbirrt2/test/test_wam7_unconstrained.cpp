/* File: test_wam7.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>
#include <openrave-core.h>

#include <gtest/gtest.h>

TEST(Wam7UnconstrainedTestCase, SimpleTest)
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
   OpenRAVE::PlannerBase::PlannerParametersPtr params(new OpenRAVE::PlannerBase::PlannerParameters());
   params->SetRobotActiveJoints(robot);
   params->SetConfigurationSpecification(env, robot->GetActiveConfigurationSpecification());
   // trivial planning query
   params->vinitialconfig.clear();
   params->vinitialconfig.resize(7, 0.0);
   params->vgoalconfig.clear();
   params->vgoalconfig.resize(7, 0.0); params->vgoalconfig[0] = 1.0;
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
   env->Destroy();
   OpenRAVE::RaveDestroy();
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
