/* File: test_params.cpp
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <openrave/openrave.h>
#include <openrave-core.h>

#include <gtest/gtest.h>

#include "../stdafx.h"

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
ROBBER(TaskSpaceRegionChain, RobotBasePtr, _pmimicbody)
ROBBER(TaskSpaceRegionChain, std::vector<int>, _mimicinds)
} // anonymous namespace

// compare transforms exactly
void transform_eq(const Transform & ta, const Transform & tb)
{
   ASSERT_EQ(ta.rot.x, tb.rot.x);
   ASSERT_EQ(ta.rot.y, tb.rot.y);
   ASSERT_EQ(ta.rot.z, tb.rot.z);
   ASSERT_EQ(ta.rot.w, tb.rot.w);
   ASSERT_EQ(ta.trans.x, tb.trans.x);
   ASSERT_EQ(ta.trans.y, tb.trans.y);
   ASSERT_EQ(ta.trans.z, tb.trans.z);
   ASSERT_EQ(ta.trans.w, tb.trans.w);
}

void test_params(const CBirrtParameters & from)
{
   CBirrtParameters to;
   std::stringstream ss1;
   ss1 << from;
   ss1 >> to;
   
   ASSERT_EQ(from.bgrabbed, to.bgrabbed);
   ASSERT_EQ(from.bsamplingstart, to.bsamplingstart);
   ASSERT_EQ(from.bsamplinggoal, to.bsamplinggoal);
   ASSERT_EQ(from.Psample, to.Psample);
   ASSERT_EQ(from.bsmoothpath, to.bsmoothpath);
   ASSERT_EQ(from.smoothingitrs, to.smoothingitrs);
   ASSERT_EQ(from.timelimit, to.timelimit);
   ASSERT_EQ(from.Tattachedik_0.size(), to.Tattachedik_0.size());
   for (unsigned int i=0; i<from.Tattachedik_0.size(); i++)
      transform_eq(from.Tattachedik_0[i], to.Tattachedik_0[i]);
   ASSERT_EQ(from.vsupportpolyx, to.vsupportpolyx);
   ASSERT_EQ(from.vsupportpolyy, to.vsupportpolyy);
   ASSERT_EQ(from.vTSRChains.size(), to.vTSRChains.size());
   for (unsigned int i=0; i<from.vTSRChains.size(); i++)
   {
      const TaskSpaceRegionChain & fc = from.vTSRChains[i];
      const TaskSpaceRegionChain & tc = to.vTSRChains[i];
      ASSERT_EQ(fc.TSRChain.size(), tc.TSRChain.size());
      for (unsigned int j=0; j<fc.TSRChain.size(); j++)
      {
         const TaskSpaceRegion & fr = fc.TSRChain[j];
         const TaskSpaceRegion & tr = tc.TSRChain[j];
         ASSERT_EQ(fr.manipind, tr.manipind);
         ASSERT_EQ(fr.relativebodyname, tr.relativebodyname);
         ASSERT_EQ(fr.relativelinkname, tr.relativelinkname);
         ASSERT_EQ(fr.prelativetolink, tr.prelativetolink);
         transform_eq(fr.T0_w, tr.T0_w);
         transform_eq(fr.Tw_e, tr.Tw_e);
         for (int k=0; k<12; k++)
            ASSERT_EQ(fr.Bw[k/2][k%2], tr.Bw[k/2][k%2]);
      }
      ASSERT_EQ(rob_bSampleGoalFromChain(fc), rob_bSampleGoalFromChain(tc));
      ASSERT_EQ(rob_bSampleStartFromChain(fc), rob_bSampleStartFromChain(tc));
      ASSERT_EQ(rob_bConstrainToChain(fc), rob_bConstrainToChain(tc));
      ASSERT_EQ(rob__pmimicbody(fc), rob__pmimicbody(tc));
      ASSERT_EQ(rob__mimicinds(fc), rob__mimicinds(tc));
   }
   ASSERT_EQ(from.vikguess, to.vikguess);
   ASSERT_EQ(from.bikfastsinglesolution, to.bikfastsinglesolution);
   ASSERT_EQ(from.pplannerstate, to.pplannerstate);
}

TEST(SerializationTestCase, Test1)
{
   CBirrtParameters params;
   params.bgrabbed = true;
   params.bsamplingstart = true;
   params.bsamplinggoal = false;
   params.Psample = 0.2;
   params.bsmoothpath = false;
   params.smoothingitrs = 314;
   params.timelimit = 284.12;
   params.Tattachedik_0.push_back(Transform());
   params.Tattachedik_0.push_back(Transform(Vector(1,0,0,0), Vector(0,0,0)));
   params.Tattachedik_0.push_back(Transform(Vector(0.5,0.5,0.5,0.5), Vector(14.3,6454.1,0.003)));
   params.vsupportpolyx.push_back(2542646.45);
   params.vsupportpolyx.push_back(0.00135);
   params.vsupportpolyy.push_back(34.7);
   params.vsupportpolyy.push_back(1.0);
   params.bikfastsinglesolution = true;
   params.vikguess.push_back(0.315);
   params.vikguess.push_back(0.00034);
   params.vikguess.push_back(4254265.45);
   enum PlannerState ps;
   params.pplannerstate = &ps;
   test_params(params);
}

int main(int argc, char **argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
