///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "RTP.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

// TODO: Implement RTP as described

ompl::geometric::RTP::RTP(const base::SpaceInformationPtr &si, bool addIntermediateStates): base::Planner(si, addIntermediateStates ? "RRTintermediate" : "RTP")
{
     specs_.approximateSolutions = true;
     specs_.directed = true;
  
     Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");
     Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
     Planner::declareParam<bool>("intermediate_states", this, &RTP::setIntermediateStates, &RTP::getIntermediateStates,
                                 "0,1");
     addIntermediateStates_ = addIntermediateStates;

}


 ompl::geometric::RTP::~RTP()
 {
     freeMemory();
 }
  
 void ompl::geometric::RTP::clear()
 {
     Planner::clear();
     sampler_.reset();
     freeMemory();
     if (!existing_motions.empty()) {
        existing_motions.clear();
    }

    lastGoalMotion_ = nullptr;
 }
 
 void ompl::geometric::RTP::setup()
 {
     Planner::setup();
     tools::SelfConfig sc(si_, getName());
     sc.configurePlannerRange(maxDistance_);
 }
 
 
 
void ompl::geometric::RTP::freeMemory()
{
   
    if (!existing_motions.empty())
    {
        for (auto &motion : existing_motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}
 
 ompl::base::PlannerStatus ompl::geometric::RTP::solve(const base::PlannerTerminationCondition &ptc)
 {
 	checkValidity();
 	base::Goal *goal = pdef_->getGoal().get();
     	auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
 	while (const base::State *st = pis_.nextStart())
     	{
         auto *motion = new Motion(si_);
         si_->copyState(motion->state, st);
         existing_motions.push_back(motion);
     	}
 	if (existing_motions.size() == 0)
     	{
         	OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
         	return base::PlannerStatus::INVALID_START;
     	}
     	
     	if (!sampler_)
         sampler_ = si_->allocStateSampler();
         
      	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), existing_motions.size());
     	Motion *solution = nullptr;
     	Motion *approxsol = nullptr;
     	double approxdif = std::numeric_limits<double>::infinity();
     	auto *rmotion = new Motion(si_);
     	base::State *rstate = rmotion->state;
     	base::State *xstate = si_->allocState();
     	
     	while(!ptc){
     	
     	/* sample random state (with goal biasing) */
         if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
             goal_s->sampleGoal(rstate);
         else
             sampler_->sampleUniform(rstate);
        
        Motion *rand_motion = existing_motions[rand() % existing_motions.size()];
        base::State *dstate = rstate;
        
        /* find state to add */
         double d = si_->distance(rand_motion->state, rstate);
         if (d > maxDistance_)
         {
             si_->getStateSpace()->interpolate(rand_motion->state, rstate, maxDistance_ / d, xstate);
             dstate = xstate;
         }
         
         if (si_->checkMotion(rand_motion->state, dstate))
         {
             if (addIntermediateStates_)
             {
                 std::vector<base::State *> states;
                 const unsigned int count = si_->getStateSpace()->validSegmentCount(rand_motion->state, dstate);
  
                 if (si_->getMotionStates(rand_motion->state, dstate, states, count, true, true))
                     si_->freeState(states[0]);
  
                 for (std::size_t i = 1; i < states.size(); ++i)
                 {
                     auto *motion = new Motion;
                     motion->state = states[i];
                     motion->parent = rand_motion;
                     existing_motions.push_back(motion);
                     
                     rand_motion = motion;
                 }
             }
             else
             {
                 auto *motion = new Motion(si_);
                 si_->copyState(motion->state, dstate);
                 motion->parent = rand_motion;
                 existing_motions.push_back(motion);
                 
  		 rand_motion = motion;
             }
             
             double dist = 0.0;
             bool sat = goal->isSatisfied(rand_motion->state, &dist);
             if (sat)
             {
                 approxdif = dist;
                 solution = rand_motion;
                 break;
             }
             if (dist < approxdif)
             {
                 approxdif = dist;
                 approxsol = rand_motion;
             }
            
           }
     	}
     	
     	bool solved = false;
     	bool approximate = false;
     	if (solution == nullptr)
     	{
         solution = approxsol;
         approximate = true;
     	}
  
     	if (solution != nullptr)
     	{
         lastGoalMotion_ = solution;
  
         /* construct the solution path */
         std::vector<Motion *> mpath;
         while (solution != nullptr)
         {
             mpath.push_back(solution);
             solution = solution->parent;
         }
  
         /* set the solution path */
         auto path(std::make_shared<PathGeometric>(si_));
         for (int i = mpath.size() - 1; i >= 0; --i)
             path->append(mpath[i]->state);
         pdef_->addSolutionPath(path, approximate, approxdif, getName());
         solved = true;
     	}
  
     	si_->freeState(xstate);
     	if (rmotion->state != nullptr)
         si_->freeState(rmotion->state);
     	delete rmotion;
  
     	OMPL_INFORM("%s: Created %u states", getName().c_str(), existing_motions.size());
  
     	return {solved, approximate};
 }
 
 
  void ompl::geometric::RTP::getPlannerData(base::PlannerData &data) const
 {
     Planner::getPlannerData(data);
  
     if (lastGoalMotion_ != nullptr)
         data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
  
     for (auto &motion : existing_motions)
     {
         if (motion->parent == nullptr)
             data.addStartVertex(base::PlannerDataVertex(motion->state));
         else
             data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
     }
 }

 
 
 
 
 
 
 
 
 
 
 
 
