///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Shreyas Poyrekar!!
//////////////////////////////////////

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H


#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        // TODO: Implement RTP as described

        class RTP : public base::Planner
        {
        public:
        	RTP(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);
        	
        	~RTP() override; //oride to keep track of the base class redefinition 

        	void getPlannerData(base::PlannerData &data) const override; //function to ge the planner data

        	base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override; // function to get the planner exit status
        	
        	void clear() override; // clear all internal data structures
        	
        	
		// set goal bias func        	
        	void setGoalBias(double goalBias)
             	{
                 goalBias_ = goalBias;
             	}
  		// get goal bias func
             	double getGoalBias() const
             	{
                 return goalBias_;
             	}
  		// get intermediate states func
             	bool getIntermediateStates() const
             	{
                 return addIntermediateStates_;
             	}
  		// set indermediate states
             	void setIntermediateStates(bool addIntermediateStates)
             	{
                 addIntermediateStates_ = addIntermediateStates;
             	}
  		// set range
             	void setRange(double distance)
             	{
                 maxDistance_ = distance;
             	}
  		// get range
             	double getRange() const
             	{
                 return maxDistance_;
             	}
             
             	void setup() override;
             	
        protected:
        	class Motion
             	{
             	public:
                 	Motion() = default;
  
                 	Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                 	{
                 	}
  
                	~Motion() = default;
  
                 	base::State *state{nullptr};
  	
                	Motion *parent{nullptr};
             	};
             	
             	void freeMemory();
             	
             	double distanceFunction(const Motion *a, const Motion *b) const
             	{
                 return si_->distance(a->state, b->state);
             	}
             	
             	base::StateSamplerPtr sampler_;
             	
             	std::vector<Motion*> existing_motions;
             	
             	double goalBias_{.05};
  
             	double maxDistance_{0.};
  
             	bool addIntermediateStates_;
  
             	RNG rng_;
  
             	Motion *lastGoalMotion_{nullptr};
        };

    }  // namespace geometric
}  // namespace ompl

#endif
