///////////////////////////////////////
// RBE550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include "AO-RRT.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>

ompl::control::AORRT::AORRT(const SpaceInformationPtr &si) : base::Planner(si, "AORRT")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    siC_ = si.get();

    // Set default control duration bounds (will be updated in setup)
    minControlDuration_ = 1;
    maxControlDuration_ = 20;

    Planner::declareParam<double>("goal_bias", this, &AORRT::setGoalBias, &AORRT::getGoalBias, "0.:.05:1.");
}

ompl::control::AORRT::~AORRT()
{
    freeMemory();
}

void ompl::control::AORRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

void ompl::control::AORRT::setup()
{
    base::Planner::setup();
    
    // Set up samplers
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler();
    
    // Control duration bounds are already set in constructor
    
    // Set up nearest neighbors data structure
    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                { return si_->distance(a->state, b->state); });
    }
    
    // Clear any existing data
    if (nn_->size() > 0)
        clear();
}

void ompl::control::AORRT::clear()
{
    base::Planner::clear();
    
    sampler_.reset();
    controlSampler_.reset();
    
    freeMemory();
    
    if (nn_)
        nn_->clear();
    
    lastGoalMotion_ = nullptr;
    bestCost_ = std::numeric_limits<double>::infinity();
}

ompl::base::PlannerStatus ompl::control::AORRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    
    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    
    // Add start states to tree
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        motion->cost = 0.0;
        nn_->add(motion);
    }
    
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: No valid start states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    
    OMPL_INFORM("%s: Starting planning with %u states", getName().c_str(), nn_->size());
    
    Motion *solution = nullptr;
    Motion *approxSolution = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();
    
    auto *rmotion = new Motion(siC_);
    base::State *xstate = si_->allocState();
    
    unsigned int iterations = 0;
    
    // Main planning loop
    while (!ptc)
    {
        iterations++;
        
        // Sample a random state
        Motion *nmotion = selectMotion();
        
        // Sample a random control
        controlSampler_->sample(rmotion->control);
        Control *rctrl = rmotion->control;
        unsigned int cd = rng_.uniformInt(minControlDuration_, maxControlDuration_);
        
        // Propagate the system
        unsigned int duration = siC_->propagateWhileValid(nmotion->state, rctrl, cd, xstate);
        
        if (duration >= minControlDuration_)
        {
            // Compute cost of this motion (simple: just the duration/distance)
            double motionCost = duration * siC_->getPropagationStepSize();
            double newCost = nmotion->cost + motionCost;
            
            // AO-RRT key modification: only accept if cost is less than current best
            if (newCost < bestCost_)
            {
                // Create new motion
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, xstate);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = duration;
                motion->parent = nmotion;
                motion->cost = newCost;
                
                // Add to tree
                nn_->add(motion);
                
                // Check if this reaches the goal
                double dist = 0.0;
                bool satisfied = goal->isSatisfied(motion->state, &dist);
                
                if (satisfied)
                {
                    approxDist = dist;
                    solution = motion;
                    
                    // Update best cost (this is the key AO-RRT mechanism)
                    if (newCost < bestCost_)
                    {
                        bestCost_ = newCost;
                        lastGoalMotion_ = motion;
                        OMPL_INFORM("%s: Found solution with cost %.3f at iteration %u", 
                                   getName().c_str(), bestCost_, iterations);
                    }
                }
                
                if (dist < approxDist)
                {
                    approxDist = dist;
                    approxSolution = motion;
                }
            }
            // If newCost >= bestCost_, the motion is pruned (not added to tree)
        }
    }
    
    bool solved = false;
    bool approximate = false;
    
    if (solution == nullptr)
    {
        solution = approxSolution;
        approximate = true;
    }
    
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;
        
        // Construct the solution path
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }
        
        // Create path
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
        {
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        }
        
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxDist, getName());
    }
    
    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    
    OMPL_INFORM("%s: Created %u states in %u iterations. Final solution cost: %.3f",
                getName().c_str(), nn_->size(), iterations, bestCost_);
    
    return base::PlannerStatus(solved, approximate);
}

ompl::control::AORRT::Motion *ompl::control::AORRT::selectMotion()
{
    // Sample state
    base::State *rstate = si_->allocState();
    sampler_->sampleUniform(rstate);
    
    // Goal biasing
    if (rng_.uniform01() < goalBias_ && lastGoalMotion_ != nullptr)
    {
        si_->freeState(rstate);
        return lastGoalMotion_;
    }
    
    // Create temporary motion to find nearest
    Motion tempMotion;
    tempMotion.state = rstate;
    
    // Find nearest neighbor
    Motion *nmotion = nn_->nearest(&tempMotion);
    si_->freeState(rstate);
    
    return nmotion;
}

void ompl::control::AORRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    
    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);
    
    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
    
    for (auto &motion : motions)
    {
        if (motion->parent)
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state),
                        base::PlannerDataVertex(motion->state));
        }
        else
        {
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        }
    }
}
