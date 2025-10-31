///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Luis Alzamora Josh Ethan
//////////////////////////////////////

#ifndef AORRT_H
#define AORRT_H

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/OptimizationObjective.h>

namespace ompl
{
    namespace control
    {
        /**
         * @brief Asymptotically-Optimal RRT (AO-RRT)
         * 
         * This planner implements the AO-RRT algorithm as described in:
         * Hauser and Zhou, "Asymptotically optimal planning by feasible 
         * kinodynamic planning in a state-cost space", IEEE TRO, 2016.
         * 
         * The key idea is to run a feasible kinodynamic planner (RRT) while
         * treating cumulative trajectory cost as an additional dimension and
         * progressively tightening a global cost bound.
         */
        class AORRT : public base::Planner
        {
        public:
            /** @brief Constructor */
            AORRT(const SpaceInformationPtr &si);

            /** @brief Destructor */
            ~AORRT() override;

            /** @brief Solve the planning problem */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** @brief Clear the planner's data structures */
            void clear() override;

            /** @brief Setup the planner */
            void setup() override;

            /** @brief Get planner data */
            void getPlannerData(base::PlannerData &data) const override;

            /** @brief Set the goal bias (probability of sampling the goal) */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** @brief Get the goal bias */
            double getGoalBias() const
            {
                return goalBias_;
            }

        protected:
            /** @brief Representation of a motion (node in the tree) */
            class Motion
            {
            public:
                Motion() = default;

                Motion(const SpaceInformation *si)
                    : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** @brief The state at this node */
                base::State *state{nullptr};

                /** @brief The control that was applied to reach this state */
                Control *control{nullptr};

                /** @brief The duration the control was applied */
                double steps{0};

                /** @brief The parent motion in the tree */
                Motion *parent{nullptr};

                /** @brief The cumulative cost to reach this state */
                double cost{0.0};
            };

            /** @brief Free the memory for a motion */
            void freeMotion(Motion *motion);
            
            /** @brief Free all memory */
            void freeMemory();

            /** @brief Sample a state */
            base::State *sampleState();

            /** @brief Select a motion to extend from */
            Motion *selectMotion();

            /** @brief Propagate from a motion using a control */
            Motion *propagate(Motion *motion, Control *control, int steps, base::State *result);

            /** @brief Add a motion to the tree */
            void addMotion(Motion *motion);

            /** @brief Check if we have a solution */
            bool haveSolution(Motion *motion);

            /** @brief Extract the solution path */
            void getSolution(Motion *motion, std::vector<Motion *> &path);

            /** @brief The control space information */
            const SpaceInformation *siC_;

            /** @brief State sampler */
            base::StateSamplerPtr sampler_;

            /** @brief Control sampler */
            ControlSamplerPtr controlSampler_;

            /** @brief Directed control sampler (for goal biasing) */
            DirectedControlSamplerPtr directedControlSampler_;

            /** @brief Nearest neighbors data structure */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** @brief Goal bias (probability of sampling goal) */
            double goalBias_{0.05};

            /** @brief Random number generator */
            RNG rng_;

            /** @brief The most recent goal motion */
            Motion *lastGoalMotion_{nullptr};

            /** @brief Best cost found so far (initially infinity) */
            double bestCost_{std::numeric_limits<double>::infinity()};

            /** @brief Optimization objective for computing costs */
            base::OptimizationObjectivePtr opt_;

            /** @brief Minimum control duration */
            unsigned int minControlDuration_{1};

            /** @brief Maximum control duration */
            unsigned int maxControlDuration_{20};
        };
    }  // namespace control
}  // namespace ompl

#endif  // AORRT_H