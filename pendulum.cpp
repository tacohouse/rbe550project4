///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Luis Alzamora Josh Ethan
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <chrono>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // The dimension of your projection for the pendulum
        return 2;  // Project to 2D: (theta, omega)
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Your projection for the pendulum
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        projection(0) = s->values[0];  // theta
        projection(1) = s->values[1];  // omega
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q, 
                 const ompl::control::Control *control,
                 ompl::control::ODESolver::StateType &qdot)
{
    // Fill in the ODE for the pendulum's dynamics
    const double theta = q[0];
    const double omega = q[1];
    
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];
    
    const double g = 9.81;
    
    qdot.resize(q.size(), 0);
    qdot[0] = omega;                        // theta_dot = omega
    qdot[1] = -g * cos(theta) + torque;     // omega_dot = -g*cos(theta) + tau
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    // Create and setup the pendulum's state space, control space, validity checker, 
    // everything you need for planning.
    
    // State space: R^2 for (theta, omega)
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -M_PI);
    bounds.setHigh(0, M_PI);
    bounds.setLow(1, -10);
    bounds.setHigh(1, 10);
    space->setBounds(bounds);
    
    // Control space
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, 1);
    
    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);
    
    // SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(cspace);
    
    // Propagation step size and control duration
    ss->getSpaceInformation()->setPropagationStepSize(0.05);
    ss->getSpaceInformation()->setMinMaxControlDuration(1, 20);
    
    // ODE solver
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(
        ss->getSpaceInformation(), &pendulumODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    
    // State validity checker
    ss->setStateValidityChecker([](const ompl::base::State *state) {
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        return s->values[1] >= -10.0 && s->values[1] <= 10.0;
    });
    
    // Start and goal
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start[0] = -M_PI / 2.0;
    start[1] = 0.0;
    
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal[0] = M_PI / 2.0;
    goal[1] = 0.0;
    
    ss->setStartAndGoalStates(start, goal, 0.1);
    
    // Register projection for KPIECE1
    space->registerDefaultProjection(std::make_shared<PendulumProjection>(space.get()));
    
    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // Do some motion planning for the pendulum
    // choice is what planner to use.
    
    if (choice == 1)
    {
        auto planner = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    else if (choice == 2)
    {
        auto planner = std::make_shared<ompl::control::EST>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    else if (choice == 3)
    {
        auto planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        ss->setPlanner(planner);
    }
    
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(30.0);
    
    if (solved)
    {
        std::cout << "Found solution!" << std::endl;
        
        ompl::control::PathControl &path = ss->getSolutionPath();
        
        // Interpolate to get more waypoints
        path.interpolate();
        
        // Convert to geometric path for visualization
        ompl::geometric::PathGeometric gpath = path.asGeometric();
        
        // Further interpolate the geometric path for smoothness
        gpath.interpolate();
        
        // Save to file
        std::ofstream fout("pendulum_path.txt");
        gpath.printAsMatrix(fout);
        fout.close();
        
        std::cout << "Path saved to pendulum_path.txt with " << gpath.getStateCount() << " waypoints" << std::endl;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss)
{
    // Manual CSV-based benchmarking (OMPL 1.6.0 database is broken)
    std::vector<std::string> plannerNames = {"RRT", "EST", "KPIECE1"};
    const int numRuns = 20;  // Testing with 2 runs (change to 20 for final)
    
    std::ofstream csv("benchmark_results.csv");
    csv << "Planner,Run,Time,Solved,PathLength,NumStates\n";
    
    std::cout << "\nRunning benchmark with:" << std::endl;
    std::cout << "  - 3 planners (RRT, EST, KPIECE1)" << std::endl;
    std::cout << "  - " << numRuns << " runs per planner" << std::endl;
    std::cout << "  - 30 seconds per run" << std::endl;
    std::cout << "  - Total: " << (3 * numRuns) << " runs\n" << std::endl;
    
    for (const auto& plannerName : plannerNames)
    {
        std::cout << "\nBenchmarking " << plannerName << "..." << std::endl;
        
        for (int run = 0; run < numRuns; ++run)
        {
            // Create fresh setup for each run
            ss->clear();
            
            // Set planner
            if (plannerName == "RRT")
                ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
            else if (plannerName == "EST")
                ss->setPlanner(std::make_shared<ompl::control::EST>(ss->getSpaceInformation()));
            else if (plannerName == "KPIECE1")
                ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
            
            ss->setup();
            
            // Time the solve
            auto start = std::chrono::high_resolution_clock::now();
            ompl::base::PlannerStatus solved = ss->solve(30.0);
            auto end = std::chrono::high_resolution_clock::now();
            
            double elapsed = std::chrono::duration<double>(end - start).count();
            
            // Get path info
            double pathLength = 0;
            int numStates = 0;
            if (solved)
            {
                ompl::control::PathControl path = ss->getSolutionPath();
                pathLength = path.length();
                numStates = path.getStateCount();
            }
            
            // Write to CSV
            csv << plannerName << "," << (run + 1) << "," << elapsed << "," 
                << (solved ? 1 : 0) << "," << pathLength << "," << numStates << "\n";
            csv.flush();
            
            std::cout << "  Run " << (run + 1) << "/" << numRuns 
                     << ": " << (solved ? "✓" : "✗") 
                     << " (" << elapsed << "s)" << std::endl;
        }
    }
    
    csv.close();
    std::cout << "\nBenchmark complete! Results saved to benchmark_results.csv" << std::endl;
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) EST" << std::endl;
            std::cout << " (3) KPIECE1" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}