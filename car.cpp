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
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include "CollisionChecking.h"
#include "AO-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // Project 4D state (x, y, theta, v) to 2D (x, y)
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        projection(0) = s->values[0];  // x
        projection(1) = s->values[1];  // y
    }
};

void carODE(const ompl::control::ODESolver::StateType &q, 
            const ompl::control::Control *control,
            ompl::control::ODESolver::StateType &qdot)
{
    // State: q = (x, y, theta, v)
    const double theta = q[2];
    const double v = q[3];
    
    // Control: u = (omega, alpha)
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double omega = u[0];  // angular velocity
    const double alpha = u[1];  // acceleration
    
    // Car dynamics
    qdot.resize(q.size(), 0);
    qdot[0] = v * cos(theta);  // x_dot
    qdot[1] = v * sin(theta);  // y_dot
    qdot[2] = omega;           // theta_dot
    qdot[3] = alpha;           // v_dot
}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // Do not change the obstacles here. These are the same obstacles used for grading.
    obstacles.emplace_back(Rectangle{5.0, -2.0, 7, 5});
    obstacles.emplace_back(Rectangle{-4, 5, 16, 2});
    obstacles.emplace_back(Rectangle{-4, -2, 7, 4});
    obstacles.emplace_back(Rectangle{8, 3, 4, 2});
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    // State space: R^4 for (x, y, theta, v)
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(4);
    
    ompl::base::RealVectorBounds bounds(4);
    bounds.setLow(0, -10);   // x
    bounds.setHigh(0, 10);

    bounds.setLow(1, -10);   // y
    bounds.setHigh(1, 10);

    bounds.setLow(2, -M_PI); // theta
    bounds.setHigh(2, M_PI);
    
    bounds.setLow(3, -1);    // v
    bounds.setHigh(3, 1);
    
    space->setBounds(bounds);
    
    // Control space: R^2 for (omega, alpha)
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, 2);
    
    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -0.3);   // omega
    cbounds.setHigh(0, 0.3);
    cbounds.setLow(1, -0.3);   // alpha
    cbounds.setHigh(1, 0.3);
    
    cspace->setBounds(cbounds);
    
    // SimpleSetup
    auto ss = std::make_shared<ompl::control::SimpleSetup>(cspace);
    
    // Propagation parameters
    ss->getSpaceInformation()->setPropagationStepSize(0.1);
    ss->getSpaceInformation()->setMinMaxControlDuration(1, 20);
    
    // ODE solver
    auto odeSolver = std::make_shared<ompl::control::ODEBasicSolver<>>(
        ss->getSpaceInformation(), &carODE);
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
    
    // State validity checker using collision checking
    ss->setStateValidityChecker(
        [obstacles](const ompl::base::State *state) {
            const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
            const double x = s->values[0];
            const double y = s->values[1];
            const double v = s->values[3];
            
            // Check velocity bounds
            if (v < -1.0 || v > 1.0)
                return false;
            
            // Check collision with obstacles (point robot)
            return isValidPoint(x, y, obstacles);
        });
    
    // Start state: qstart = (1, -5, 0, 0)
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start[0] = 1.0;   // x
    start[1] = -5.0;  // y
    start[2] = 0.0;   // theta
    start[3] = 0.0;   // v
    
    // Goal state: qgoal = (7, 4, 3.14, 0), tolerance 0.5
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal[0] = 7.0;    // x
    goal[1] = 4.0;    // y
    goal[2] = 3.14;   // theta
    goal[3] = 0.0;    // v
    
    ss->setStartAndGoalStates(start, goal, 0.5);
    
    // Register projection for KPIECE1
    space->registerDefaultProjection(std::make_shared<CarProjection>(space.get()));
    
    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice, double timeout)
{
    // Set planner based on choice
    std::string plannerName;
    if (choice == 1)
    {
        auto planner = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        plannerName = "KPIECE1";
    }
    else if (choice == 2)
    {
        auto planner = std::make_shared<ompl::control::SST>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        plannerName = "SST";
    }
    else if (choice == 3)
    {
        auto planner = std::make_shared<ompl::control::AORRT>(ss->getSpaceInformation());
        ss->setPlanner(planner);
        plannerName = "AO-RRT";
    }
    
    std::cout << "Using " << plannerName << " planner" << std::endl;
    
    ss->setup();
    
    std::cout << "Planning for " << timeout << " seconds..." << std::endl;
    ompl::base::PlannerStatus solved = ss->solve(timeout);
    
    if (solved)
    {
        std::cout << "Found solution!" << std::endl;
        
        ompl::control::PathControl &path = ss->getSolutionPath();
        
        // Interpolate for visualization
        path.interpolate();
        
        // Convert to geometric path
        ompl::geometric::PathGeometric gpath = path.asGeometric();
        gpath.interpolate();
        
        // Save to file with planner and timeout in filename
        std::string filename = "car_path_" + plannerName + "_" + std::to_string((int)timeout) + "s.txt";
        std::ofstream fout(filename);
        gpath.printAsMatrix(fout);
        fout.close();
        
        std::cout << "Path saved to " << filename << " with " << gpath.getStateCount() << " waypoints" << std::endl;
        std::cout << "Visualize with: python3 ../visualize_car.py " << filename << " " << plannerName << " " << (int)timeout << std::endl;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    // Manual CSV-based benchmarking (OMPL 1.6.0 database is broken)
    std::vector<std::string> plannerNames = {"KPIECE1", "SST", "AO-RRT"};
    const int numRuns = 20; // 20
    
    std::ofstream csv("benchmark_car_results.csv");
    csv << "Planner,Run,Time,Solved,PathLength,NumStates\n";
    
    std::cout << "\nBenchmark summary:" << std::endl;
    std::cout << "  - planners: (KPIECE1, SST, AO-RRT)" << std::endl;
    std::cout << "  - " << numRuns << " runs per planner" << std::endl;
    std::cout << "  - 200 seconds limit per run" << std::endl;
    std::cout << "  - Total: " << (3 * numRuns) << " runs\n" << std::endl;
    
    for (const auto& plannerName : plannerNames)
    {
        std::cout << "\nBenchmarking " << plannerName << "..." << std::endl;
        
        for (int run = 0; run < numRuns; ++run)
        {
            // Create fresh setup for each run
            ss->clear();
            
            // Set planner
            if (plannerName == "KPIECE1")
                ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
            else if (plannerName == "SST")
                ss->setPlanner(std::make_shared<ompl::control::SST>(ss->getSpaceInformation()));
            else if (plannerName == "AO-RRT")
                ss->setPlanner(std::make_shared<ompl::control::AORRT>(ss->getSpaceInformation()));
            
            ss->setup();
            
            // Time the solve
            auto start = std::chrono::high_resolution_clock::now();
            ompl::base::PlannerStatus solved = ss->solve(200.0);
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
                     << ": " << (solved ? "yes" : "no") 
                     << " (" << elapsed << "s)" << std::endl;
        }
    }
    
    csv.close();
    std::cout << "\nBenchmark complete! Results saved to benchmark_car_results.csv" << std::endl;
    std::cout << "Analyze with: python3 ../analyze_benchmark_csv.py benchmark_car_results.csv car" << std::endl;
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) KPIECE1" << std::endl;
            std::cout << " (2) SST" << std::endl;
            std::cout << " (3) AO-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        int timeoutChoice;
        do
        {
            std::cout << "Timeout? " << std::endl;
            std::cout << " (1)  5 seconds (quick test)" << std::endl;
            std::cout << " (2) 1000 seconds (long)" << std::endl;

            std::cin >> timeoutChoice;
        } while (timeoutChoice < 1 || timeoutChoice > 2);

        double timeouts[] = {5.0, 1000.0};
        double timeout = timeouts[timeoutChoice - 1];

        planCar(ss, planner, timeout);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}