// OMPL setup
#include <ompl/config.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/RandomNumbers.h>
#include <omplapp/config.h>

// C++ library
#include <iostream>
#include <valarray>
#include <limits>
#include <fstream>
#include <vector>
#include <math.h>

// Planner
#include <ompl/control/planners/rrt/RRT.h>
#include "SMR.h"

//environment parsing
const std::string startStr = "start";
const std::string endStr = "end";
const std::string endPolygon = "ep";

const double epsilon = 0.01;
const double square = 0.025;
const double alpha = 2.0;

// Car Limits
const double turning_radius = M_PI / 3;
const double radius = 0.05;
const double radius_sd [] = {0.01, 0.02};
const double delta = 0.25;
const double delta_sd [] = {0.1, 0.2};

typedef std::pair<double, double> Point2D;
typedef std::vector<Point2D> Rect;

namespace ob = ompl::base;
namespace oc = ompl::control;

double dist(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

bool lineIntersection(Point2D ours0, Point2D ours1, Point2D theirs0, Point2D theirs1)
{
    double theirs_lowerX = std::min(theirs0.first, theirs1.first);
    double theirs_upperX = std::max(theirs0.first, theirs1.first);
    double ours_lowerX = std::min(ours0.first, ours1.first);
    double ours_upperX = std::max(ours0.first, ours1.first);

    // Check if Y range of the lines overlap
    double theirs_lowerY = std::min(theirs0.second, theirs1.second);
    double theirs_upperY = std::max(theirs0.second, theirs1.second);
    double ours_lowerY = std::min(ours0.second, ours1.second);
    double ours_upperY = std::max(ours0.second, ours1.second);

    bool y0_overlap = (ours_lowerY >= theirs_lowerY) && (ours_lowerY <= theirs_upperY);
    bool y1_overlap = (ours_upperY >= theirs_lowerY) && (ours_lowerY <= theirs_upperY);
    if(!(y0_overlap || y1_overlap))
        return false;

    double ours_m = (ours1.second - ours0.second) / (ours1.first - ours0.first);
    double ours_b = ours0.second - ours_m * ours0.first;

    double theirs_m = (theirs1.second - theirs0.second) / (theirs1.first - theirs0.first);
    double theirs_b = theirs0.second - theirs_m * theirs0.first;

    if(isinf(ours_m))
    {
        // Check if X range of the lines overlap
        bool x_overlap = (theirs_lowerX < ours0.first) && (theirs_upperX > ours0.first);
        if(!x_overlap)
            return false;

        double theirs_value = theirs_m * ours0.first + theirs_b;
        return (theirs_value >= ours_lowerY) && (theirs_value <= ours_upperY);
    }

    if(isinf(theirs_m))
    {
        // Check if X range of the lines overlap
        bool x_overlap = (ours_lowerX < theirs0.first) && (ours_upperX > theirs0.first);
        if(!x_overlap)
            return false; 

        double ours_value = ours_m * theirs0.first + ours_b;
        return (ours_value >= theirs_lowerY) && (ours_value <= theirs_upperY);
    }

    // Brute-Force
    for(double pos = ours0.first; pos < ours1.first; pos+=epsilon)
    {
        if(pos >= theirs_lowerX && pos <= theirs_upperX)
        {
            double ours_value = ours_m * pos + ours_b;
            double theirs_value = theirs_m * pos + theirs_b;
            double diff = ours_value - theirs_value;
            if(abs(diff) < epsilon)
                return true;
        }
    }
    return false;
}

// Definition of the ODE for the kinematic car.
void KinematicCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const int u = control->as<oc::DiscreteControlSpace::ControlType>()->value;
    double sign = bool(std::floor(u)) ? 1.0 : -1.0;

    ompl::RNG sample;
    const double Delta = sample.gaussian(delta, delta_sd[u]);
    const double Radius = sample.gaussian(radius, radius_sd[u]);

    // Zero out qdot
    qdot.resize (q.size(), 0);
    qdot[0] = Delta * cos(q[2]);
    qdot[1] = Delta * sin(q[2]);
    qdot[2] = sign * Delta / Radius;
}

// This is a callback method invoked after numerical integration.
void CarPostIntegration (const ob::State* state, const oc::Control* control, const double duration, ob::State *result)
{
    ompl::base::CompoundState* cstate = result->as<ompl::base::CompoundState>();
    ompl::base::SO2StateSpace::StateType* so2state = cstate->as<ompl::base::SO2StateSpace::StateType>(1);

    // Normalize orientation between -pi and pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds (so2state);
}

bool isStateValidCar(const ob::State *state, const double minBound, const double maxBound, const std::vector<Rect> obstacles)
{
    const ompl::base::CompoundState* cstate = state->as<ompl::base::CompoundState>();
    const ompl::base::RealVectorStateSpace::StateType* r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    const ompl::base::SO2StateSpace::StateType* so2state = cstate->as<ompl::base::SO2StateSpace::StateType>(1);

    double x = r2state->values[0];
    double y = r2state->values[1];
    double theta = so2state->value;    

    // Initial Square Robot Points
    std::vector<Point2D> pts;
    pts.push_back(std::make_pair(0, 0));
    pts.push_back(std::make_pair(square, 0));
    pts.push_back(std::make_pair(square, square));
    pts.push_back(std::make_pair(0, square));

    // Transform Square Robot Points to state position
    for(int i = 0; i < pts.size(); ++i)
    {
        double newX = pts[i].first * cos(theta) - pts[i].second * sin(theta) + x;
        double newY = pts[i].first * sin(theta) + pts[i].second * cos(theta) + y;

        if(newX < minBound || newY < minBound || newX > maxBound || newY > maxBound)
        {
            return false;
        }

        pts[i] = std::make_pair(newX, newY);
    }

    for(const Rect& r : obstacles)
    {
        // None of the points of square robot are contained inside of the obstacle 
        for(int j = 0; j < pts.size(); ++j)
        {
            if(pts[j].first >= r[0].first && pts[j].first <= r[2].first && pts[j].second >= r[0].second && pts[j].second <= r[2].second)
                return false;
        }

        // Edge of rectangle r
        for(int i = 0; i < r.size(); ++i)
        {
            // Edge of square robot
            for(int j = 0; j < pts.size(); ++j)
            {
                bool intersection = lineIntersection(pts[j], pts[(j+1) % pts.size()], r[i], r[(i+1) % r.size()]);
                if(intersection)
                {
                    return false;
                }
            }
        }
    }

    return true;
}

/// @cond IGNORE
// turning angle - left=0 or right=1
class CarControlSpace : public oc::DiscreteControlSpace
{
    public:
        CarControlSpace(const ob::StateSpacePtr &stateSpace) : oc::DiscreteControlSpace(stateSpace, 0, 1) {}
};
/// @endcond

void plan(std::vector<Rect> obstacles, std::vector<double> startV, std::vector<double> goalV, int env, bool benchmark = false)
{
    // x, y, theta, b
    ompl::base::StateSpacePtr space(new ompl::base::CompoundStateSpace());
    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
    ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());
    space->as<ompl::base::CompoundStateSpace>()->addSubspace(r2, 1.0);
    space->as<ompl::base::CompoundStateSpace>()->addSubspace(so2, alpha);

    // Create a control space
    oc::ControlSpacePtr cspace(new CarControlSpace(space));

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(isStateValidCar, _1, 0.0, 1.0, obstacles));

    // Use the ODESolver to propagate the system.  Call KinematicCarPostIntegration
    // when integration has finished to normalize the orientation values.
    oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<> (ss.getSpaceInformation(), &KinematicCarODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, CarPostIntegration));
    ss.getSpaceInformation()->setPropagationStepSize(0.10);

    /// create a start state
    ob::ScopedState<> start(space);
    for(int i = 0; i < startV.size(); ++i)
        start[i] = startV[i];

    /// create a goal state
    ob::ScopedState<> goal(space);
    for(int i = 0; i < goalV.size(); ++i)
        goal[i] = goalV[i];

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.15);

    if(benchmark)
    {
        // Benchmark Code - Project 5
        std::string title = "benchmark";
        ompl::tools::Benchmark b(ss, title);
        //b.addPlanner(ompl::base::PlannerPtr(new ompl::control::RRT(ss.getSpaceInformation())));
        ompl::base::PlannerPtr planner(new ompl::control::SMR(ss.getSpaceInformation()));
        ss.setPlanner(planner);
        ss.setup();
        planner->as<ompl::control::SMR>()->setupSMR();
        b.addPlanner(ompl::base::PlannerPtr(planner));

        ompl::tools::Benchmark::Request req;
        req.maxTime = 20.0;
        req.maxMem = 1000.0;
        req.runCount = 100;
        req.displayProgress = true;
        b.benchmark(req);
        std::string logfile = title + ".log";
        b.saveResultsToFile(logfile.c_str());
    }
    else
    {
        // RRT
        //ompl::base::PlannerPtr planner(new ompl::control::RRT(ss.getSpaceInformation()));

        // SMR
        ompl::base::PlannerPtr planner(new ompl::control::SMR(ss.getSpaceInformation()));
        ss.setPlanner(planner);
        ss.setup();
        planner->as<ompl::control::SMR>()->setupSMR();

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = ss.solve(20.0);

        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            ompl::control::PathControl& path = ss.getSolutionPath();
            path.printAsMatrix(std::cout);

            // print path to file
            std::ofstream fout("path.txt");
            fout << env << std::endl;
            path.printAsMatrix(fout);
            fout.close();
        }
        else
            std::cout << "No solution found" << std::endl;
    }
}

void readObstacles(int control, std::vector<Rect>& env, std::vector<double>& start, std::vector<double>& end)
{
    std::string filename = "./environments/env" + std::to_string(control) + ".txt";
    std::ifstream fin(filename);
    std::string line;

    while(std::getline(fin, line) && line != startStr)
    {
        std::istringstream iss(line);
        double v;
        iss >> v; 
        start.push_back(v);
    }

    while(std::getline(fin, line) && line != endStr)
    {
        std::istringstream iss(line);
        double v;
        iss >> v; 
        end.push_back(v);
    }

    while(std::getline(fin, line))
    {
        std::vector<Point2D> polygon;
        while(line != endPolygon)
        {
            std::istringstream iss(line);
            double x, y;
            iss >> x >> y; 
            polygon.push_back(std::make_pair(x, y));
            std::getline(fin, line);
        }
        env.push_back(polygon);
    }
    fin.close();
}

int main(int, char **)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    bool choice;
    std::cout << "Plan for: "<< std::endl;
    std::cout << " (0) Single Run" << std::endl;
    std::cout << " (1) Benchmark SMR Planner" << std::endl;
    std::cin >> choice;

    int env;
    std::cout << "Environment ID: "<< std::endl;
    std::cin >> env;

    std::vector<Rect> obstacle;
    std::vector<double> start;
    std::vector<double> goal;
    readObstacles(env, obstacle, start, goal);

    if(choice)
    {
        plan(obstacle, start, goal, env, true);
    }
    else
    {
        plan(obstacle, start, goal, env);
    }

    return 0;
}
