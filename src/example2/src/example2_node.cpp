#include "am_traj/am_traj.hpp"

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>

using namespace std;
using namespace ros;
using namespace Eigen;

class Config
{
public:
    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("TrajectoryTopic", trajectoryTopic);
        nh_priv.getParam("WayPointsTopic", wayPointsTopic);
        nh_priv.getParam("RouteTopic", routeTopic);
        nh_priv.getParam("AccTopic", accTopic);
        nh_priv.getParam("FrameName", frameName);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("WeightAcc", weightAcc);
        nh_priv.getParam("WeightJerk", weightJerk);
        nh_priv.getParam("MaxAccRate", maxAccRate);
        nh_priv.getParam("MaxVelRate", maxVelRate);
        nh_priv.getParam("Iterations", iterations);
        nh_priv.getParam("Epsilon", epsilon);
    }

    // Advertised Topics
    string trajectoryTopic;
    string wayPointsTopic;
    string routeTopic;
    string accTopic;

    // Frame Name
    std::string frameName;

    // Params
    double weightT;
    double weightAcc;
    double weightJerk;
    double maxAccRate;
    double maxVelRate;
    int iterations;
    double epsilon;
};

class Visualizer
{
public:
    Visualizer(const Config &conf, ros::NodeHandle &nh) : config(conf)
    {
        trajectoryPub = nh.advertise<visualization_msgs::Marker>(config.trajectoryTopic, 1);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>(config.wayPointsTopic, 1);
        routePub = nh.advertise<visualization_msgs::Marker>(config.routeTopic, 1);
        accPub = nh.advertise<visualization_msgs::MarkerArray>(config.accTopic, 1);
    }

private:
    Config config;
    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher accPub;

public:
    void visualize(const Trajectory &traj, const vector<Vector3d> &route, int id = 0)
    {
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker, accMarker;
        visualization_msgs::MarkerArray accMarkers;

        routeMarker.id = id;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = config.frameName;
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.05;

        wayPointsMarker = routeMarker;
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 1.00;
        wayPointsMarker.color.b = 1.00;
        wayPointsMarker.scale.x = 0.30;
        wayPointsMarker.scale.y = 0.30;
        wayPointsMarker.scale.z = 0.30;

        trajMarker = routeMarker;
        trajMarker.ns = "trajectory";
        trajMarker.scale.x = 0.15;
        if (id == 0)
        {
            trajMarker.color.r = 1.00;
            trajMarker.color.g = 0.00;
            trajMarker.color.b = 0.00;
        }
        else if (id == 1)
        {
            trajMarker.color.r = 0.00;
            trajMarker.color.g = 1.00;
            trajMarker.color.b = 0.00;
        }
        else
        {
            trajMarker.color.r = 0.00;
            trajMarker.color.g = 0.00;
            trajMarker.color.b = 1.00;
        }

        accMarker = routeMarker;
        accMarker.type = visualization_msgs::Marker::ARROW;
        accMarker.header.stamp = ros::Time::now();
        accMarker.ns = "acc";
        if (id == 0)
        {
            accMarker.color.r = 255.0 / 255.0;
            accMarker.color.g = 20.0 / 255.0;
            accMarker.color.b = 147.0 / 255.0;
        }
        else if (id == 1)
        {
            accMarker.color.r = 60.0 / 255.0;
            accMarker.color.g = 179.0 / 255.0;
            accMarker.color.b = 113.0 / 255.0;
        }
        else
        {
            accMarker.color.r = 30.0 / 255.0;
            accMarker.color.g = 144.0 / 255.0;
            accMarker.color.b = 255.0 / 255.0;
        }
        accMarker.scale.x = 0.05;
        accMarker.scale.y = 0.15;
        accMarker.scale.z = 0.30;

        if (route.size() > 0)
        {
            bool first = true;
            Vector3d last;
            for (auto it : route)
            {
                if (first)
                {
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::Point point;

                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);
                last = it;

                wayPointsMarker.points.push_back(point);
            }

            routePub.publish(routeMarker);
        }

        if (route.size() > 0)
        {
            for (auto it : route)
            {
                geometry_msgs::Point point;
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub.publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            double T = 0.01;
            Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            trajectoryPub.publish(trajMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            if (id == 0)
            {
                accMarker.action = visualization_msgs::Marker::DELETEALL;   // 发送一个 DELETEALL 动作的加速度标记，以确保清除上一次发布的所有旧加速度标记
                accMarkers.markers.push_back(accMarker);
                accPub.publish(accMarkers);
                accMarkers.markers.clear();
                accMarker.action = visualization_msgs::Marker::ADD;
            }

            double T = 0.10;
            for (double t = 0; t < traj.getTotalDuration(); t += T)
            {
                accMarker.id += 3;
                accMarker.points.clear();
                geometry_msgs::Point point;
                Vector3d X = traj.getPos(t);    // 加速度起点：当前位置
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                accMarker.points.push_back(point);
                X += traj.getAcc(t); // 加速度终点：位置向量+加速度向量等
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                accMarker.points.push_back(point);
                accMarkers.markers.push_back(accMarker);
            }
            accPub.publish(accMarkers);
        }
    }
};

class RandomRouteGenerator
{
public:
    RandomRouteGenerator(Array3d l, Array3d u)
        : lBound(l), uBound(u), uniformReal(0.0, 1.0) {}

    inline vector<Vector3d> generate(int N)
    {
        vector<Vector3d> route;
        Array3d temp;
        route.emplace_back(0.0, 0.0, 0.0);  // 起点设为 (0,0,0)
        for (int i = 0; i < N; i++)
        {
            temp << uniformReal(gen), uniformReal(gen), uniformReal(gen);
            temp = (uBound - lBound) * temp + lBound;
            route.emplace_back(temp);
        }
        return route;
    }

private:
    Array3d lBound;
    Array3d uBound;
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> uniformReal;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "example0_node");
    ros::NodeHandle nh_, nh_priv("~");

    Config config(nh_priv);
    Visualizer viz(config, nh_);    // 可视化类
    RandomRouteGenerator routeGen(Array3d(-16, -16, -16), Array3d(16, 16, 16));
    AmTraj amTrajOpt(config.weightT, config.weightAcc, config.weightJerk,
                     config.maxVelRate, config.maxAccRate, config.iterations, config.epsilon);

    vector<Vector3d> route;
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Trajectory traj, traj_GD;
    Rate lp(0.25);
    int groupSize = 5;
    std::vector<double> durs;
    durs.clear();
    std::vector<CoefficientMat> coeffMats;

    std::chrono::high_resolution_clock::time_point tc0, tc1;
    double d0, d1, d2;
    for (int i = 5; i < 11 && ok(); i++)    // 段数
    {
        for (int j = 0; j < groupSize && ok(); j++)
        {
            std::cout << "---------------------------------------------------------------------------------------" << std::endl;
            std::cout << "Number of Segments: " << i << ", Group: " << j << std::endl;

            route = routeGen.generate(i);
            
            tc0 = std::chrono::high_resolution_clock::now();
            traj = amTrajOpt.genOptimalTrajDTs3(route, zeroVec, zeroVec, zeroVec, zeroVec);   // 只实现了 s=3
            tc1 = std::chrono::high_resolution_clock::now();
            d1 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count(); 
            viz.visualize(traj, route, 1);
            std::cout << "GREEN: Un-constrained Spatial Optimal Trajectory" << std::endl
                      << "      Planning time:" << d1*1000 << " ms" << std::endl
                      << "      Lap Time: " << traj.getTotalDuration() << " s" << std::endl
                      << "      Cost: " << amTrajOpt.evaluateObjective(traj) << std::endl
                      << "      Maximum Velocity Rate: " << traj.getMaxVelRate() << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << traj.getMaxAccRate() << " m/s^2" << std::endl;
            
            // durs = traj.getDurations();
            // for (int k = 0; k < i; k++)
            // {
            //     std::cout << "            Segment " << k << ": duration: " << durs[k] << " s" << std::endl;
            // }
            // durs.clear();
            // coeffMats = traj.getCoeffMats();
            // for (size_t i = 0; i < coeffMats.size(); ++i) {
            //     std::cout << "coeffMats[" << i << "] =\n"
            //             << coeffMats[i] << std::endl << std::endl;
            // }
            // coeffMats.clear();

            


            mav_trajectory_generation::Vertex::Vector vertices;
            const int dimension = 3;
            // const int derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;
            const int derivative_to_optimize = 2;
            mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

            start.makeStartOrEnd(route[0], derivative_to_optimize);
            vertices.push_back(start);
            for (int k = 1; k < route.size()-1; k++)
            {
                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, route[k]);
                vertices.push_back(middle);
            }
            end.makeStartOrEnd(route[route.size()-1], derivative_to_optimize);
            vertices.push_back(end);

            mav_trajectory_generation::NonlinearOptimizationParameters parameters;
            parameters.max_iterations = 1000;
            parameters.f_rel = 0.05;
            parameters.x_rel = 0.1;
            parameters.time_penalty = config.weightT;
            parameters.use_soft_constraints = false;
            parameters.print_debug_info = false;
            parameters.print_debug_info_time_allocation = false;
            parameters.initial_stepsize_rel = 0.1;
            parameters.inequality_constraint_tolerance = 0.1;
            parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kRichterTime;
            
            std::vector<double> segment_times;
            const double v_max = config.maxVelRate;
            const double a_max = config.maxAccRate;

            tc0 = std::chrono::high_resolution_clock::now();
            segment_times = amTrajOpt.allocateTime(route, 1.0);
            const int N = 6;
            mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
            opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
            // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
            // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
            opt.optimize();
            tc1 = std::chrono::high_resolution_clock::now();
            d2 = std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count(); 


            mav_traj2am_traj(opt, durs, traj_GD);

            viz.visualize(traj_GD, route, 2);
            std::cout << "BLUE: Gradient descent-based Nonlinear Optimization with Trapezoidal Time Allocation" << std::endl
                      << "      Planning time:" << d2*1000 << " ms" << std::endl
                      << "      Lap Time: " << opt.optimization_info_.lap_time << " s" << std::endl
                      << "      Cost: " << opt.optimization_info_.cost_soft_constraints + opt.optimization_info_.cost_time + opt.optimization_info_.cost_trajectory << std::endl
                      << "      Maximum Velocity Rate: " << traj_GD.getMaxVelRate() << " m/s" << std::endl
                      << "      Maximum Acceleration Rate: " << traj_GD.getMaxAccRate() << " m/s^2" << std::endl;

            // for (int k = 0; k < i; k++)
            // {
            //     std::cout << "            Segment " << k << ": duration: " << durs[k] << " s" << std::endl;
            // }
            // durs.clear();
            // coeffMats = traj_GD.getCoeffMats();
            // for (size_t i = 0; i < coeffMats.size(); ++i) {
            //     std::cout << "coeffMats[" << i << "] =\n"
            //             << coeffMats[i] << std::endl << std::endl;
            // }
            // coeffMats.clear();

            spinOnce();
            lp.sleep();
        }

    }

    return 0;
}
