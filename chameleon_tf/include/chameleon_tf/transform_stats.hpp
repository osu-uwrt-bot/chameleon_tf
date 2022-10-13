#pragma once

#include <vector>
#include <cmath>
#include <geometry_msgs/msg/transform.hpp>

namespace tf_stats
{
    template <typename T>
    struct StatsContainer
    {
        std::vector<T> samples;
        T avg, stddev;
    };

    StatsContainer<double> computeContainer(const std::vector<double> &samples)
    {
        StatsContainer<double> container;
        container.samples = samples;

        // first compute the average
        double total = 0.0;
        for (auto sample : samples)
        {
            total += sample;
        }
        container.avg = total / samples.size();

        // now the stddev
        double sumSq = 0.0;
        for (auto sample : samples)
        {
            sumSq += pow((sample - container.avg), 2);
        }
        double stddev = sqrt(sumSq / samples.size() - 1);

        return container;
    }

    double euclideanDist(const geometry_msgs::msg::Vector3 &input)
    {
        return sqrt(pow(input.x, 2) + pow(input.y, 2) + pow(input.z, 2));
    }

    // While not techincally correct, this should accomplish the goal for now
    double quatDistance(const geometry_msgs::msg::Quaternion & input){
        return sqrt(pow(input.x, 2) + pow(input.y, 2) + pow(input.z, 2) + pow(input.w, 2));
    }

    StatsContainer<geometry_msgs::msg::Transform> getTransformStats(
        const std::vector<geometry_msgs::msg::Transform> & transforms)
    {
        // set up storage
        StatsContainer<geometry_msgs::msg::Transform> container;
        container.samples = transforms;

        // work translations to build 3 double vectors
        // and 4 vectors for quaternions
        std::vector<double> xSamples, ySamples, zSamples,
        quatX, quatY, quatZ, quatW;
        for(auto sample : transforms){
            xSamples.push_back(sample.translation.x);
            ySamples.push_back(sample.translation.y);
            zSamples.push_back(sample.translation.z);
            quatW.push_back(sample.rotation.w);
            quatX.push_back(sample.rotation.x);
            quatY.push_back(sample.rotation.y);
            quatZ.push_back(sample.rotation.z);
        }

        // compute x, y, z stats
        StatsContainer<double> xData, yData, zData;
        xData = computeContainer(xSamples);
        yData = computeContainer(ySamples);
        zData = computeContainer(zSamples);

        // compute quat stats
        StatsContainer<double> wQuatData, xQuatData, yQuatData, zQuatData;
        wQuatData = computeContainer(quatW);
        xQuatData = computeContainer(quatX);
        yQuatData = computeContainer(quatY);
        zQuatData = computeContainer(quatZ);

        // fuse it all back together
        geometry_msgs::msg::Vector3 avgT, stddevT;
        avgT.x = xData.avg;
        avgT.y = yData.avg;
        avgT.z = zData.avg;
        stddevT.x = xData.stddev;
        stddevT.y = yData.stddev;
        stddevT.z = zData.stddev;

        // fuse the rotation
        geometry_msgs::msg::Quaternion avgR, stddevR;
        avgR.w = wQuatData.avg;
        avgR.x = xQuatData.avg;
        avgR.y = yQuatData.avg;
        avgR.z = zQuatData.avg;
        stddevR.w = wQuatData.stddev;
        stddevR.x = xQuatData.stddev;
        stddevR.y = yQuatData.stddev;
        stddevR.z = zQuatData.stddev;

        // pack the container
        container.avg.translation = avgT;
        container.avg.rotation = avgR;
        container.avg.translation = stddevT;
        container.avg.rotation = stddevR;

        return container;
    }

} // namespace tf_stats