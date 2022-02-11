package org.firstinspires.ftc.teamcode.utils;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;

import java.util.ArrayList;
import java.util.List;

public class ProfileGenerator {
    LinearInterpolator linearInterpolator;

    double maximumAcceleration;
    double maximumVelocity;
    double maximumAccelerationTime;
    double accelerationRegimeDistance;

    public ProfileGenerator(double maximumAcceleration, double maximumVelocity) {
        this.linearInterpolator = new LinearInterpolator();
        this.maximumAcceleration = maximumAcceleration;
        this.maximumVelocity = maximumVelocity;
        // Given the current constraints, compute the maximum acceleration time
        this.maximumAccelerationTime = maximumVelocity / maximumAcceleration;
        // Compute the total distance travelled from (de)acceleration
        this.accelerationRegimeDistance = maximumAccelerationTime * maximumVelocity;
    }

    public Profile generateProfile(double difference) {
        // I have a suspicion that the way I'm computing things may introduce floating-point bugs
        // Oh well, too bad.
        double duration;
        List<Double> xList = new ArrayList<>();
        List<Double> yList = new ArrayList<>();
        xList.add(0.0);
        yList.add(0.0);
        // Generate unbounded triangle time
        double unboundedTriangleTime = Math.sqrt(2 * difference / maximumAcceleration);
        // If the unbounded triangle is smaller than the maximum triangle, we don't need to go
        // through a constant velocity step in the profile.
        if (unboundedTriangleTime <= maximumAccelerationTime) {
            // Add peak
            xList.add(unboundedTriangleTime);
            yList.add(unboundedTriangleTime * maximumAcceleration);
            // Add endpoint
            duration = 2 * unboundedTriangleTime;
            xList.add(duration);
            yList.add(0.0);
        } else {
            // Add acceleration point
            xList.add(maximumAccelerationTime);
            yList.add(maximumVelocity);
            // Add constant velocity regime
            xList.add(maximumAccelerationTime + ((difference - accelerationRegimeDistance) / maximumVelocity));
            yList.add(maximumVelocity);
            // Add endpoint
            duration = 2 * maximumAccelerationTime + ((difference - accelerationRegimeDistance) / maximumVelocity);
            xList.add(duration);
            yList.add(0.0);
        }
        // Generate a function by linearly interpolating between points, and create a profile
        return new Profile(
                linearInterpolator.interpolate(
                        xList.stream().mapToDouble(Double::doubleValue).toArray(),
                        yList.stream().mapToDouble(Double::doubleValue).toArray()
                ), duration
        );
    }

    public Profile generateProfile(double from, double to) {
        return generateProfile(to - from);
    }
}
