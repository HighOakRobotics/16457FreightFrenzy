package org.firstinspires.ftc.teamcode.utils;

import org.apache.commons.math3.analysis.function.Constant;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;

import java.text.BreakIterator;
import java.util.ArrayList;
import java.util.Arrays;
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
        System.out.println("maximumAccelerationTime" + maximumAccelerationTime);
        System.out.println("accelerationRegimeDistance" + accelerationRegimeDistance);
    }

    public Profile generateProfile(double rawDifference) {
        if (rawDifference == 0)
            return new Profile(new Constant(0), 0);

        double difference = Math.abs(rawDifference);
        int sign = rawDifference < 0 ? -1 : 1;
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
            yList.add(sign * unboundedTriangleTime * maximumAcceleration);
            // Add endpoint
            duration = 2 * unboundedTriangleTime;
            xList.add(duration);
            yList.add(0.0);
        } else {
            // Add acceleration point
            xList.add(maximumAccelerationTime);
            yList.add(sign * maximumVelocity);
            // Add constant velocity regime
            xList.add(maximumAccelerationTime + ((difference - accelerationRegimeDistance) / maximumVelocity));
            yList.add(sign * maximumVelocity);
            // Add endpoint
            duration = 2 * maximumAccelerationTime + ((difference - accelerationRegimeDistance) / maximumVelocity);
            xList.add(duration);
            yList.add(0.0);
        }

        System.out.println("profileGeneratePointsX:"+ Arrays.toString(xList.toArray()));
        System.out.println("profileGeneratePointsY:"+ Arrays.toString(yList.toArray()));

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
