package org.firstinspires.ftc.teamcode.utils;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class Profile {
    final PolynomialSplineFunction velocityProfile;
    final double duration;

    public Profile(PolynomialSplineFunction velocityProfile, double duration) {
        this.velocityProfile = velocityProfile;
        this.duration = duration;
    }

    public double getProfileVelocity(double timeDifference) {
        if (timeDifference > duration)
            return velocityProfile.value(duration);
        else
            return velocityProfile.value(timeDifference);
    }

    public boolean isProfileComplete(double timeDifference) {
        return timeDifference >= duration;
    }

    public PolynomialSplineFunction getVelocityProfile() {
        return velocityProfile;
    }

    public double getDuration() {
        return duration;
    }
}
