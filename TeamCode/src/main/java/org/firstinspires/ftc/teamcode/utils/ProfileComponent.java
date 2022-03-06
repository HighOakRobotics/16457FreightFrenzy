package org.firstinspires.ftc.teamcode.utils;

public class ProfileComponent {
    final double position;
    final double velocity;

    public ProfileComponent(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }
}
