package org.firstinspires.ftc.teamcode.subsystems;

public class ArmWaypoint {
    public ArmWaypoint(double rotatorAngle, double armAngle, Arm.WristState wristState, Arm.GripperState gripperState) {
        this.rotatorAngle = rotatorAngle;
        this.armAngle = armAngle;
        this.wristState = wristState;
        this.gripperState = gripperState;
    }

    @Override
    public String toString() {
        return "ArmWaypoint{" +
                "rotatorAngle=" + rotatorAngle +
                ", armAngle=" + armAngle +
                ", wristState=" + wristState +
                ", gripperState=" + gripperState +
                '}';
    }

    double rotatorAngle;
    double armAngle;
    Arm.WristState wristState;
    Arm.GripperState gripperState;
}
