package org.firstinspires.ftc.teamcode.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ArmWaypoint {
    public ArmWaypointType getType() {
        return type;
    }

    public ArmWaypointLocation getLocation() {
        return location;
    }

    public double getRotatorAngle() {
        return rotatorAngle;
    }

    public double getArmAngle() {
        return armAngle;
    }

    public Arm.WristState getWristState() {
        return wristState;
    }

    public Arm.GripperState getGripperState() {
        return gripperState;
    }

    public ArmWaypoint(ArmWaypointType type, ArmWaypointLocation location, double rotatorAngle, double armAngle, Arm.WristState wristState, Arm.GripperState gripperState) {
        this.type = type;
        this.location = location;
        this.neighbors = new ArrayList<>();
        this.rotatorAngle = rotatorAngle;
        this.armAngle = armAngle;
        this.wristState = wristState;
        this.gripperState = gripperState;
    }

    public void addNeighbor(ArmWaypoint neighbor) {
        neighbors.add(neighbor);
        neighbor.neighbors.add(this);
    }

    public List<ArmWaypoint> getNeighbors() {
        return neighbors;
    }

    @Override
    public String toString() {
        return "ArmWaypoint{" +
                "type=" + type +
                ", location=" + location +
                ", neighbors=" + neighbors.size() +
                ", rotatorAngle=" + rotatorAngle +
                ", armAngle=" + armAngle +
                ", wristState=" + wristState +
                ", gripperState=" + gripperState +
                "}";
    }

    ArmWaypointType type;
    ArmWaypointLocation location;
    List<ArmWaypoint> neighbors;
    double rotatorAngle;
    double armAngle;
    Arm.WristState wristState;
    Arm.GripperState gripperState;

    public enum ArmWaypointType {
        FIXED, TRACKING
    }

    public enum ArmWaypointLocation {
        RIGHT, FRONT, LEFT, BACK
    }
}
