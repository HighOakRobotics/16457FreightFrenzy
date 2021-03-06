package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Task;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypoint;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;

import java.util.function.DoubleSupplier;

// For more on how this Task works, please reference https://www.desmos.com/calculator/qqvap26kgk
public class ArmTrackingTask extends Task {

    private static final double ARM_LENGTH = 11.3385827;
    private static final double ARM_HEIGHT = 0.5;
    private static final double MIN_DESIRED_HEIGHT = 4.5;
    private static final double MAX_DESIRED_HEIGHT = 20;
    private static final double WRIST_LOCK_HEIGHT = 17;

    private static final double MAX_STOP_HEIGHT_ERROR = 0.5;

    private static final double INPUT_MULTIPLIER = 0.5;

    boolean didNotInitialize;

    Arm arm;
    DoubleSupplier raiseSupplier;
    DoubleSupplier lowerSupplier;

    ArmWaypoint.ArmWaypointLocation location;
    ArmWaypointGraph.ArmWaypointName trackingWaypointName;

    double currentHeight;
    Double heightOverride;
    boolean stopOnTarget;

    public ArmTrackingTask(Arm arm, DoubleSupplier raiseSupplier, DoubleSupplier lowerSupplier) {
        this.arm = arm;
        this.raiseSupplier = raiseSupplier;
        this.lowerSupplier = lowerSupplier;
        this.stopOnTarget = false;
    }

    public ArmTrackingTask(Arm arm, double heightOverride) {
        this(arm, () -> 0.0, () -> 0.0);
        this.heightOverride = heightOverride;
        this.stopOnTarget = true;
    }

    private double computeArmAngle(double desiredHeight) {
        double height = Range.clip(desiredHeight, MIN_DESIRED_HEIGHT, MAX_DESIRED_HEIGHT);
        return Math.acos((ARM_LENGTH - (height - ARM_HEIGHT)) / ARM_LENGTH);
    }

    private double computeArmHeight(double currentAngle) {
        double height = -1 * (Math.cos(currentAngle) * ARM_LENGTH) + ARM_LENGTH + ARM_HEIGHT;
        return Range.clip(height, MIN_DESIRED_HEIGHT, MAX_DESIRED_HEIGHT);
    }

    @Override
    public void init() {
        location = ArmWaypointGraph.getInstance().waypointMap.get(arm.getLastWaypoint()).getLocation();
        didNotInitialize = false;
        if (arm.controlLocked() || location == ArmWaypoint.ArmWaypointLocation.FRONT) {
            didNotInitialize = true;
            running = false;
            return;
        }
        arm.lockControl();
        trackingWaypointName = ArmWaypointGraph.getInstance().trackingWaypoints.get(location);
        if (trackingWaypointName != null) {
            running = true;
            arm.setLastWaypoint(trackingWaypointName);
        } else running = false;
        currentHeight = computeArmHeight(arm.getArmAngle());
        arm.setArmState(Arm.ArmState.TARGET_POSITION);
        arm.setWristState(Arm.WristState.TARGET);
    }

    @Override
    public void loop() {
        if (didNotInitialize) return;
        if (!running) return;
        double input = INPUT_MULTIPLIER * (raiseSupplier.getAsDouble() - lowerSupplier.getAsDouble());
        if (raiseSupplier.getAsDouble() + lowerSupplier.getAsDouble() < 0.01 && !stopOnTarget) running = false;
        if (stopOnTarget && Math.abs(computeArmHeight(arm.getArmAngle()) - currentHeight) < MAX_STOP_HEIGHT_ERROR) running = false;
        currentHeight = Range.clip(currentHeight + input, MIN_DESIRED_HEIGHT, MAX_DESIRED_HEIGHT);
        if (heightOverride != null) currentHeight = heightOverride;
        double armAngle = computeArmAngle(currentHeight);
        double wristAngle = armAngle + Math.PI / 2;

        if (currentHeight >= WRIST_LOCK_HEIGHT) wristAngle = Math.PI;

        arm.setArmAngle(armAngle);
        arm.setWristTarget(wristAngle);
    }

    @Override
    public void stop(boolean interrupted) {
        if (didNotInitialize) return;
        arm.unlockControl();
    }
}
