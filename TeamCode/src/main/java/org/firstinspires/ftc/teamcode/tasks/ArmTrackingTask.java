package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypoint;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;

import java.util.function.DoubleSupplier;

public class ArmTrackingTask extends Task {

    Arm arm;
    DoubleSupplier raiseSupplier;
    DoubleSupplier lowerSupplier;

    ArmWaypoint.ArmWaypointLocation location;

    public ArmTrackingTask(Arm arm, DoubleSupplier raiseSupplier, DoubleSupplier lowerSupplier) {
        this.arm = arm;
        this.raiseSupplier = raiseSupplier;
        this.lowerSupplier = lowerSupplier;
    }

    @Override
    public void init() {
        location = ArmWaypointGraph.getInstance().waypointMap.get(arm.getLastWaypoint()).getLocation();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop(boolean interrupted) { }
}
