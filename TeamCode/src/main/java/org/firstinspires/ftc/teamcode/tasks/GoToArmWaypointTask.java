package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmWaypoint;
import org.firstinspires.ftc.teamcode.subsystems.ArmWaypointGraph;

import java.util.Arrays;
import java.util.List;

public class GoToArmWaypointTask extends Task {

    final long MINIMUM_TRANSITION_TIME = 250;
    final long SERVO_ACTION_TRANSITION_TIME = 750;

    Arm arm;
    ArmWaypointGraph.ArmWaypointName target;
    List<ArmWaypoint> armWaypointList;
    int currentIndex;
    ArmWaypoint currentWaypoint;
    ArmWaypoint previousWaypoint;
    Clock clock;

    public GoToArmWaypointTask(Arm arm, ArmWaypointGraph.ArmWaypointName target) {
        this.arm = arm;
        this.target = target;
        this.clock = new Clock();
    }

    @Override
    public void init() {
        currentIndex = 0;
        armWaypointList = ArmWaypointGraph.getInstance().generatePath(arm.getLastWaypoint(), target);
        currentWaypoint = armWaypointList.get(currentIndex);
        previousWaypoint = currentWaypoint;
        clock.startTiming();
        running = true;
    }

    @Override
    public void loop() {
        arm.moveArmToRadians(currentWaypoint.getArmAngle());
        arm.moveRotatorToRadians(currentWaypoint.getRotatorAngle());
        arm.setArmState(Arm.ArmState.TARGET);
        arm.setGripperState(currentWaypoint.getGripperState());
        arm.setWristState(currentWaypoint.getWristState());

        // Detect servo action
        long transitionTime = MINIMUM_TRANSITION_TIME;
        if (currentWaypoint.getWristState() != previousWaypoint.getWristState() ||
                currentWaypoint.getGripperState() != previousWaypoint.getGripperState())
            transitionTime = SERVO_ACTION_TRANSITION_TIME;

        if (clock.getMillis() > transitionTime &&
                arm.isWithinTarget() &&
                currentIndex + 1 < armWaypointList.size()) {
            currentIndex++;
            previousWaypoint = currentWaypoint;
            currentWaypoint = armWaypointList.get(currentIndex);
            clock.startTiming();
        } else if (arm.isWithinTarget() && currentIndex + 1 == armWaypointList.size()) {
            running = false;
        }
        //telemetry.addLine("GoToArmWaypoint State")
        //        .addData("Clock MS", clock.getMillis())
        //        .addData("armInTarget", arm.isWithinTarget())
        //        .addData("currentIndex", currentIndex);
    }

    @Override
    public void stop(boolean interrupted) {
        arm.setLastWaypoint(target);
    }
}