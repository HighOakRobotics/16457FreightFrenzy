package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypoint;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;

import java.util.List;

public class GoToArmWaypointTask extends Task {

    boolean didNotInitialize;

    final long MINIMUM_TRANSITION_TIME = 500;
    final long MAXIMUM_TRANSITION_TIME = 4000;
    final long SERVO_ACTION_TRANSITION_TIME = 1000;

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
        didNotInitialize = false;
        if (arm.controlLocked()) {
            didNotInitialize = true;
            running = false;
            return;
        }
        arm.lockControl();
        currentIndex = 0;
        armWaypointList = ArmWaypointGraph.getInstance().generatePath(arm.getLastWaypoint(), target);
        currentWaypoint = armWaypointList.get(currentIndex);
        previousWaypoint = currentWaypoint;
        clock.startTiming();
        running = true;
    }

    @Override
    public void loop() {
        if (didNotInitialize) return;
        if (clock.getMillis() > MAXIMUM_TRANSITION_TIME) {
            arm.setArmAngle(previousWaypoint.getArmAngle());
            arm.setRotatorAngle(previousWaypoint.getRotatorAngle());
            arm.setArmState(Arm.ArmState.TARGET_POSITION);
            arm.setGripperState(previousWaypoint.getGripperState());
            arm.setWristState(previousWaypoint.getWristState());
            arm.setLastWaypoint(previousWaypoint.getName());
            arm.unlockControl();
            running = false;
            return;
        }

        arm.setArmAngle(currentWaypoint.getArmAngle());
        arm.setRotatorAngle(currentWaypoint.getRotatorAngle());
        arm.setArmState(Arm.ArmState.TARGET_POSITION);
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
            arm.setLastWaypoint(currentWaypoint.getName());
            previousWaypoint = currentWaypoint;
            currentWaypoint = armWaypointList.get(currentIndex);
            clock.startTiming();
        } else if (arm.isWithinTarget() && currentIndex + 1 == armWaypointList.size()) {
            running = false;
        }
    }

    @Override
    public void stop(boolean interrupted) {
        if (didNotInitialize) return;
        arm.unlockControl();
    }
}