package org.firstinspires.ftc.teamcode.tasks;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypoint;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.utils.Profile;
import org.firstinspires.ftc.teamcode.utils.ProfileGenerator;

import java.util.List;

public class GoToArmWaypointTask extends Task {

    final double ARM_VELOCITY_CONSTRAINT = 5;
    final double ARM_ACCELERATION_CONSTRAINT = 1;
    final double ROTATOR_VELOCITY_CONSTRAINT = 5;
    final double ROTATOR_ACCELERATION_CONSTRAINT = 1;

    final double MAXIMUM_ARM_ERROR = Math.PI / 24;
    final double MAXIMUM_ROTATOR_ERROR = Math.PI / 24;

    boolean didNotInitialize;

    Arm arm;
    ArmWaypointGraph.ArmWaypointName target;
    List<ArmWaypoint> armWaypointList;
    int currentIndex;
    ArmWaypoint currentWaypoint;
    ArmWaypoint previousWaypoint;
    ProfileGenerator armProfileGenerator;
    ProfileGenerator rotatorProfileGenerator;
    Profile currentArmProfile;
    Profile currentRotatorProfile;
    Clock clock;

    public GoToArmWaypointTask(Arm arm, ArmWaypointGraph.ArmWaypointName target) {
        this.arm = arm;
        this.target = target;
        this.clock = new Clock();
        this.armProfileGenerator = new ProfileGenerator(ARM_ACCELERATION_CONSTRAINT, ARM_VELOCITY_CONSTRAINT);
        this.rotatorProfileGenerator = new ProfileGenerator(ROTATOR_ACCELERATION_CONSTRAINT, ROTATOR_VELOCITY_CONSTRAINT);
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
        if (armWaypointList.size() <= 1) {
            arm.unlockControl();
            didNotInitialize = true;
            running = false;
            return;
        }
        previousWaypoint = armWaypointList.get(currentIndex);
        currentIndex++;
        currentWaypoint = armWaypointList.get(currentIndex);
        generateProfiles();
        clock.startTiming();
        running = true;
    }

    @Override
    public void loop() {
        if (didNotInitialize) return;

        arm.setArmState(Arm.ArmState.TARGET_VELOCITY);
        arm.setArmVelocity(currentArmProfile.getProfileVelocity(clock.getSeconds()));
        arm.setRotatorVelocity(currentRotatorProfile.getProfileVelocity(clock.getSeconds()));
        arm.setGripperState(currentWaypoint.getGripperState());
        arm.setWristState(currentWaypoint.getWristState());

        if (currentArmProfile.isProfileComplete(clock.getSeconds()) && currentRotatorProfile.isProfileComplete(clock.getSeconds())
                && Math.abs(arm.getArmAngle() - currentWaypoint.getArmAngle()) <= MAXIMUM_ARM_ERROR
                && Math.abs(arm.getRotatorAngle() - currentWaypoint.getRotatorAngle()) <= MAXIMUM_ROTATOR_ERROR
                && currentIndex + 1 < armWaypointList.size()) {
            currentIndex++;
            arm.setLastWaypoint(currentWaypoint.getName());
            previousWaypoint = currentWaypoint;
            currentWaypoint = armWaypointList.get(currentIndex);
            clock.startTiming();
        } else if (currentArmProfile.isProfileComplete(clock.getSeconds()) && currentRotatorProfile.isProfileComplete(clock.getSeconds())
                && (!(Math.abs(arm.getArmAngle() - currentWaypoint.getArmAngle()) <= MAXIMUM_ARM_ERROR)
                || !(Math.abs(arm.getRotatorAngle() - currentWaypoint.getRotatorAngle()) <= MAXIMUM_ROTATOR_ERROR))) {
            arm.setArmAngle(previousWaypoint.getArmAngle());
            arm.setRotatorAngle(previousWaypoint.getRotatorAngle());
            arm.setArmState(Arm.ArmState.TARGET_POSITION);
            arm.setGripperState(previousWaypoint.getGripperState());
            arm.setWristState(previousWaypoint.getWristState());
            arm.setLastWaypoint(previousWaypoint.getName());
            arm.unlockControl();
            currentWaypoint = previousWaypoint;
            generateProfiles();
            running = false;
        } else if (currentArmProfile.isProfileComplete(clock.getSeconds()) && currentRotatorProfile.isProfileComplete(clock.getSeconds())
                && currentIndex + 1 == armWaypointList.size()) {
            running = false;
        }
    }

    @Override
    public void stop(boolean interrupted) {
        if (didNotInitialize) return;
        arm.setArmAngle(currentWaypoint.getArmAngle());
        arm.setRotatorAngle(currentWaypoint.getRotatorAngle());
        arm.setArmState(Arm.ArmState.TARGET_POSITION);
        arm.setGripperState(currentWaypoint.getGripperState());
        arm.setWristState(currentWaypoint.getWristState());
        arm.setLastWaypoint(currentWaypoint.getName());
        arm.unlockControl();
    }

    public void generateProfiles() {
        currentArmProfile = armProfileGenerator.generateProfile(previousWaypoint.getArmAngle(), currentWaypoint.getArmAngle());
        currentRotatorProfile = rotatorProfileGenerator.generateProfile(previousWaypoint.getRotatorAngle(), currentWaypoint.getRotatorAngle());
    }
}