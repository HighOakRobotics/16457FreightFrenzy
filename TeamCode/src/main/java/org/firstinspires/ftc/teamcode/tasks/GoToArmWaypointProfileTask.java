package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypoint;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.utils.Profile;
import org.firstinspires.ftc.teamcode.utils.ProfileGenerator;

import java.util.List;

public class GoToArmWaypointProfileTask extends Task {

    final long MINIMUM_TRANSITION_TIME = 250;
    final long SERVO_ACTION_TRANSITION_TIME = 750;

    Arm arm;
    ArmWaypointGraph.ArmWaypointName target;
    List<ArmWaypoint> armWaypointPath;
    int currentIndex;
    ArmWaypoint currentWaypoint;
    ArmWaypoint previousWaypoint;
    Clock clock;
    double currentWaypointDuration;
    ProfileGenerator armProfileGenerator;
    ProfileGenerator rotatorProfileGenerator;
    Profile currentArmProfile;
    Profile currentRotatorProfile;
    boolean isCurrentWaypointServoAction;

    public GoToArmWaypointProfileTask(Arm arm, ArmWaypointGraph.ArmWaypointName target) {
        this.arm = arm;
        this.target = target;
        this.clock = new Clock();
        this.armProfileGenerator = new ProfileGenerator(
                ArmConstants.ARM_MAXIMUM_ACCELERATION, ArmConstants.ARM_MAXIMUM_VELOCITY
        );
        this.rotatorProfileGenerator = new ProfileGenerator(
                ArmConstants.ROTATOR_MAXIMUM_ACCELERATION, ArmConstants.ROTATOR_MAXIMUM_VELOCITY
        );
    }

    @Override
    public void init() {
        armWaypointPath = ArmWaypointGraph.getInstance().generatePath(arm.getLastWaypoint(), target);
        currentIndex = 0;
        changeWaypoint();
        running = true;

        arm.setArmState(Arm.ArmState.TARGET_VELOCITY);
    }

    @Override
    public void loop() {
        arm.setArmAngle(currentArmProfile.getProfileVelocity(clock.getSeconds()));
        arm.setRotatorAngle(currentRotatorProfile.getProfileVelocity(clock.getSeconds()));
        arm.setGripperState(currentWaypoint.getGripperState());
        arm.setWristState(currentWaypoint.getWristState());

        if (clock.getSeconds() >= currentWaypointDuration && currentIndex + 1 < armWaypointPath.size()) {
            changeWaypoint();
        }
        if (currentIndex == armWaypointPath.size() - 1)
            running = false;
    }

    @Override
    public void stop(boolean interrupted) {
        arm.setArmState(Arm.ArmState.TARGET_POSITION);
        arm.setArmAngle(currentWaypoint.getArmAngle());
        arm.setRotatorAngle(currentWaypoint.getRotatorAngle());
        arm.setLastWaypoint(target);
    }

    public void changeWaypoint() {
        currentIndex++;
        previousWaypoint = armWaypointPath.get(currentIndex - 1);
        currentWaypoint = armWaypointPath.get(currentIndex);
        currentArmProfile = armProfileGenerator.generateProfile(
                previousWaypoint.getArmAngle(),
                currentWaypoint.getArmAngle()
        );
        currentRotatorProfile = rotatorProfileGenerator.generateProfile(
                previousWaypoint.getRotatorAngle(),
                currentWaypoint.getRotatorAngle()
        );
        currentWaypointDuration = Math.max(currentArmProfile.getDuration(), currentRotatorProfile.getDuration());
        isCurrentWaypointServoAction = currentWaypoint.getWristState() != previousWaypoint.getWristState() ||
                currentWaypoint.getGripperState() != previousWaypoint.getGripperState();
        clock.startTiming();
    }
}
