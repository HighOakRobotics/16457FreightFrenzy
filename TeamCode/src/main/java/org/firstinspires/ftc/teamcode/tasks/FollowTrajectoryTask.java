package org.firstinspires.ftc.teamcode.tasks;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;

import java.util.function.Supplier;

public class FollowTrajectoryTask extends Task {
    private final Mecanum mecanum;
    private final Supplier<Trajectory> trajectory;

    public FollowTrajectoryTask(Mecanum mecanum, Supplier<Trajectory> trajectory) {
        this.mecanum = mecanum;
        this.trajectory = trajectory;
        this.running = true;
    }

    public FollowTrajectoryTask(Mecanum mecanum, Pose2d pose) {
        this(mecanum, () -> mecanum.mecanum()
                .trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
                .lineToLinearHeading(pose)
                .build());
    }

    @Override
    public void init() {
        mecanum.mecanum().followTrajectory(trajectory.get());
    }

    @Override
    public void loop() {
        if (!mecanum.mecanum().isBusy()) {
            running = false;
            return;
        }
        mecanum.mecanum().update();
    }

    @Override
    public void stop(boolean interrupted) {
        mecanum.mecanum().idle();
    }
}