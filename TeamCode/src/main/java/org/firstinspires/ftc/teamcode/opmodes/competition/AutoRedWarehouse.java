package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.ParallelDeadlineBundle;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.SwitchTask;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.task.WaitTask;

import org.firstinspires.ftc.teamcode.opmodes.testing.DuckDetectorTestOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.legacy.LegacyArm;
import org.firstinspires.ftc.teamcode.subsystems.legacy.LegacyGripper;
import org.firstinspires.ftc.teamcode.tasks.ArmTrackingTask;
import org.firstinspires.ftc.teamcode.tasks.FollowTrajectoryTask;
import org.firstinspires.ftc.teamcode.tasks.GoToArmWaypointTask;

import java.util.HashMap;
import java.util.Map;

public class AutoRedWarehouse extends SequoiaOpMode {

    DuckDetector duckDetector = new DuckDetector(0,0,0);
    Arm arm = new Arm();
    Mecanum mecanum = new Mecanum();
    Carousel carousel = new Carousel();

    Map<Object, Task> positionMap = new HashMap<Object, Task>(){{
        put(DuckDetector.DuckPipeline.DuckPosition.LEFT, new SequentialTaskBundle(
                new ParallelTaskBundle(
                        new SequentialTaskBundle(
                                new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.LEFT_TRACKING),
                                new InstantTask(() -> {
                                    arm.setWristState(Arm.WristState.HORIZONTAL);
                                    new ArmTrackingTask(arm, 6);
                                })
                        ),
                        new FollowTrajectoryTask(mecanum, new Pose2d(12,-60,Math.PI/2)
                        ),
                new FollowTrajectoryTask(mecanum, new Pose2d(12,-53,Math.PI/2)))
        ));
        put(DuckDetector.DuckPipeline.DuckPosition.CENTER, new SequentialTaskBundle(
                new ParallelTaskBundle(
                        new SequentialTaskBundle(
                                new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.LEFT_TRACKING),
                                new InstantTask(() -> {
                                    arm.setWristState(Arm.WristState.HORIZONTAL);
                                    new ArmTrackingTask(arm, 10);
                                })
                        ),
                        new FollowTrajectoryTask(mecanum, new Pose2d(12,-60,Math.PI/2)
                        ),
                new FollowTrajectoryTask(mecanum, new Pose2d(12,-53,Math.PI/2)))
        ));
        put(DuckDetector.DuckPipeline.DuckPosition.RIGHT, new SequentialTaskBundle(
                new ParallelTaskBundle(
                        new SequentialTaskBundle(
                                new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.LEFT_TRACKING),
                                new InstantTask(() -> {
                                    arm.setWristState(Arm.WristState.HORIZONTAL);
                                    new ArmTrackingTask(arm, 18); // 11 18
                                })
                        ),
                        new FollowTrajectoryTask(mecanum, new Pose2d(12,-58,Math.PI/2)
                ),
                new FollowTrajectoryTask(mecanum, new Pose2d(12,-51,Math.PI/2)))
        ));
    }};

    @Override
    public void initTriggers() {
        mecanum.mecanum().setPoseEstimate(new Pose2d(12, 30));
        arm.setGripperState(Arm.GripperState.CLOSE);
    }

    @Override
    public void runTriggers() {
        DuckDetector.DuckPipeline.DuckPosition position = duckDetector.getAnalysis();
        scheduler.schedule(new SequentialTaskBundle(
                //new WaitTask(8, TimeUnit.SECONDS),
                new SwitchTask(positionMap, () -> position),
                new InstantTask(() -> arm.setGripperState(Arm.GripperState.OPEN)),
                new WaitTask(1),
                new FollowTrajectoryTask(mecanum, () -> mecanum.mecanum()
                        .trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
                        .lineToLinearHeading(mecanum.mecanum().getPoseEstimate()
                                .plus(new Pose2d(0,-5)))
                        .build()),
                new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_UPRIGHT),
                new FollowTrajectoryTask(mecanum, () -> mecanum.mecanum()
                        .trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-66.5,-59.5,-Math.PI/2))
                        .build()),
                new InstantTask(() -> carousel.setSetpoint(-10)),
                new WaitTask(3),
                new InstantTask(() -> carousel.setSetpoint(0)),
                new FollowTrajectoryTask(mecanum, () -> mecanum.mecanum()
                        .trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(0,-72.5,Math.PI))
                        .build()),
                new FollowTrajectoryTask(mecanum, () -> mecanum.mecanum()
                        .trajectoryBuilder(mecanum.mecanum().getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(48,-72.5,Math.PI))
                        .build()),
                new InstantTask(this::requestOpModeStop)
        ));
    }
}
