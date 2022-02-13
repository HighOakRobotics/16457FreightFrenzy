package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.Task;

import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.legacy.LegacyGripper;
import org.firstinspires.ftc.teamcode.tasks.ArmTrackingTask;
import org.firstinspires.ftc.teamcode.tasks.FollowTrajectoryTask;
import org.firstinspires.ftc.teamcode.tasks.GoToArmWaypointTask;

import java.util.HashMap;
import java.util.Map;

public class AutoBlueWarehouse extends SequoiaOpMode {
    DuckDetector duckDetector = new DuckDetector(0, 105, 185);
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
                        new FollowTrajectoryTask(mecanum, new Pose2d(12,60,Math.PI/2)
                        ),
                        new FollowTrajectoryTask(mecanum, new Pose2d(12,53,Math.PI/2)))
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
                        new FollowTrajectoryTask(mecanum, new Pose2d(12,60,Math.PI/2)
                        ),
                        new FollowTrajectoryTask(mecanum, new Pose2d(12,51,Math.PI/2)))
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
                        new FollowTrajectoryTask(mecanum, new Pose2d(12,58,Math.PI/2)
                        ),
                        new FollowTrajectoryTask(mecanum, new Pose2d(12,51,Math.PI/2)))
        ));
    }};
    @Override
    public void initTriggers() {
        mecanum.mecanum().setPoseEstimate(new Pose2d(33,63.5, Math.PI));
        //legacyGripper.setState(LegacyGripper.GripperState.CLOSED);
    }

    @Override
    public void runTriggers() {

    }
}
