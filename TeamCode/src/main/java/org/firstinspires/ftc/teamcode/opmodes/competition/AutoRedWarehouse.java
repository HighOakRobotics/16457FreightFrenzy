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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.testing.DuckDetectorTestOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.legacy.LegacyArm;
import org.firstinspires.ftc.teamcode.subsystems.legacy.LegacyGripper;
import org.firstinspires.ftc.teamcode.tasks.ArmTrackingTask;
import org.firstinspires.ftc.teamcode.tasks.DuckProfileTask;
import org.firstinspires.ftc.teamcode.tasks.FollowTrajectoryTask;
import org.firstinspires.ftc.teamcode.tasks.GoToArmWaypointTask;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AutoRedWarehouse extends SequoiaOpMode {

    DuckDetector duckDetector = new DuckDetector(50,140,230);
    Arm arm = new Arm();
    Mecanum mecanum = new Mecanum();
    Carousel carousel = new Carousel();

    @Override
    public void initTriggers() {
        mecanum.mecanum().setPoseEstimate(new Pose2d(16, -66, 0));
    }

    @Override
    public void runTriggers() {
        DuckDetector.DuckPipeline.DuckPosition position = duckDetector.getAnalysis();
        scheduler.schedule(new SequentialTaskBundle(
                new ParallelTaskBundle(
                        new SequentialTaskBundle(
                                new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.LEFT_TRACKING),
                                new SwitchTask(new HashMap<Object, Task>() {{
                                    put(DuckDetector.DuckPipeline.DuckPosition.LEFT, new SequentialTaskBundle(
                                            new ArmTrackingTask(arm, 5)
                                    ));
                                    put(DuckDetector.DuckPipeline.DuckPosition.CENTER, new SequentialTaskBundle(
                                            new ArmTrackingTask(arm, 12)
                                    ));
                                    put(DuckDetector.DuckPipeline.DuckPosition.RIGHT, new SequentialTaskBundle(
                                            new ArmTrackingTask(arm, 18)
                                    ));
                                }}, () -> position),
                                new WaitTask(500, TimeUnit.MILLISECONDS)
                        ),
                        new FollowTrajectoryTask(mecanum, new Pose2d(-12, -50, 0))
                ),
                new FollowTrajectoryTask(mecanum, new Pose2d(-12, -42, 0)),
                new InstantTask(() -> arm.setGripperState(Arm.GripperState.OPEN)),
                new WaitTask(500, TimeUnit.MILLISECONDS),
                new FollowTrajectoryTask(mecanum, new Pose2d(-12, -50, 0)),
                new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_UPRIGHT),

                new FollowTrajectoryTask(mecanum, new Pose2d(0, -66, 0)),
                new FollowTrajectoryTask(mecanum, new Pose2d(50, -66, 0)),
                new FollowTrajectoryTask(mecanum, new Pose2d(50, -40, 0)),

                new InstantTask(this::requestOpModeStop)
        ));
    }
}
