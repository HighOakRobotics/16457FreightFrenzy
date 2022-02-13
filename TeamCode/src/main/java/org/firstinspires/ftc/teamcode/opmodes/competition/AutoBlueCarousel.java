package org.firstinspires.ftc.teamcode.opmodes.competition;


import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.SwitchTask;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.subsystems.legacy.LegacyArm;
import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;
import org.firstinspires.ftc.teamcode.subsystems.legacy.LegacyGripper;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.tasks.ArmTrackingTask;
import org.firstinspires.ftc.teamcode.tasks.DuckProfileTask;
import org.firstinspires.ftc.teamcode.tasks.FollowTrajectoryTask;
import org.firstinspires.ftc.teamcode.tasks.GoToArmWaypointTask;

@Autonomous(group = "Working Title")
public class AutoBlueCarousel extends SequoiaOpMode {

    DuckDetector duckDetector = new DuckDetector(70, 160, 250);
    Mecanum mecanum = new Mecanum();
    Arm arm = new Arm();
    Carousel carousel = new Carousel();

    @Override
    public void initTriggers() {
        mecanum.mecanum().setPoseEstimate(new Pose2d(-40, 66, Math.PI));
    }

    @Override
    public void runTriggers() {
        DuckDetector.DuckPipeline.DuckPosition position = duckDetector.getAnalysis();
        scheduler.schedule(new SequentialTaskBundle(
                new FollowTrajectoryTask(mecanum, new Pose2d(-46, 50, Math.PI / 2)),
                new FollowTrajectoryTask(mecanum, new Pose2d(-60, 64, 0)),
                new DuckProfileTask(carousel, 1),

                new ParallelTaskBundle(
                        new SequentialTaskBundle(
                                new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.RIGHT_TRACKING),
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
                        new FollowTrajectoryTask(mecanum, new Pose2d(-12, 50, 0))
                ),
                new FollowTrajectoryTask(mecanum, new Pose2d(-12, 42, 0)),
                new InstantTask(() -> arm.setGripperState(Arm.GripperState.OPEN)),
                new WaitTask(500, TimeUnit.MILLISECONDS),
                new FollowTrajectoryTask(mecanum, new Pose2d(-12, 50, 0)),
                new GoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_UPRIGHT),

                new FollowTrajectoryTask(mecanum, new Pose2d(0, 66, 0)),
                new FollowTrajectoryTask(mecanum, new Pose2d(50, 66, 0)),
                new FollowTrajectoryTask(mecanum, new Pose2d(50, 40, 0)),

                new InstantTask(this::requestOpModeStop)
        ));
    }
}
