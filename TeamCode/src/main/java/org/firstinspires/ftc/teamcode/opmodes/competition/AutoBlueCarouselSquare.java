package org.firstinspires.ftc.teamcode.opmodes.competition;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.ParallelTaskBundle;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.SwitchTask;
import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.tasks.ArmTrackingTask;
import org.firstinspires.ftc.teamcode.tasks.AutoSlowDuckProfileTask;
import org.firstinspires.ftc.teamcode.tasks.FollowTrajectoryTask;
import org.firstinspires.ftc.teamcode.tasks.LegacyGoToArmWaypointTask;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

@Autonomous(group = "Working Title")
public class AutoBlueCarouselSquare extends SequoiaOpMode {

    DuckDetector duckDetector = new DuckDetector(70, 160, 250);
    Mecanum mecanum = new Mecanum();
    Arm arm = new Arm();
    Carousel carousel = new Carousel();

    @Override
    public void initTriggers() {
        mecanum.mecanum().setPoseEstimate(new Pose2d(-37, 66, Math.PI));
    }

    @Override
    public void runTriggers() {
        DuckDetector.DuckPipeline.DuckPosition position = duckDetector.getAnalysis();
        scheduler.schedule(new SequentialTaskBundle(
                new FollowTrajectoryTask(mecanum, new Pose2d(-46, 50, Math.PI / 2)),
                new FollowTrajectoryTask(mecanum, new Pose2d(-60, 64, -Math.PI / 4)),
                new AutoSlowDuckProfileTask(carousel, 1),

                new FollowTrajectoryTask(mecanum, new Pose2d(-60, 38, -Math.PI / 2)),
                new LegacyGoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.LEFT_TRACKING),
                new SwitchTask(new HashMap<Object, Task>() {{
                    put(DuckDetector.DuckPipeline.DuckPosition.LEFT, new SequentialTaskBundle(
                            new ArmTrackingTask(arm, 6)
                    ));
                    put(DuckDetector.DuckPipeline.DuckPosition.CENTER, new SequentialTaskBundle(
                            new ArmTrackingTask(arm, 13)
                    ));
                    put(DuckDetector.DuckPipeline.DuckPosition.RIGHT, new SequentialTaskBundle(
                            new ArmTrackingTask(arm, 19)
                    ));
                }}, () -> position),
                new WaitTask(1000, TimeUnit.MILLISECONDS),

                new FollowTrajectoryTask(mecanum, new Pose2d(-22, 30, -Math.PI / 2)),
                new InstantTask(() -> arm.setGripperState(Arm.GripperState.OPEN)),
                new WaitTask(500, TimeUnit.MILLISECONDS),
                new FollowTrajectoryTask(mecanum, new Pose2d(-60, -38, -Math.PI / 2)),
                new LegacyGoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_UPRIGHT),

                // If the first one failed
                new LegacyGoToArmWaypointTask(arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_UPRIGHT),

                new InstantTask(this::requestOpModeStop)
        ));
    }
}
