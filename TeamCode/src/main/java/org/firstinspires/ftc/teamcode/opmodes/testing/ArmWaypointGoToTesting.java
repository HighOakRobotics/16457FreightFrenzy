package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmWaypointGraph;
import org.firstinspires.ftc.teamcode.tasks.LegacyGoToArmWaypointTask;

@TeleOp
public class ArmWaypointGoToTesting extends SequoiaOpMode {

    Arm arm = new Arm();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        scheduler.schedule(new SequentialTaskBundle(
                new LegacyGoToArmWaypointTask(
                        arm, ArmWaypointGraph.ArmWaypointName.BACK_DOWN_UPRIGHT
                ),
                new WaitTask(1),
                new LegacyGoToArmWaypointTask(
                        arm, ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_READY
                ),
                new WaitTask(1),
                new LegacyGoToArmWaypointTask(
                        arm, ArmWaypointGraph.ArmWaypointName.BACK_DOWN_READY
                )
        ));
    }
}
