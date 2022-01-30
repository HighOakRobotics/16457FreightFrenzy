package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.SequentialTaskBundle;
import com.ftc11392.sequoia.task.WaitTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

@TeleOp
public class WristGripperSyncTest extends SequoiaOpMode {

    Arm arm = new Arm();
    Gripper gripper = new Gripper();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        scheduler.schedule(new SequentialTaskBundle(
                new InstantTask(() -> arm.setState(Arm.ArmState.INTAKE)),
                new WaitTask(1),
                new InstantTask(() -> gripper.setState(Gripper.GripperState.INTAKE))
        ));
    }
}
