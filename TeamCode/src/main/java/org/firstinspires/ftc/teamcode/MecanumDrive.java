package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanum;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.tasks.GamepadDriveTask;

@TeleOp(name = "MecanumDrive", group = "not quackology")
public class MecanumDrive extends SequoiaOpMode {
    private final Mecanum drivetrain = new Mecanum();
    private final Arm arm = new Arm();

    @Override
    public void initTriggers() {
    }

    @Override
    public void runTriggers() {
        gamepad1H.sticksButton(0.01).onPressWithCancel(new GamepadDriveTask(gamepad1, drivetrain));

        gamepad1H.upButton().whilePressed(new InstantTask(() -> {
            arm.setSetpoint(arm.getSetpoint() + 0.1);
        }));

        gamepad1H.downButton().whilePressed(new InstantTask(() -> {
            arm.setSetpoint(arm.getSetpoint() - 0.1);
        }));
    }
}
