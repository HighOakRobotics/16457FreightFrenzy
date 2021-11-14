package org.firstinspires.ftc.teamcode.opmodes.competition;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;
import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Rotator;
import org.firstinspires.ftc.teamcode.tasks.GamepadDriveTask;

import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "TeleOp", group = "Working Title")
public class TeleOperated extends SequoiaOpMode {
    private final Mecanum drivetrain = new Mecanum();
    private final Arm arm = new Arm();
    private final Rotator rotator = new Rotator();
    private final Gripper gripper = new Gripper();

    @Override
    public void initTriggers() {
    }

    @Override
    public void runTriggers() {
        gamepad1H.sticksButton(0.01).onPressWithCancel(new GamepadDriveTask(gamepad1, drivetrain));

        gamepad1H.leftButton().onPress(new InstantTask(() -> arm.setMode(Arm.ArmMode.HORIZONTAL)));
        gamepad1H.rightButton().onPress(new InstantTask(() -> arm.setMode(Arm.ArmMode.VERTICAL)));
        gamepad1H.leftBumperButton().onPress(new InstantTask(() -> {
            arm.setMode(Arm.ArmMode.HOME);
            gripper.setState(Gripper.GripperState.CLOSED);
        }));

        gamepad1H.leftTriggerButton(0.01).whilePressed(new InstantTask(() -> arm.modifySetpoint(-0.75 * gamepad1.left_trigger)));
        gamepad1H.rightTriggerButton(0.01).whilePressed(new InstantTask(() -> arm.modifySetpoint(0.75 * gamepad1.right_trigger)));

        AtomicInteger rotationdir = new AtomicInteger(1);
        gamepad1H.aToggleButton().risingWithCancel(new StartEndTask(() -> {
            rotator.setSetpoint(10 * rotationdir.get());
        }, () -> {
            rotator.setSetpoint(0);
            rotationdir.updateAndGet(v -> v * -1);
        }));

        gamepad1H.bToggleButton().risingWithCancel(new StartEndTask(() -> gripper.setState(Gripper.GripperState.OPEN), () -> gripper.setState(Gripper.GripperState.CLOSED)));
    }
}
