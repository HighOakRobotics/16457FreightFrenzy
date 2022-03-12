package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.StartEndTask;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Mecanum;

public class GamepadDriveTask extends StartEndTask {


    public GamepadDriveTask(Gamepad gamepad, Mecanum drive) {
        super(() -> {
            drive.mecanum().setDriveDST(
                    () -> gamepad.right_bumper ? gamepad.left_stick_y : gamepad.left_stick_y * 0.8,
                    () -> gamepad.right_bumper ? gamepad.left_stick_x : gamepad.left_stick_x * 0.8,
                    () -> gamepad.right_bumper ? gamepad.right_stick_x : gamepad.right_stick_x * 0.7
            );
        }, () -> {
            drive.mecanum().idle();
        });
    }
}
