package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.InstantTask;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;

public class GamepadDriveTask extends InstantTask {


    public GamepadDriveTask(Gamepad gamepad, MecanumDriveTrain drive) {
        super(() -> {
            drive.drive(gamepad);
        });
    }
}
