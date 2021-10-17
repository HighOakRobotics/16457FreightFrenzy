package org.firstinspires.ftc.teamcode;

import com.ftc11392.sequoia.SequoiaOpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.tasks.GamepadDriveTask;

public class MecanumDrive extends SequoiaOpMode {
    private MecanumDriveTrain drivetrain;
    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        gamepad1H.sticksButton(0.01).onPress(new GamepadDriveTask(gamepad1, drivetrain));


    }
}
