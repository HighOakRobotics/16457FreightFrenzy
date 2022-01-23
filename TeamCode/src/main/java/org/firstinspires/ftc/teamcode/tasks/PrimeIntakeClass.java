package org.firstinspires.ftc.teamcode.tasks;

import com.ftc11392.sequoia.task.Task;
import com.ftc11392.sequoia.util.Clock;

import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class PrimeIntakeClass extends Task {

    Arm2 arm;
    Gripper gripper;
    Intake intake;
    Clock clock;

    public PrimeIntakeClass(Arm2 arm, Gripper gripper, Intake intake) {
        this.arm = arm;
        this.gripper = gripper;
        this.intake = intake;
        this.clock = new Clock();
    }

    @Override
    public void init() {
        running = true;
        arm.setMode(Arm2.ArmMode.INTAKE);
        gripper.setState(Gripper.GripperState.CLOSED);
        clock.startTiming();
    }

    @Override
    public void loop() {
        if (arm.getArmPosition() < 565 && arm.getWristPosition() < -400)
            running = false;
    }

    @Override
    public void stop(boolean interrupted) {
        intake.setSetpoint(-10);
        gripper.setState(Gripper.GripperState.OPEN);
    }
}
