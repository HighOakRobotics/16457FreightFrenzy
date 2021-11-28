package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake extends Subsystem{
    DcMotorEx IntakeMotor;
    double velocity;
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setVelocity(0, AngleUnit.DEGREES);
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {
        IntakeMotor.setVelocity(velocity, AngleUnit.DEGREES);
    }

    @Override
    public void stop() {
        IntakeMotor.setVelocity(0);
    }
}
