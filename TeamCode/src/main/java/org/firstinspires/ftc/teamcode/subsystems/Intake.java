package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake extends Subsystem {
    DcMotorEx intake;

    public double getSetpoint() {
        return setpoint;
    }

    public void setVelocity(double setpoint) {
        this.setpoint = setpoint;
    }

    double setpoint;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "rotator");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setVelocity(0, AngleUnit.DEGREES);

        setpoint = 0;
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {
        intake.setVelocity( 1800.0, AngleUnit.DEGREES);
    }

    @Override
    public void stop() {
        intake.setVelocity(0);
    }
}
