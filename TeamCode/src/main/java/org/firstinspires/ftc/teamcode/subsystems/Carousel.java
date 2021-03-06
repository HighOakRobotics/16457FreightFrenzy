package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Carousel extends Subsystem {
    DcMotorEx carousel;
    static final double ratio = 7.5; // Carousel is 15 in, wheel is 2 in diameter.

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    double setpoint;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setVelocity(0, AngleUnit.DEGREES);

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
        carousel.setVelocity( ratio * setpoint, AngleUnit.DEGREES);
    }

    @Override
    public void stop() {
        carousel.setVelocity(0);
    }
}
