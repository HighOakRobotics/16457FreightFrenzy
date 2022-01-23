package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm extends Subsystem {
    DcMotorEx arm, rotator;
    int armTarget, rotatorTarget;
    double armTargetPower = 0.8, rotatorTargetPower = 0.8;
    public final static double TICKS_PER_RADIAN = 85.5776129005;
    ArmState state;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        rotator = hardwareMap.get(DcMotorEx.class, "rotator");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(armTargetPower);
        rotator.setPower(rotatorTargetPower);
        arm.setTargetPosition(0);
        rotator.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTarget = 0;
        rotatorTarget = 0;
        state = ArmState.TARGET;
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {
        switch (state) {
            case IDLE:
                arm.setPower(0);
                rotator.setPower(0);
                break;
            case TARGET:
                arm.setPower(armTargetPower);
                rotator.setPower(rotatorTargetPower);
                arm.setTargetPosition(armTarget);
                rotator.setTargetPosition(rotatorTarget);
                break;
        }

        telemetry.addLine()
                .addData("armRad", ticksToRadians(arm.getCurrentPosition()))
                .addData("rotRad", ticksToRadians(rotator.getCurrentPosition()));
    }

    @Override
    public void stop() {

    }

    public ArmState getState() {
        return state;
    }

    public void setState(ArmState state) {
        this.state = state;
    }

    public double radiansToTicks(double radians) {
        return radians * TICKS_PER_RADIAN;
    }

    public double radiansToTicks(double angle, AngleUnit angleUnit) {
        return radiansToTicks(angleUnit.toRadians(angle));
    }

    public double ticksToRadians(int ticks) {
        return ticks / TICKS_PER_RADIAN;
    }

    public void moveArmToRadians(double radians) {
        armTarget = (int) Math.round(radiansToTicks(radians));

    }

    public void moveRotatorToRadians(double radians) {
        rotatorTarget = (int) Math.round(radiansToTicks(radians));
    }

    public enum ArmState {
        IDLE, TARGET
    }
}
