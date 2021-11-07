package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.ftc11392.sequoia.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Subsystem {

    PIDFController armController;
    PIDFController wristController;
    DcMotorEx arm;
    DcMotorEx wrist;
    double setpoint;
    ArmMode mode;
    // MAGIC
    double[] kArmH = {-0.00914442238794, 0.321261633047, -3.0919162105, -62.3691118109, 3883.42569027};
    double[] kWristH = {0.000811713368041, -0.0282206891393, 0.290243892411, 5.84456287387, -278.301815978};
    //more magic
    double[] kArmV={-0.00403961044806,0.0462405085444,-0.4377148918,-60.3767064186,3378.86017533};
    double[] kWristV={0.00322737462992,-0.0863502618213,0.894681801453,2.32437048804,-94.9712215617};
    int kArmMin = -100000;
    int kArmMax = 100000;
    int kWristMin = -100000;
    int kWristMax = 100000;

    int armHome;
    int wristHome;

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public ArmMode getMode() {
        return mode;
    }

    public void setMode(ArmMode mode) {
        this.mode = mode;
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        wrist = hardwareMap.get(DcMotorEx.class, "wrist");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(0);
        wrist.setPower(0);
        arm.setTargetPosition(0);
        wrist.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armHome = arm.getCurrentPosition();
        wristHome = wrist.getCurrentPosition();

        setpoint = 0;

        mode = ArmMode.HOME;
    }

    public double apply(double v, double[] k) {
        double result = 0;
        for (int i = 0; i < k.length; i++)
            result += Math.pow(v, k.length - 1 - i) * k[i];
        return result;
    }

    @Override
    public void initPeriodic() {
    }

    @Override
    public void start() {
        arm.setPower(1);
        wrist.setPower(0.5);
    }

    @Override
    public void runPeriodic() {
        telemetry.addData("armMode", mode);
        telemetry.addData("armC", arm.getCurrentPosition());
        telemetry.addData("wristC", wrist.getCurrentPosition());
        telemetry.addData("armT", arm.getTargetPosition());
        telemetry.addData("wristT", wrist.getTargetPosition());

        switch (mode) {
            case HORIZONTAL:
                arm.setTargetPosition(Range.clip((int) Math.round(apply(setpoint, kArmH)), kArmMin, kArmMax));
                wrist.setTargetPosition(Range.clip((int) Math.round(apply(setpoint, kWristH)), kWristMin, kWristMax));
                break;
            case VERTICAL:
                arm.setTargetPosition(Range.clip((int) Math.round(apply(setpoint, kArmV)), kArmMin, kArmMax));
                wrist.setTargetPosition(Range.clip((int) Math.round(apply(setpoint, kWristV)), kWristMin, kWristMax));
                break;
            case HOME:
                arm.setTargetPosition(Range.clip(armHome, kArmMin, kArmMax));
                wrist.setTargetPosition(Range.clip(wristHome, kWristMin, kWristMax));
                break;
        }
    }

    @Override
    public void stop() {
        arm.setPower(0);
        wrist.setPower(0);
    }

    public enum ArmMode {
        HORIZONTAL, VERTICAL, HOME
    }
}
