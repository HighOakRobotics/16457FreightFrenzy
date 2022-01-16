package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Subsystem {
    DcMotorEx arm;
    DcMotorEx wrist;
    double setpoint;
    double setpointTarget;
    ArmMode mode;
    // MAGIC
    double[] kArmH = {-0.000430352, 0.0139650, 0.0250669, -3.86955, -40.7864, 4091.66};
    double[] kWristH = {-0.00000174650, 0.00219642, -0.108194, 1.81155, -5.45495, -353.855};
    double kSetpointMinH = 0;
    double kSetpointMaxH = 25.5;
    //more magic
    double[] kArmV = {-0.00734996, 0.274343, -3.80298, 22.5151, -113.945, 3544.10};
    double[] kWristV = {0.00122341, -0.0462197, 0.618546, -3.20674, 10.5636, -138.016};
    double kSetpointMinV = 0;
    double kSetpointMaxV = 15.5;
    //Physical limits
    int kArmMin = -10;
    int kArmMax = 4200;
    int kWristMin = -450;
    int kWristMax = 10;

    double armPower = 1.0;
    double wristPower = 0.4;

    int armHome;
    int wristHome;
    int armIntake = 560;
    int wristIntake = -410;

    public void modifySetpoint(double amount) {/*
        double[] kArm;
        double min, max;
        switch (mode) {
            case VERTICAL:
                kArm = kArmV;
                min = kSetpointMinV;
                max = kSetpointMaxV;
                break;
            case HORIZONTAL:
                kArm = kArmH;
                min = kSetpointMinH;
                max = kSetpointMaxH;
                break;
            default:
                return;
        }
        kArm[kArm.length - 1] = kArm[kArm.length - 1] - arm.getCurrentPosition();
        PolynomialFunction poly = new PolynomialFunction(kArm);
        UnivariateSolver solver = new MullerSolver();
        double currheight = solver.solve(100, poly, min, max, setpoint);
        setpointTarget = currheight + amount;*/
        setpointTarget = setpoint+amount;
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
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(0);
        wrist.setPower(0);
        arm.setTargetPosition(0);
        wrist.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(10, 0, 0, 0));
        wrist.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(20, 0, 0, 0));

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
        arm.setPower(armPower);
        wrist.setPower(wristPower);
    }

    @Override
    public void runPeriodic() {
        if (mode == ArmMode.POWER_OFF) { arm.setPower(0); wrist.setPower(0);}
        else { arm.setPower(armPower); wrist.setPower(wristPower); }
        double armTarget = 0.0;
        double wristTarget = 0.0;
        switch (mode) {
            case HORIZONTAL:
                setpoint = Range.clip(setpoint, kSetpointMinH, kSetpointMaxH);
                armTarget = apply(setpoint, kArmH);
                wristTarget = apply(setpoint, kWristH);
                break;
            case VERTICAL:
                setpoint = Range.clip(setpoint, kSetpointMinV, kSetpointMaxV);
                armTarget = apply(setpoint, kArmV);
                wristTarget = apply(setpoint, kWristV);
                break;
            case HOME:
                armTarget = armHome;
                wristTarget = wristHome;
                break;
            case INTAKE:
                armTarget = armIntake;
                wristTarget = wristIntake;
                break;
        }

        // Rough limit compensation to prevent jamming wrist against the ground
        if (wrist.getCurrentPosition() > -150 && arm.getCurrentPosition() > 3200 && mode == ArmMode.HORIZONTAL)
            armTarget = kArmV[kArmV.length - 1];
        // Rough limit compensation to prevent wrist from crashing into robot from HOME > INTAKE transistion
        if ((wrist.getCurrentPosition() > -400 && arm.getCurrentPosition() < 2000 && mode == ArmMode.INTAKE) ||
                (wrist.getCurrentPosition() < -200 && arm.getCurrentPosition() < 2000 && mode == ArmMode.HOME))
            armTarget = 1200;
        if (wrist.getCurrentPosition() < -200 && mode == ArmMode.HOME && arm.getCurrentPosition() < 1100)
            wristTarget = wrist.getCurrentPosition();
        arm.setTargetPosition(Range.clip((int) Math.round(armTarget), kArmMin, kArmMax));
        wrist.setTargetPosition(Range.clip((int) Math.round(wristTarget), kWristMin, kWristMax));

        setpoint = (setpointTarget - setpoint) / 2 + setpoint;

        telemetry.addLine("[ARM]")
                .addData("Mode", mode)
                .addData("Current", "a:%d w:%d", arm.getCurrentPosition(), wrist.getCurrentPosition())
                .addData("Target", "a:%f.0 w:%f.0", armTarget, wristTarget)
                .addData("Power", "a:%f.0 w:%f.0", arm.getPower(), wrist.getPower());
    }

    @Override
    public void stop() {
        arm.setPower(0);
        wrist.setPower(0);
    }

    public int getArmPosition() {
        return arm.getCurrentPosition();
    }

    public int getWristPosition() {
        return wrist.getCurrentPosition();
    }

    public enum ArmMode {
        HORIZONTAL, VERTICAL, HOME, INTAKE, POWER_OFF
    }
}
