package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.ftc11392.sequoia.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.analysis.solvers.MullerSolver;
import org.apache.commons.math3.analysis.solvers.NewtonRaphsonSolver;
import org.apache.commons.math3.analysis.solvers.PegasusSolver;
import org.apache.commons.math3.analysis.solvers.PolynomialSolver;
import org.apache.commons.math3.analysis.solvers.UnivariateSolver;

public class Arm extends Subsystem {
    DcMotorEx arm;
    DcMotorEx wrist;
    double setpoint;
    double setpointTarget;
    ArmMode mode;
    // MAGIC
    double[] kArmH = {-0.00914442238794, 0.321261633047, -3.0919162105, -62.3691118109, 3883.42569027};
    double[] kWristH = {0.000811713368041, -0.0282206891393, 0.290243892411, 5.84456287387, -278.301815978};
    double kSetpointMinH = 0;
    double kSetpointMaxH = 24;
    //more magic
    double[] kArmV = {-0.00403961044806, 0.0462405085444, -0.4377148918, -60.3767064186, 3378.86017533};
    double[] kWristV = {0.00322737462992, -0.0863502618213, 0.894681801453, 2.32437048804, -94.9712215617};
    double kSetpointMinV = 0;
    double kSetpointMaxV = 12.5;
    //Physical limits
    int kArmMin = 0;
    int kArmMax = 3964;
    int kWristMin = -405;
    int kWristMax = 10;

    int armHome;
    int wristHome;

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
        setpointTarget = currheight + amount;*/ setpointTarget = setpoint+amount;
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
        arm.setPower(1);
        wrist.setPower(0.5);
    }

    @Override
    public void runPeriodic() {
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
        }

        // Rough limit compensation to prevent jamming wrist against the ground
        if (wrist.getCurrentPosition() > -150 && arm.getCurrentPosition() > 3200 && mode == ArmMode.HORIZONTAL)
            armTarget = kArmV[kArmV.length - 1];
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

    public enum ArmMode {
        HORIZONTAL, VERTICAL, HOME
    }
}
