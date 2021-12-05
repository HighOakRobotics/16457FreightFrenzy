package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.ftc11392.sequoia.SequoiaOpMode;
import com.ftc11392.sequoia.task.InstantTask;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.apache.commons.math3.util.Pair;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

class ArmRegressionTuner extends SequoiaOpMode {

    double heightInput = 0.0; // Inches

    List<Pair<Double, Integer>> armData = new LinkedList<>();
    List<Pair<Double, Integer>> wristData = new LinkedList<>();

    Arm arm = new Arm();

    @Override
    public void initTriggers() {

    }

    @Override
    public void runTriggers() {
        arm.setMode(Arm.ArmMode.POWER_OFF);
        telemetry.log().add("A");

        gamepad1H.upButton().onPress(new InstantTask(() -> {
            heightInput += 0.1;
            telemetry.log().add("height:" + heightInput);
        }));
        gamepad1H.downButton().onPress(new InstantTask(() -> {
            heightInput -= 0.1;
            telemetry.log().add("height:" + heightInput);
        }));
        gamepad1H.aButton().onPress(new InstantTask(() -> {
            armData.add(new Pair<>(heightInput, arm.getArmPosition()));
            wristData.add(new Pair<>(heightInput, arm.getWristPosition()));
            telemetry.log().add("Datum appended, data length now: " + armData.size());
        }));
        gamepad1H.xButton().onPress(new InstantTask(() -> {
            telemetry.log().add("Arm: " + Arrays.toString(computeRegression(armData, 5)));
            telemetry.log().add("Wrist: " + Arrays.toString(computeRegression(wristData, 5)));
        }));
    }

    public double[] computeRegression(List<Pair<Double, Integer>> data, int degree) {
        final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(degree);
        final WeightedObservedPoints obs = new WeightedObservedPoints();
        for (Pair<Double, Integer> datum : data) {
            obs.add(datum.getFirst(), datum.getSecond());
        }
        return fitter.fit(obs.toList());
    }
}
