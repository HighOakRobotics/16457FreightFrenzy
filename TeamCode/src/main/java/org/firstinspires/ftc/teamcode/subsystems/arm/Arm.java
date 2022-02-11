package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.AngularServo;

public class Arm extends Subsystem {
    DcMotorEx arm;
    DcMotorEx rotator;
    AngularServo wrist;
    AngularServo gripper;
    int armTargetPosition;
    int rotatorTargetPosition;
    double armTargetVelocity;
    double rotatorTargetVelocity;
    double wristTarget;
    double gripperTarget;

    double WRIST_UPRIGHT_POSITION = 0.0;
    double WRIST_HORIZONTAL_POSITION = Math.PI / 4;

    double GRIPPER_OPEN_POSITION = 0.3;
    double GRIPPER_CLOSE_POSITION = 0.0;
    double GRIPPER_INTAKE_POSITION = 0.6;
    double GRIPPER_ELEMENT_POSITION = 0.25;

    double armTargetPower = 0.8;
    double rotatorTargetPower = 0.8;
    public final static double TICKS_PER_RADIAN = 85.5776129005;

    ArmState armState;

    WristState wristState;
    GripperState gripperState;

    // Untracked value, updated externally
    ArmWaypointGraph.ArmWaypointName lastWaypoint;

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
        armTargetPosition = 0;
        rotatorTargetPosition = 0;
        armTargetVelocity = 0;
        rotatorTargetVelocity = 0;

        wrist = new AngularServo(
                hardwareMap.get(Servo.class, "wrist"), 3 * Math.PI / 4
        );
        wrist.setPosition(0);
        wristState = WristState.UPRIGHT;

        gripper = new AngularServo(
                hardwareMap.get(Servo.class, "gripper"), 3 * Math.PI / 4);
        gripper.setPosition(0);
        gripperState = GripperState.CLOSE;

        wristTarget = 0;
        gripperTarget = 0;

        armState = ArmState.TARGET_POSITION;

        lastWaypoint = ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_UPRIGHT;

        arm.setTargetPositionTolerance(10);
        rotator.setTargetPositionTolerance(10);

        arm.setPositionPIDFCoefficients(7.5);
        rotator.setPositionPIDFCoefficients(2);
        arm.setVelocityPIDFCoefficients(10, 3, 0, 0);
        rotator.setVelocityPIDFCoefficients(10, 3, 0, 0);
    }

    @Override
    public void initPeriodic() {

    }

    @Override
    public void start() {

    }

    @Override
    public void runPeriodic() {
        // Arm and rotator state handling
        switch (armState) {
            case IDLE:
                arm.setPower(0);
                rotator.setPower(0);
                break;
            case TARGET_POSITION:
                arm.setPower(armTargetPower);
                rotator.setPower(rotatorTargetPower);
                arm.setTargetPosition(armTargetPosition);
                rotator.setTargetPosition(rotatorTargetPosition);
                break;
            case TARGET_VELOCITY:
                arm.setPower(armTargetPower);
                rotator.setPower(rotatorTargetPower);
                arm.setVelocity(armTargetVelocity);
                rotator.setVelocity(rotatorTargetVelocity);
                break;
        }
        // Wrist state handling
        switch (wristState) {
            case UPRIGHT:
                wrist.setPosition(WRIST_UPRIGHT_POSITION);
                break;
            case HORIZONTAL:
                wrist.setPosition(WRIST_HORIZONTAL_POSITION);
                break;
            case TARGET:
                wrist.setPosition(wristTarget);
                break;
        }
        // Gripper state handling
        switch (gripperState) {
            case OPEN:
                gripper.setPosition(GRIPPER_OPEN_POSITION);
                break;
            case CLOSE:
                gripper.setPosition(GRIPPER_CLOSE_POSITION);
                break;
            case INTAKE:
                gripper.setPosition(GRIPPER_INTAKE_POSITION);
                break;
            case ELEMENT:
                gripper.setPosition(GRIPPER_ELEMENT_POSITION);
                break;
            case TARGET:
                gripper.setPosition(gripperTarget);
                break;
        }

        telemetry.addLine()
                .addData("armRad", ticksToRadians(arm.getCurrentPosition()))
                .addData("rotRad", ticksToRadians(rotator.getCurrentPosition()))
                .addData("armCTET", "%d %d %d %d", arm.getCurrentPosition(), arm.getTargetPosition(), Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()), arm.getTargetPositionTolerance())
                .addData("rotCTET", "%d %d %d %d", rotator.getCurrentPosition(), rotator.getTargetPosition(), Math.abs(rotator.getCurrentPosition() - rotator.getTargetPosition()), rotator.getTargetPositionTolerance());
    }

    @Override
    public void stop() {

    }

    public ArmState getArmState() {
        return armState;
    }

    public void setArmState(ArmState armState) {
        this.armState = armState;
    }

    public WristState getWristState() {
        return wristState;
    }

    public void setWristState(WristState wristState) {
        this.wristState = wristState;
    }

    public GripperState getGripperState() {
        return gripperState;
    }

    public void setGripperState(GripperState gripperState) {
        this.gripperState = gripperState;
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

    public void setArmPosition(double radians) {
        armTargetPosition = (int) Math.round(radiansToTicks(radians));

    }
    public void setRotatorPosition(double radians) {
        rotatorTargetPosition = (int) Math.round(radiansToTicks(radians));
    }
    public void setArmVelocity(double angularRateRadians) {
        armTargetVelocity = radiansToTicks(angularRateRadians);
    }

    public void setRotatorVelocity(double angularRateRadians) {
        rotatorTargetVelocity = radiansToTicks(angularRateRadians);
    }

    public boolean isWithinTarget() {
        return (
                (Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()) <= arm.getTargetPositionTolerance()) &&
                        (Math.abs(rotator.getCurrentPosition() - rotator.getTargetPosition()) < rotator.getTargetPositionTolerance())
        );
    }

    public ArmWaypointGraph.ArmWaypointName getLastWaypoint() {
        return lastWaypoint;
    }

    public void setLastWaypoint(ArmWaypointGraph.ArmWaypointName lastWaypoint) {
        this.lastWaypoint = lastWaypoint;
    }

    public enum ArmState {
        IDLE, TARGET_POSITION, TARGET_VELOCITY
    }

    public enum WristState {
        UPRIGHT, HORIZONTAL, TARGET
    }

    public enum GripperState {
        OPEN, CLOSE, INTAKE, ELEMENT, TARGET
    }
}
