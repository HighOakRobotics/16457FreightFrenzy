package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.AngularServo;

public class Arm extends Subsystem {
    boolean controlLock;

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
    double WRIST_HORIZONTAL_POSITION = Math.PI / 2 - Math.PI / 24;
    double WRIST_TRACKING_STAGING_POSITION = 2 * Math.PI / 3 + Math.PI / 2;

    double GRIPPER_OPEN_POSITION = Math.PI / 3;
    double GRIPPER_CLOSE_POSITION = 0.0;
    double GRIPPER_INTAKE_POSITION = Math.PI / 3;
    double GRIPPER_ELEMENT_POSITION = Math.PI / 3;

    double armPositioningTargetPower = 1.0;
    double rotatorPositioningTargetPower = 1.0;
    double armVelocityTargetPower = 1.0;
    double rotatorVelocityTargetPower = 1.0;
    public final static double TICKS_PER_RADIAN = 85.5776129005;

    double trackingHeight;

    ArmState armState;

    WristState wristState;
    GripperState gripperState;

    // Untracked value, updated externally
    ArmWaypointGraph.ArmWaypointName lastWaypoint;

    @Override
    public void initialize(HardwareMap hardwareMap) {
        controlLock = false;

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        rotator = hardwareMap.get(DcMotorEx.class, "rotator");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(armPositioningTargetPower);
        rotator.setPower(rotatorPositioningTargetPower);
        arm.setTargetPosition(0);
        rotator.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTargetPosition = 0;
        rotatorTargetPosition = 0;
        armTargetVelocity = 0;
        rotatorTargetVelocity = 0;

        wrist = new AngularServo(
                hardwareMap.get(Servo.class, "wrist"), 3 * Math.PI / 2 - Math.PI / 6);
        wrist.setPosition(0);
        wristState = WristState.UPRIGHT;

        gripper = new AngularServo(
                hardwareMap.get(Servo.class, "gripper"), 3 * Math.PI / 2 - Math.PI / 6);
        gripper.setPosition(0);
        gripperState = GripperState.CLOSE;

        trackingHeight = 0;

        wristTarget = 0;
        gripperTarget = 0;

        armState = ArmState.TARGET_POSITION;

        lastWaypoint = ArmWaypointGraph.ArmWaypointName.INTAKE_DOWN_UPRIGHT;

        arm.setTargetPositionTolerance(10);
        rotator.setTargetPositionTolerance(5);

        arm.setPositionPIDFCoefficients(6.0);
        rotator.setPositionPIDFCoefficients(4.5);
        arm.setVelocityPIDFCoefficients(30, 5, 0, 0);
        rotator.setVelocityPIDFCoefficients(30, 5, 2, 0);
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
                arm.setPower(armPositioningTargetPower);
                rotator.setPower(rotatorPositioningTargetPower);
                arm.setTargetPosition(armTargetPosition);
                rotator.setTargetPosition(rotatorTargetPosition);
                break;
            case TARGET_VELOCITY:
                arm.setPower(armVelocityTargetPower);
                rotator.setPower(rotatorVelocityTargetPower);
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
            case TRACKING_STAGING:
                wrist.setPosition(WRIST_TRACKING_STAGING_POSITION);
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
                .addData("armPosCTET", "%d %d %d %d", arm.getCurrentPosition(), arm.getTargetPosition(), Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()), arm.getTargetPositionTolerance())
                .addData("rotPosCTET", "%d %d %d %d", rotator.getCurrentPosition(), rotator.getTargetPosition(), Math.abs(rotator.getCurrentPosition() - rotator.getTargetPosition()), rotator.getTargetPositionTolerance())
                .addData("armVelAR", "%.2f %.2f", arm.getVelocity(), rotator.getVelocity())
                .addData("states", "arm%s wrist%s gripper%s", armState, wristState, gripperState);
    }

    @Override
    public void stop() {

    }

    public boolean controlLocked() {
        return controlLock;
    }

    public void lockControl() {
        controlLock = true;
    }

    public void unlockControl() {
        controlLock = false;
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

    public void setArmAngle(double radians) {
        armTargetPosition = (int) Math.round(radiansToTicks(radians));

    }

    public void setRotatorAngle(double radians) {
        rotatorTargetPosition = (int) Math.round(radiansToTicks(radians));
    }

    public void setWristTarget(double radians) {
        wristTarget = radians;
    }

    public void setGripperTarget(double radians) {
        gripperTarget = radians;
    }

    public double getArmAngle() {
        return ticksToRadians(arm.getCurrentPosition());
    }

    public double getRotatorAngle() {
        return ticksToRadians(rotator.getCurrentPosition());
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
        UPRIGHT, HORIZONTAL, TARGET, TRACKING_STAGING
    }

    public enum GripperState {
        OPEN, CLOSE, INTAKE, ELEMENT, TARGET
    }
}
