package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.ftc11392.sequoia.subsystem.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    double armTargetPower;
    double rotatorTargetPower;
    double armTargetVelocity;
    double rotatorTargetVelocity;
    double wristTarget;
    double gripperTarget;

    double WRIST_UPRIGHT_POSITION = 0.0;
    double WRIST_HORIZONTAL_POSITION = Math.PI / 2 - Math.PI / 18;
    double WRIST_TRACKING_STAGING_POSITION = 2 * Math.PI / 3 + Math.PI / 2;

    double GRIPPER_OPEN_POSITION = Math.PI / 3;
    double GRIPPER_CLOSE_POSITION = 0.0;
    double GRIPPER_INTAKE_POSITION = Math.PI / 5.5;
    double GRIPPER_ELEMENT_POSITION = Math.PI / 3;

    double armPositioningTargetPower = 1.0;
    double rotatorPositioningTargetPower = 1.0;
    double armVelocityTargetPower = 1.0;
    double rotatorVelocityTargetPower = 1.0;
    public final static double TICKS_PER_RADIAN_ROTATOR = 753.2 / (2 * Math.PI);
    public final static double TICKS_PER_RADIAN_ARM = 226.811709;

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
        armTargetPower = 0;
        rotatorTargetPower = 0;

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
        rotator.setTargetPositionTolerance(10);

        arm.setPositionPIDFCoefficients(6.5);
        rotator.setPositionPIDFCoefficients(4.5);
        arm.setVelocityPIDFCoefficients(10, 5, 2, 0);
        rotator.setVelocityPIDFCoefficients(20, 1, 5, 0);
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
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case TARGET_POSITION:
                arm.setPower(armPositioningTargetPower);
                rotator.setPower(rotatorPositioningTargetPower);
                arm.setTargetPosition(armTargetPosition);
                rotator.setTargetPosition(rotatorTargetPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case TARGET_VELOCITY:
                if (armTargetVelocity > 0) {
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm.setVelocity(armTargetVelocity);
                } else {
                    arm.setTargetPosition(arm.getCurrentPosition());
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (rotatorTargetVelocity > 0) {
                    rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rotator.setVelocity(rotatorTargetVelocity);
                } else {
                    rotator.setTargetPosition(rotator.getCurrentPosition());
                    rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case TARGET_POWER:
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(armTargetPower);
                rotator.setPower(rotatorTargetPower);
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
                .addData("armRad", ticksToRadiansRotator(arm.getCurrentPosition()))
                .addData("rotRad", ticksToRadiansRotator(rotator.getCurrentPosition()))
                .addData("armPosCTET", "%d %d %d %d", arm.getCurrentPosition(), arm.getTargetPosition(), Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()), arm.getTargetPositionTolerance())
                .addData("rotPosCTET", "%d %d %d %d", rotator.getCurrentPosition(), rotator.getTargetPosition(), Math.abs(rotator.getCurrentPosition() - rotator.getTargetPosition()), rotator.getTargetPositionTolerance())
                .addData("armVelAR", "%.2f %.2f", arm.getVelocity(), rotator.getVelocity())
                .addData("states", "arm%s wrist%s gripper%s", armState, wristState, gripperState)
                .addData("lastWaypoint", lastWaypoint);
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


    public double radiansToTicksRotator(double radians) {
        return radians * TICKS_PER_RADIAN_ROTATOR;
    }

    public double radiansToTicksRotator(double angle, AngleUnit angleUnit) {
        return radiansToTicksRotator(angleUnit.toRadians(angle));
    }

    public double ticksToRadiansRotator(int ticks) {
        return ticks / TICKS_PER_RADIAN_ROTATOR;
    }

    public double radiansToTicksArm(double radians) {
        return radians * TICKS_PER_RADIAN_ARM;
    }

    public double radiansToTicksArm(double angle, AngleUnit angleUnit) {
        return radiansToTicksArm(angleUnit.toRadians(angle));
    }

    public double ticksToRadiansArm(int ticks) {
        return ticks / TICKS_PER_RADIAN_ARM;
    }

    public void setArmAngle(double radians) {
        armTargetPosition = (int) Math.round(radiansToTicksArm(radians));

    }

    public void setRotatorAngle(double radians) {
        rotatorTargetPosition = (int) Math.round(radiansToTicksRotator(radians));
    }

    public void setArmTargetPower(double power) {
        armTargetPower = Range.clip(power, -1, 1);
    }

    public void stopAndResetEncoders() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRotatorTargetPower(double power) {
        rotatorTargetPower = Range.clip(power, -1, 1);
    }

    public void setWristTarget(double radians) {
        wristTarget = radians;
    }

    public void setGripperTarget(double radians) {
        gripperTarget = radians;
    }

    public double getArmAngle() {
        return ticksToRadiansArm(arm.getCurrentPosition());
    }

    public double getRotatorAngle() {
        return ticksToRadiansRotator(rotator.getCurrentPosition());
    }

    public void setArmVelocity(double angularRateRadians) {
        armTargetVelocity = radiansToTicksArm(angularRateRadians);
    }

    public void setRotatorVelocity(double angularRateRadians) {
        rotatorTargetVelocity = radiansToTicksRotator(angularRateRadians);
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
        IDLE, TARGET_POSITION, TARGET_VELOCITY, TARGET_POWER
    }

    public enum WristState {
        UPRIGHT, HORIZONTAL, TARGET, TRACKING_STAGING
    }

    public enum GripperState {
        OPEN, CLOSE, INTAKE, ELEMENT, TARGET
    }
}
