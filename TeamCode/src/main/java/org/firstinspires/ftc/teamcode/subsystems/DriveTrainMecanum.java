package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.subsystems.positioning.Position;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.AxesSigns;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.DashboardUtil;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.subsystems.DriveConstants.kV;

import android.annotation.SuppressLint;
import android.os.Build;

@Config
public class DriveTrainMecanum extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(3, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 100;

    public enum Mode {
        DRIVE_DST,
        DRIVE_ABS,
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;

    private final NanoClock clock;
    private final PIDFController turnController;
    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private final TrajectoryFollower follower;
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;
    private final DcMotorEx rightFront;
    private final List<DcMotorEx> motors;
    private final BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;
    private DoubleSupplier drivePower;
    private DoubleSupplier strafePower;
    private DoubleSupplier turnPower;
    private double currentAngle;
    private double angleRad;
    private double driveAngleOffset;
    private double speed;
    private double angle;
    private double turn;
    private DoubleSupplier xPower;
    private DoubleSupplier yPower;
    private DoubleSupplier headingPower;

    private Mode mode;
    private MotionProfile turnProfile;
    private double turnStart;
    private Pose2d lastPoseOnTurn;

    private LinkedList<Pose2d> poseHistory;

    public DriveTrainMecanum(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new LinkedList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        driveAngleOffset = 0.0;

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontleft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backleft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backright");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontright");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    public Supplier<Position> getPositionSupplier() {
        return Position::new;
    }

    public void setDriveDST() {
        mode = Mode.DRIVE_DST;
    }

    public void setDriveDST(DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier turn) {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drivePower = drive;
        strafePower = strafe;
        turnPower = turn;
        mode = Mode.DRIVE_DST;
    }

    // TODO remove when the linter is less scuffed
    @SuppressLint("NewApi")
    private void setMotorsDST() {
        double d = drivePower.getAsDouble() * DriveConstants.dMultiplier;
        double s = strafePower.getAsDouble() * DriveConstants.sMultiplier;
        double t = turnPower.getAsDouble() * DriveConstants.tMultiplier;

        double v = d - s - t;
        double v1 = d + s - t;
        double v2 = d - s + t;
        double v3 = d + s + t;

        setMotorPowers(v, v1, v2, v3);
    }

    private void setMotorsABS() {
        currentAngle = getPoseEstimate().getHeading();
        currentAngle += Math.PI;
        currentAngle *= -1;
        currentAngle %= 2 * Math.PI;
        if (currentAngle > Math.PI) {
            currentAngle -= 2 * Math.PI;
        }

        double x = Range.clip((yPower.getAsDouble() >= 0 ? 1 : -1) * Math.pow(yPower.getAsDouble(), 2), -1, 1);
        double y = Range.clip((xPower.getAsDouble() >= 0 ? 1 : -1) * Math.pow(xPower.getAsDouble(), 2), -1, 1);
        speed = Range.clip(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)), -1, 1);
        angle = Math.atan2(-x, y);

        angle = angle - (currentAngle - driveAngleOffset);

        this.turn = headingPower.getAsDouble();

        angleRad = angle * (Math.PI / 180);

        double frontLeft = (speed * Math.cos((Math.PI / 4.0) - angle)) - (turn);
        double frontRight = (speed * Math.cos((Math.PI / 4.0) + angle)) + (turn);
        double backLeft = (speed * Math.cos((Math.PI / 4.0) + angle)) - (turn);
        double backRight = (speed * Math.cos((Math.PI / 4.0) - angle)) + (turn);

        //safe drive
        //if (Math.abs(frontLeft) < 0.05) frontLeft = 0.0;
        //if (Math.abs(frontRight) < 0.05) frontRight = 0.0;
        //if (Math.abs(backLeft) < 0.05) backLeft = 0.0;
        //if (Math.abs(backRight) < 0.05) backRight = 0.0;

        setMotorPowers(frontLeft, backLeft, backRight, frontRight);
    }

    public void setDriveABS() {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mode = Mode.DRIVE_ABS;
    }

    public void setDriveABS(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier turn) {
        xPower = xSupplier;
        yPower = ySupplier;
        headingPower = turn;

        mode = Mode.DRIVE_ABS;
    }
/*
	private void setMotorsABS() {
		double frontLeft = (-speed * Math.cos((Math.PI / 4.0) - angle)) + (turn);
		double frontRight = (speed * Math.cos((Math.PI / 4.0) + angle)) + (turn);
		double backLeft = (-speed * Math.cos((Math.PI / 4.0) + angle)) + (turn);
		double backRight = (speed * Math.cos((Math.PI / 4.0) - angle)) + (turn);

		//safe drive
		if (Math.abs(frontLeft) < 0.05) frontLeft = 0.0;
		if (Math.abs(frontRight) < 0.05) frontRight = 0.0;
		if (Math.abs(backLeft) < 0.05) backLeft = 0.0;
		if (Math.abs(backRight) < 0.05) backRight = 0.0;

		setMotorPowers(frontLeft, backLeft, backRight, frontRight);
	}

 */

    public void idle() {
        mode = Mode.IDLE;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

    public void turn(double angle) {
        //setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );

        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void followTrajectory(Trajectory trajectory) {
        //setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
            case DRIVE_DST:
            case DRIVE_ABS:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                setMotorPowers(0, 0, 0, 0);
                break;
            case DRIVE_DST:
                setMotorsDST();
                break;
            case DRIVE_ABS:
                setMotorsABS();
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }


                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        DashboardUtil.drawRobot(fieldOverlay, currentPose);
        dashboard.sendTelemetryPacket(packet);
    }



    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double getRawHeadingVelocity() { return imu.getAngularVelocity().zRotationRate; }
}