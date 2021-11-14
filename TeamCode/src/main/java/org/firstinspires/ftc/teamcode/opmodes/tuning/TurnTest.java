package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainMecanum;


/*
 * This is a simple routine to test turning capabilities.
 */

@Config
@Autonomous(group = "tuning")
public class TurnTest extends LinearOpMode {
	public static double ANGLE = 90; // deg

	@Override
	public void runOpMode() throws InterruptedException {
		DriveTrainMecanum drive = new DriveTrainMecanum(hardwareMap);

		waitForStart();

		if (isStopRequested()) return;

		drive.turn(Math.toRadians(ANGLE));

		while (drive.isBusy()) drive.update();
	}
}
