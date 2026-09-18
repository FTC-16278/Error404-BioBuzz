package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.DigitalSensor;
import org.firstinspires.ftc.teamcode.mechanisms.IndicatorLight;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * TeleOp program for an FTC robot with:
 *  - Four-motor Mecanum wheel drivetrain
 *  - One launcher motor using velocity control
 *  - Intake motor
 * CONTROLS:
 *  Left stick Y = forward/backward (up = forward, down = backward)
 *  Left stick X = strafe (left = left, right = right)
 *  Right stick X = rotate (left = CCW, right = CW)
 *  Right bumper = launcher to fire shots
 *  Left bumper  = intake on
 *  Left trigger = intake off
 */

@TeleOp (name = "MecanumIntakeTeleOp", group = "Error404")
public class MecanumIntakeTeleOp extends OpMode {
    double DRIVE_MAX_POWER = 1.0;
    double DRIVE_MAX_FORWARD_SPEED = 1.0;  // throttle strafe speed
    double DRIVE_MAX_ANGULAR_SPEED = 1.0;  // throttle rotate speed

    double INTAKE_POWER = 0.75;

    double LAUNCH_TARGET_VELOCITY = 1300;
    double LAUNCH_MIN_VELOCITY    = 1280;
    double FEEDER_RUN_SECONDS = 0.20;

    // === Drivetrain motors ===
    private final MecanumDrive mecanumDrive = new MecanumDrive()
            .leftFrontName("leftDriveFront")
            .leftBackName("leftDriveBack")
            .rightFrontName("rightDriveFront")
            .rightBackName("rightDriveBack")
            .leftFrontDirection(REVERSE)
            .leftBackDirection(REVERSE)
            .rightFrontDirection(FORWARD)
            .rightBackDirection(FORWARD)
            .useBrakeMode(true);

    // === Intake ===
    private final IntakeMotor intakeMotor = new IntakeMotor()
            .motorName("intake")
            .motorUseBrakeMode(true)
            .motorDirection(REVERSE);

    // === Launcher and feeders ===
    private final Launcher launcher = new Launcher()
            .launcherName("launcher")
            .launcherUseBrakeMode(true)
            .launcherDirection(REVERSE)
            .feederName("feeder")
            .feederDirection(FORWARD);

    // === Run timer & Misc. ===
    private final ElapsedTime runTime = new ElapsedTime();
    private enum Alliance { BLUE, RED, NONE }
    private Alliance alliance = Alliance.NONE;
    private boolean isAlmostEndGame = false;
    private boolean isIMURequested = false;
    private double driveHeadingOffset = 0.0;

    @Override
    public void init() {
        /* === Drivetrain setup === */
        mecanumDrive.build(hardwareMap);
        mecanumDrive.setMaxPower(DRIVE_MAX_POWER);

        /* === IMU setup === */
        mecanumDrive.initRevIMU(hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        /* === Intake setup === */
        intakeMotor.build(hardwareMap);
        intakeMotor.setPower(INTAKE_POWER);

        /* === Launcher setup === */
        launcher.build(hardwareMap);
        launcher.setLauncherVelocity(LAUNCH_TARGET_VELOCITY, LAUNCH_MIN_VELOCITY);
        launcher.setFeederRunSec(FEEDER_RUN_SECONDS);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed())
            alliance = Alliance.BLUE;
        else if (gamepad1.circleWasPressed())
            alliance = Alliance.RED;

        if (gamepad1.triangleWasPressed())
            isIMURequested = true;
        else if (gamepad1.crossWasPressed())
            isIMURequested = false;

        if (isIMURequested) {
            if (alliance == Alliance.BLUE)
                driveHeadingOffset = -90.0;
            else if (alliance == Alliance.RED)
                driveHeadingOffset = 90.0;
        } else {
            driveHeadingOffset = 0.0;
        }

        telemetry.addData("Alliance", "Press Square/Circle button to select Blue/Red team\n");
        telemetry.addData("Use RevIMU", "Press Triangle/Cross button to choose Field/Robot Orientation\n");
        telemetry.addLine();
        telemetry.addLine("------------");
        telemetry.addData("Alliance Team", alliance.toString());
        telemetry.addData("Use Orientation", isIMURequested ? "Field centric" : "Robot centric");
        telemetry.addData("Robot Heading Offset", driveHeadingOffset);
        telemetry.update();
    }

    @Override
    public void start() {
        if (!isIMURequested)
            mecanumDrive.resetDriveYaw();
        mecanumDrive.setDriveAngularOffset(driveHeadingOffset);
        mecanumDrive.restartDrives();
        runTime.reset();
    }

    @Override
    public void loop() {
        if (runTime.seconds() > 120.0) {
            intakeMotor.setIntakeOff();
            launcher.setLauncherOff();
            terminateOpModeNow();
        } else if (runTime.seconds() > 113.0 && !isAlmostEndGame) {
            gamepad1.rumbleBlips(2);
            isAlmostEndGame = true;
        }

        // === Drive Control ===
        double forward = -gamepad1.left_stick_y * DRIVE_MAX_FORWARD_SPEED;
        double strafe  = gamepad1.left_stick_x  * DRIVE_MAX_FORWARD_SPEED;
        double rotate  = gamepad1.right_stick_x * DRIVE_MAX_ANGULAR_SPEED;

        if (!isIMURequested)
            mecanumDrive.runDrive(forward, strafe, rotate);
        else
            mecanumDrive.runDriveFieldRelative(forward, strafe, rotate);

        // === Intake Motor ===
        boolean intakeOn    = gamepad1.left_bumper;
        boolean intakeOff   = gamepad1.left_trigger > 0.5;

        intakeMotor.run(intakeOn, intakeOff);

        // === Launcher ===
        boolean launch = gamepad1.right_bumper;

        launcher.launch(launch);

        // === Status Output ===
        telemetry.addData("Alliance Team", alliance.toString());
        telemetry.addData("Use Orientation", isIMURequested ? "Field centric" : "Robot centric");
        telemetry.addData("Run Time", runTime.toString());
        telemetry.addData("Front Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerFront(), mecanumDrive.getRightPowerFront());
        telemetry.addData("Back Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerBack(), mecanumDrive.getRightPowerBack());
        telemetry.addData("Launcher State", launcher.getState());
        telemetry.addData("Launcher Speed", launcher.getLauncherVelocity());
        telemetry.addData("Intake State", intakeMotor.getState());
        telemetry.addData("Intake Direction", intakeMotor.getMotorDirection());
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Robot Heading", mecanumDrive.getDriveHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
