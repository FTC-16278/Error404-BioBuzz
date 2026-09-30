package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/**
 * TeleOp program for an FTC robot with:
 *  - Four-motor Mecanum wheel drivetrain
 *  - One launcher motor using velocity control
 *  - Intake motor and two side servos
 * CONTROLS:
 *  Left stick Y = forward/backward (up = forward, down = backward)
 *  Left stick X = strafe (left = left, right = right)
 *  Right stick X = rotate (left = CCW, right = CW)
 *  Right bumper = launcher to fire shots
 *  Right trigger = intake speed increases as pushing it further
 *  Left trigger = intake speed decreases as pushing it further
 */

@TeleOp (name = "Mecannum BioBuzz TeleOp", group = "Error404")
public class MecanumBioBuzzTeleOp extends OpMode {
    double DRIVE_MAX_FORWARD_SPEED = 1.0;  // throttle strafe speed
    double DRIVE_MAX_ANGULAR_SPEED = 1.0;  // throttle rotate speed

    double LAUNCH_TARGET_VELOCITY = 1250; // 2678 RPM
    double LAUNCH_MIN_VELOCITY    = 1200; // 2571 RPM

    // === Drivetrain motors ===
    private final MecanumDrive mecanumDrive = new MecanumDrive()
            .leftFrontName("left_front_drive")
            .leftBackName("left_back_drive")
            .rightFrontName("right_front_drive")
            .rightBackName("right_back_drive")
            .leftFrontDirection(REVERSE)
            .leftBackDirection(REVERSE)
            .rightFrontDirection(FORWARD)
            .rightBackDirection(FORWARD)
            .useBrakeMode(true);

    // === Intake ===
    private final IntakeMotor intakeMotor = new IntakeMotor()
            .motorName("intake")
            .leftServoName("left_intake_servo")
            .rightServoName("right_intake_servo")
            .motorUseBrakeMode(true)
            .motorDirection(FORWARD)
            .leftServoDirection(FORWARD)
            .rightServoDirection(REVERSE);

    // === Launcher and feeders ===
    private final Launcher launcher = new Launcher()
            .launcherName("launcher")
            .launcherUseBrakeMode(true)
            .launcherDirection(FORWARD)
            .feederName("windmillServo")
            .feederDirection(REVERSE);

    // === Run timer & Misc. ===
    private final ElapsedTime runTime = new ElapsedTime();
    private boolean isAlmostEndGame = false;

    @Override
    public void init() {
        /* === Drivetrain setup === */
        mecanumDrive.build(hardwareMap);

        /* === IMU setup === */
        mecanumDrive.initRevIMU(hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        /* === Intake setup === */
        intakeMotor.build(hardwareMap);

        /* === Launcher setup === */
        launcher.build(hardwareMap);
        launcher.setLauncherVelocity(LAUNCH_TARGET_VELOCITY, LAUNCH_MIN_VELOCITY);
    }

    @Override
    public void start() {
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

        mecanumDrive.runDrive(forward, strafe, rotate);

        // === Launcher ===
        boolean launch = gamepad1.right_bumper;

        launcher.launch(launch);

        // === Intake Motor ===
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (launch) intakePower = intakePower < 0 ? intakePower - 0.5 : intakePower + 0.5;

        intakeMotor.run(intakePower);

        // === Status Output ===
        telemetry.addData("Run Time", runTime.toString());
        telemetry.addData("Front Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerFront(), mecanumDrive.getRightPowerFront());
        telemetry.addData("Back Motor Power", "left (%.2f), right (%.2f)", mecanumDrive.getLeftPowerBack(), mecanumDrive.getRightPowerBack());
        telemetry.addData("Launcher State", launcher.getState());
        telemetry.addData("Launcher Speed", launcher.getLauncherVelocity());
        telemetry.addData("Intake Power", intakeMotor.getIntakeMotorPower());
        telemetry.update();
    }
}
