/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot with Mecanum
 * Wheels for the 2026-2027 FIRST® Tech Challenge. On top of a mecanum wheel drivetrain, it uses
 * two motors driving an intake roller set and dual high-speed launcher motors (one for pollen,
 * one for nectar).
 */

@TeleOp(name = "DualIntakeDualLaunch", group = "Iterative OpMode")
//@Disabled
public class DualIntakeDualLaunch extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx pollenLauncher = null;
    private DcMotorEx nectarLauncher = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private CRServo windmillServo = null;

    // Intake state variables: 0 = OFF, 1 = FORWARD (Right Bumper), -1 = REVERSE (Left Bumper)
    private int intakeState = 0;
    private boolean rightBumperPreviousState = false;
    private boolean leftBumperPreviousState = false;

    // Toggle state variables for Pollen Launcher (Left Trigger)
    private boolean pollenToggleState = false;
    private boolean leftTriggerPreviousState = false;

    // Toggle state variables for Nectar Launcher (Right Trigger)
    private boolean nectarToggleState = false;
    private boolean rightTriggerPreviousState = false;

    // Threshold for trigger press detection
    private final double TRIGGER_THRESHOLD = 0.5;


    /*
     * Separate speed constants for each launcher (pollen for small balls, nectar for large balls).
     */
    public final int POLLEN_LAUNCHER_TARGET_VELOCITY = 1250; // 2678 RPM
    public final int POLLEN_LAUNCHER_MIN_VELOCITY = 1200;    // 2571 RPM

    public final int NECTAR_LAUNCHER_TARGET_VELOCITY = 1250; // 2678 RPM
    public final int NECTAR_LAUNCHER_MIN_VELOCITY = 1200;    // 2571 RPM


    /*
     * Drivetrain and intake power variables.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Power variable for intake
    double intakePower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize hardware variables.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "pollen_launcher");
        nectarLauncher = hardwareMap.get(DcMotorEx.class, "nectar_launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        /*
         * Set motor directions.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Intake motor directions set to pull objects inward when driven with positive power.
         */
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Launcher motor directions both set to REVERSE.
         */
        pollenLauncher.setDirection(DcMotor.Direction.FORWARD);
        nectarLauncher.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Set zero power behavior to BRAKE for all motors so drive units and launchers decelerate rapidly.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftIntake.setZeroPowerBehavior(BRAKE);
        rightIntake.setZeroPowerBehavior(BRAKE);
        pollenLauncher.setZeroPowerBehavior(BRAKE);
        nectarLauncher.setZeroPowerBehavior(BRAKE);

        /*
         * Configure both launcher motors for closed-loop velocity control.
         */
        pollenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        nectarLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * Set feeder servo initial state.
         */
        windmillServo.setPower(0);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell driver initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        /*
         * Full Mecanum Drivetrain Control:
         * - forward:  -gamepad1.left_stick_y  (pushing up moves forward, pulling down moves backward)
         * - strafe:   gamepad1.left_stick_x   (pushing right strafes right, pushing left strafes left)
         * - rotate:   gamepad1.right_stick_x  (pushing right turns right, pushing left turns left)
         */
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Process intake controls via Right Bumper (Forward) and Left Bumper (Reverse)
        boolean rightBumperCurrentState = gamepad1.right_bumper;
        boolean leftBumperCurrentState = gamepad1.left_bumper;

        // Pressing Right Bumper toggles between Forward (1) and Off (0)
        if (rightBumperCurrentState && !rightBumperPreviousState) {
            intakeState = (intakeState == 1) ? 0 : 1;
        }
        rightBumperPreviousState = rightBumperCurrentState;

        // Pressing Left Bumper toggles between Reverse (-1) and Off (0)
        if (leftBumperCurrentState && !leftBumperPreviousState) {
            intakeState = (intakeState == -1) ? 0 : -1;
        }
        leftBumperPreviousState = leftBumperCurrentState;

        // Set power based on intakeState
        if (intakeState == 1) {
            intakePower = 1.0;
        } else if (intakeState == -1) {
            intakePower = -1.0;
        } else {
            intakePower = 0.0;
        }

        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);

        // Process trigger toggle launcher controls
        launchers();

        /*
         * Telemetry outputs for drive, intake, and launchers.
         */
        telemetry.addData("Drive Power", "LF: %.2f | RF: %.2f | LB: %.2f | RB: %.2f",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("Intake Mode", (intakeState == 1) ? "Forward" : (intakeState == -1) ? "Reverse" : "Off");
        telemetry.addData("Pollen Launcher Toggle (LT)", pollenToggleState);
        telemetry.addData("Nectar Launcher Toggle (RT)", nectarToggleState);
        telemetry.addData("Pollen Velocity", pollenLauncher.getVelocity());
        telemetry.addData("Nectar Velocity", nectarLauncher.getVelocity());
    }

    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe - rotate;
        rightBackPower = forward + strafe + rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launchers() {
        // Evaluate analog trigger presses as boolean edge triggers
        boolean leftTriggerCurrentState = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean rightTriggerCurrentState = gamepad1.right_trigger > TRIGGER_THRESHOLD;

        // Toggle Pollen Launcher state on Left Trigger edge press
        if (leftTriggerCurrentState && !leftTriggerPreviousState) {
            pollenToggleState = !pollenToggleState;
        }
        leftTriggerPreviousState = leftTriggerCurrentState;

        // Toggle Nectar Launcher state on Right Trigger edge press
        if (rightTriggerCurrentState && !rightTriggerPreviousState) {
            nectarToggleState = !nectarToggleState;
        }
        rightTriggerPreviousState = rightTriggerCurrentState;

        // Drive Pollen Launcher motor according to its toggle state
        if (pollenToggleState) {
            pollenLauncher.setVelocity(POLLEN_LAUNCHER_TARGET_VELOCITY);
        } else {
            pollenLauncher.setVelocity(0);
        }

        // Drive Nectar Launcher motor according to its toggle state
        if (nectarToggleState) {
            nectarLauncher.setVelocity(NECTAR_LAUNCHER_TARGET_VELOCITY);
        } else {
            nectarLauncher.setVelocity(0);
        }

        // Turn on feeder servo if an active launcher reaches its minimum target speed
        boolean pollenReady = pollenToggleState && (pollenLauncher.getVelocity() > POLLEN_LAUNCHER_MIN_VELOCITY);
        boolean nectarReady = nectarToggleState && (nectarLauncher.getVelocity() > NECTAR_LAUNCHER_MIN_VELOCITY);

        if (pollenReady || nectarReady) {
            windmillServo.setPower(1.0);
        } else {
            windmillServo.setPower(0.0);
        }
    }
}
