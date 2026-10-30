/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous Use This Code", group = "ERROR404")
//@Disabled
public class AutonomousUseThisCode extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx pollenLauncher = null;

    // Declare gate servos as standard Servo
    private Servo nectarGate = null;
    private Servo pollenGate = null;

    // ==========================================
    // TUNING VARIABLES (Easily adjust times and speeds here)
    // ==========================================
    // Launcher Settings
    public final int POLLEN_TARGET_VELOCITY = 1225; // Pollen (small Yellow ballS) launcher speed
    public final double POLLEN_MIN_GATE_VELOCITY = POLLEN_TARGET_VELOCITY * 0.90;
    public final double POLLEN_FIRE_DURATION = 4.0; // Seconds to fire pollen balls

    // Movement Speeds
    public final double DRIVE_SPEED = 0.8;
    public final double STRAFE_SPEED = 0.8; // Speed used for strafing left

    // Drive Balance Trims (Matches TeleOp to prevent drifting while driving straight)
    public final double LEFT_DRIVE_MULTIPLIER = 1.0;
    public final double RIGHT_DRIVE_MULTIPLIER = 1.0;

    // Movement Durations (Seconds)
    public final double MOVE_BACK_DURATION = 0.5;
    public final double STRAFE_LEFT_DURATION = 1.2;   // Adjust this duration to control how far it strafes left
    // ==========================================

    // Pollen & Nectar Gate Positions
    public final double POLLEN_GATE_CLOSED = 0.0;
    public final double POLLEN_GATE_OPEN = 0.3;
    public final double NECTAR_GATE_CLOSED = 0.0;

    @Override
    public void runOpMode() {

        // ==========================================
        // CONTROL HUB MOTORS & SERVOS
        // ==========================================
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");   // Port 0
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");       // Port 1
        nectarGate = hardwareMap.get(Servo.class, "nectar_gate");
        nectarGate.setDirection(Servo.Direction.REVERSE);

        // ==========================================
        // EXPANSION HUB MOTORS & SERVOS
        // ==========================================
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); // Port 0
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");   // Port 1
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "pollen_launcher");   // Port 3
        pollenGate = hardwareMap.get(Servo.class, "pollen_gate");               // Servo port

        // Drivetrain direction setup
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Launcher direction setup
        pollenLauncher.setDirection(DcMotor.Direction.FORWARD);

        // Brake behavior for instant stopping on all motors
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        pollenLauncher.setZeroPowerBehavior(BRAKE);

        // Configure launcher motors to use encoders and PIDF tuning
        pollenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // Ensure gates start firmly in the closed position upon initialization
        nectarGate.setPosition(NECTAR_GATE_CLOSED);
        pollenGate.setPosition(POLLEN_GATE_CLOSED);

        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // ==========================================
            // STEP 1: Fire pollen balls for 4.0 seconds (Fixed Gate Flutter)
            // ==========================================
            double fireStartTime = getRuntime();
            boolean gateOpened = false; // Latch flag to keep gate open once triggered

            while (opModeIsActive() && (getRuntime() - fireStartTime < POLLEN_FIRE_DURATION)) {
                pollenLauncher.setVelocity(POLLEN_TARGET_VELOCITY);

                // Open gate once velocity threshold is met and keep it open
                if (!gateOpened && pollenLauncher.getVelocity() >= POLLEN_MIN_GATE_VELOCITY) {
                    pollenGate.setPosition(POLLEN_GATE_OPEN);
                    gateOpened = true;
                }

                telemetry.addData("Autonomous Step", "Firing Pollen Launcher");
                telemetry.addData("Velocity", pollenLauncher.getVelocity());
                telemetry.addData("Gate Open", gateOpened);
                telemetry.update();
            }

            // Turn off pollen launcher and close gate
            pollenLauncher.setVelocity(0);
            pollenGate.setPosition(POLLEN_GATE_CLOSED);

            // ==========================================
            // STEP 2: Move backward using a variable
            // ==========================================
            double moveBackStartTime = getRuntime();
            while (opModeIsActive() && (getRuntime() - moveBackStartTime < MOVE_BACK_DURATION)) {
                setDrivePower(-DRIVE_SPEED, -DRIVE_SPEED, -DRIVE_SPEED, -DRIVE_SPEED);
                telemetry.addData("Autonomous Step", "Moving Backwards");
                telemetry.update();
            }
            stopDrive();

            // ==========================================
            // STEP 3: Strafe left using a variable
            // ==========================================
            double strafeLeftStartTime = getRuntime();
            while (opModeIsActive() && (getRuntime() - strafeLeftStartTime < STRAFE_LEFT_DURATION)) {
                // Mecanum strafe left formula: LF(-), RF(+), LB(+), RB(-)
                setDrivePower(-STRAFE_SPEED, STRAFE_SPEED, STRAFE_SPEED, -STRAFE_SPEED);
                telemetry.addData("Autonomous Step", "Strafing Left");
                telemetry.update();
            }
            stopDrive();

            // ==========================================
            // STEP 4: Complete
            // ==========================================
            telemetry.addData("Autonomous Step", "Complete");
            telemetry.update();
        }
    }

    // ==========================================
    // HELPER METHODS
    // ==========================================

    // Applies balance multipliers so straight/strafe driving doesn't drift unevenly
    private void setDrivePower(double lf, double rf, double lb, double rb) {
        leftFrontDrive.setPower(lf * LEFT_DRIVE_MULTIPLIER);
        rightFrontDrive.setPower(rf * RIGHT_DRIVE_MULTIPLIER);
        leftBackDrive.setPower(lb * LEFT_DRIVE_MULTIPLIER);
        rightBackDrive.setPower(rb * RIGHT_DRIVE_MULTIPLIER);
    }

    // Stops all drivetrain motors safely
    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}