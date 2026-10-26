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
public class AutonomousBioBuzz extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx pollenLauncher = null;
    private DcMotorEx nectarLauncher = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;

    // Declare gate servos as standard Servo
    private Servo nectarGate = null;
    private Servo pollenGate = null;

    // ==========================================
    // TUNING VARIABLES (Easily adjust times and speeds here)
    // ==========================================
    // Launcher Settings
    public final int POLLEN_TARGET_VELOCITY = 1225;
    public final double POLLEN_MIN_GATE_VELOCITY = POLLEN_TARGET_VELOCITY * 0.90;
    public final double POLLEN_FIRE_DURATION = 4.0; // Seconds to fire pollen balls

    // Movement Speeds
    public final double DRIVE_SPEED = 0.8;
    public final double TURN_SPEED = 0.3;

    // Drive Balance Trims (Matches TeleOp to prevent drifting while driving straight)
    public final double LEFT_DRIVE_MULTIPLIER = 1.0;
    public final double RIGHT_DRIVE_MULTIPLIER = 0.8;

    // Movement Durations (Seconds)
    public final double MOVE_BACK_DURATION = 0.5;
    public final double TURN_RIGHT_DURATION = 0.8;   // Adjust this to dial in the exact 90-degree turn
    public final double MOVE_FORWARD_DURATION = 0.5;
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
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");             // Port 2
        nectarLauncher = hardwareMap.get(DcMotorEx.class, "nectar_launcher");   // Port 3
        nectarGate = hardwareMap.get(Servo.class, "nectar_gate");
        nectarGate.setDirection(Servo.Direction.REVERSE);

        // ==========================================
        // EXPANSION HUB MOTORS & SERVOS
        // ==========================================
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); // Port 0
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");   // Port 1
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");           // Port 2
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "pollen_launcher");   // Port 3
        pollenGate = hardwareMap.get(Servo.class, "pollen_gate");               // Servo port

        // Drivetrain direction setup
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Launcher direction setup
        nectarLauncher.setDirection(DcMotor.Direction.REVERSE);
        pollenLauncher.setDirection(DcMotor.Direction.FORWARD);

        // Brake behavior for instant stopping on all motors
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        pollenLauncher.setZeroPowerBehavior(BRAKE);
        nectarLauncher.setZeroPowerBehavior(BRAKE);

        // Configure launcher motors to use encoders and PIDF tuning
        pollenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        nectarLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // Ensure gates start firmly in the closed position upon initialization
        nectarGate.setPosition(NECTAR_GATE_CLOSED);
        pollenGate.setPosition(POLLEN_GATE_CLOSED);

        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // ==========================================
            // STEP 1: Fire pollen balls for 4.0 seconds
            // ==========================================
            double fireStartTime = getRuntime();
            while (opModeIsActive() && (getRuntime() - fireStartTime < POLLEN_FIRE_DURATION)) {
                pollenLauncher.setVelocity(POLLEN_TARGET_VELOCITY);

                // Open gate automatically once velocity threshold is met
                if (pollenLauncher.getVelocity() >= POLLEN_MIN_GATE_VELOCITY) {
                    pollenGate.setPosition(POLLEN_GATE_OPEN);
                } else {
                    pollenGate.setPosition(POLLEN_GATE_CLOSED);
                }

                telemetry.addData("Autonomous Step", "Firing Pollen Launcher");
                telemetry.addData("Velocity", pollenLauncher.getVelocity());
                telemetry.update();
            }

            // Turn off pollen launcher and close gate
            pollenLauncher.setVelocity(0);
            pollenGate.setPosition(POLLEN_GATE_CLOSED);

            // ==========================================
            // STEP 2: Move backward for 0.5 seconds
            // ==========================================
            double moveBackStartTime = getRuntime();
            while (opModeIsActive() && (getRuntime() - moveBackStartTime < MOVE_BACK_DURATION)) {
                setDrivePower(-DRIVE_SPEED, -DRIVE_SPEED, -DRIVE_SPEED, -DRIVE_SPEED);
                telemetry.addData("Autonomous Step", "Moving Backwards");
                telemetry.update();
            }
            stopDrive();

            // ==========================================
            // STEP 3: Turn right 90 degrees
            // ==========================================
            double turnStartTime = getRuntime();
            while (opModeIsActive() && (getRuntime() - turnStartTime < TURN_RIGHT_DURATION)) {
                // Mecanum turn right: left side forward (+), right side backward (-)
                setDrivePower(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED);
                telemetry.addData("Autonomous Step", "Turning Right");
                telemetry.update();
            }
            stopDrive();

            // ==========================================
            // STEP 4: Move forward for 0.5 seconds
            // ==========================================
            double moveForwardStartTime = getRuntime();
            while (opModeIsActive() && (getRuntime() - moveForwardStartTime < MOVE_FORWARD_DURATION)) {
                setDrivePower(DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED);
                telemetry.addData("Autonomous Step", "Moving Forward");
                telemetry.update();
            }
            stopDrive();

            // ==========================================
            // STEP 5: Complete
            // ==========================================
            telemetry.addData("Autonomous Step", "Complete");
            telemetry.update();
        }
    }

    // Helper method to apply balance multipliers so straight driving doesn't drift
    private void setDrivePower(double lf, double rf, double lb, double rb) {
        leftFrontDrive.setPower(lf * LEFT_DRIVE_MULTIPLIER);
        rightFrontDrive.setPower(rf * RIGHT_DRIVE_MULTIPLIER);
        leftBackDrive.setPower(lb * LEFT_DRIVE_MULTIPLIER);
        rightBackDrive.setPower(rb * RIGHT_DRIVE_MULTIPLIER);
    }

    // Helper method to stop all drivetrain motors
    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}

        On Sun, Oct 25, 2026 at 8:43 PM Dreyken <starfright22@gmail.com> wrote:
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

        package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp Code Use This", group = "StarterBot")
//@Disabled
public class UseThisCode extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx pollenLauncher = null;
    private DcMotorEx nectarLauncher = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;

    // Declare gate servos as standard Servo
    private Servo nectarGate = null;
    private Servo pollenGate = null;

    // Separate speed variables for each launcher
    public final int NECTAR_TARGET_VELOCITY = 1800; // Nectar launcher speed
    public final int POLLEN_TARGET_VELOCITY = 1350; // Pollen launcher speed

    // Minimum velocity thresholds set to 90% of the target velocity
    public final double NECTAR_MIN_GATE_VELOCITY = NECTAR_TARGET_VELOCITY * 0.90;
    public final double POLLEN_MIN_GATE_VELOCITY = POLLEN_TARGET_VELOCITY * 0.90;

    // Pollen Gate Positions
    public final double POLLEN_GATE_CLOSED = 0.0;
    public final double POLLEN_GATE_OPEN = 0.3;

    // Nectar Gate Positions
    public final double NECTAR_GATE_CLOSED = 0.0;
    public final double NECTAR_GATE_OPEN = 0.3;

    // Drive Balance Trims (Adjust these if the robot still drifts)
    public final double LEFT_DRIVE_MULTIPLIER = 1.0;
    public final double RIGHT_DRIVE_MULTIPLIER = 0.8;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Intake power variable
    double intakePower;

    // Toggle state variables for triggers & bumpers
    boolean leftTriggerWasPressed = false;
    boolean nectarLauncherOn = false;

    boolean rightTriggerWasPressed = false;
    boolean pollenLauncherOn = false;

    boolean leftBumperWasPressed = false;
    boolean intakeForwardOn = false;

    boolean rightBumperWasPressed = false;
    boolean intakeReverseOn = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // ==========================================
        // CONTROL HUB MOTORS & SERVOS
        // ==========================================
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");   // Port 0
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");       // Port 1
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");             // Port 2
        nectarLauncher = hardwareMap.get(DcMotorEx.class, "nectar_launcher");   // Port 3
        nectarGate = hardwareMap.get(Servo.class, "nectar_gate");
        nectarGate.setDirection(Servo.Direction.REVERSE);

        // ==========================================
        // EXPANSION HUB MOTORS & SERVOS
        // ==========================================
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); // Port 0
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");   // Port 1
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");           // Port 2
        pollenLauncher = hardwareMap.get(DcMotorEx.class, "pollen_launcher");   // Port 3
        pollenGate = hardwareMap.get(Servo.class, "pollen_gate");               // Servo port

        // Drivetrain direction setup
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Intake direction setup (Right motor reversed so both spin together)
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        // Launcher direction setup (Nectar reversed as requested)
        nectarLauncher.setDirection(DcMotor.Direction.REVERSE);
        pollenLauncher.setDirection(DcMotor.Direction.FORWARD);

        // Brake behavior for instant stopping on all motors including launchers
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        leftIntake.setZeroPowerBehavior(BRAKE);
        rightIntake.setZeroPowerBehavior(BRAKE);

        pollenLauncher.setZeroPowerBehavior(BRAKE);
        nectarLauncher.setZeroPowerBehavior(BRAKE);

        // Configure launcher motors to use encoders and PIDF tuning
        pollenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pollenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        nectarLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // Ensure gates start firmly in the closed position upon initialization
        nectarGate.setPosition(NECTAR_GATE_CLOSED);
        pollenGate.setPosition(POLLEN_GATE_CLOSED);

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
        // Driver controls: Left stick Y = forward/reverse, Left stick X = strafe, Right stick X = turn
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Launcher Toggle Logic for Left Trigger (nectar_launcher) and Right Trigger (pollen_launcher)
        boolean leftTriggerPressed = (gamepad1.left_trigger > 0.5);
        if (leftTriggerPressed && !leftTriggerWasPressed) {
            nectarLauncherOn = !nectarLauncherOn;
        }
        leftTriggerWasPressed = leftTriggerPressed;

        boolean rightTriggerPressed = (gamepad1.right_trigger > 0.5);
        if (rightTriggerPressed && !rightTriggerWasPressed) {
            pollenLauncherOn = !pollenLauncherOn;
        }
        rightTriggerWasPressed = rightTriggerPressed;

        launch();

        // Intake Toggle Logic for Left Bumper (Forward) and Right Bumper (Reverse)
        boolean leftBumperPressed = gamepad1.left_bumper;
        if (leftBumperPressed && !leftBumperWasPressed) {
            intakeForwardOn = !intakeForwardOn;
            if (intakeForwardOn) {
                intakeReverseOn = false; // Turn off reverse if forward is toggled on
            }
        }
        leftBumperWasPressed = leftBumperPressed;

        boolean rightBumperPressed = gamepad1.right_bumper;
        if (rightBumperPressed && !rightBumperWasPressed) {
            intakeReverseOn = !intakeReverseOn;
            if (intakeReverseOn) {
                intakeForwardOn = false; // Turn off forward if reverse is toggled on
            }
        }
        rightBumperWasPressed = rightBumperPressed;

        // Calculate intake power based on toggle states
        if (intakeForwardOn) {
            intakePower = 1.0;
        } else if (intakeReverseOn) {
            intakePower = -1.0;
        } else {
            intakePower = 0.0;
        }

        // Write power to intake motors
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Launchers", "nectar (%s), pollen (%s)", nectarLauncherOn ? "ON" : "OFF", pollenLauncherOn ? "ON" : "OFF");
        telemetry.addData("Launcher Vels", "nectar (%.0f), pollen (%.0f)", nectarLauncher.getVelocity(), pollenLauncher.getVelocity());
    }

    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Apply balance multipliers to counteract mechanical pull
        leftFrontDrive.setPower(leftFrontPower * LEFT_DRIVE_MULTIPLIER);
        rightFrontDrive.setPower(rightFrontPower * RIGHT_DRIVE_MULTIPLIER);
        leftBackDrive.setPower(leftBackPower * LEFT_DRIVE_MULTIPLIER);
        rightBackDrive.setPower(rightBackPower * RIGHT_DRIVE_MULTIPLIER);
    }

    void launch() {
        // Nectar Launcher & Gate (Left)
        if (nectarLauncherOn) {
            nectarLauncher.setVelocity(NECTAR_TARGET_VELOCITY);

            // Only open the gate once the motor velocity reaches or exceeds the minimum threshold
            if (nectarLauncher.getVelocity() >= NECTAR_MIN_GATE_VELOCITY) {
                nectarGate.setPosition(NECTAR_GATE_OPEN);
            } else {
                nectarGate.setPosition(NECTAR_GATE_CLOSED);
            }
        } else {
            nectarLauncher.setVelocity(0);
            nectarGate.setPosition(NECTAR_GATE_CLOSED);
        }

        // Pollen Launcher & Gate (Right)
        if (pollenLauncherOn) {
            pollenLauncher.setVelocity(POLLEN_TARGET_VELOCITY);

            // Only open the gate once the motor velocity reaches or exceeds the minimum threshold
            if (pollenLauncher.getVelocity() >= POLLEN_MIN_GATE_VELOCITY) {
                pollenGate.setPosition(POLLEN_GATE_OPEN);
            } else {
                pollenGate.setPosition(POLLEN_GATE_CLOSED);
            }
        } else {
            pollenLauncher.setVelocity(0);
            pollenGate.setPosition(POLLEN_GATE_CLOSED);
        }
    }
}