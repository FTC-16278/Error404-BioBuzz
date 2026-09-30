package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is the Intake mechanism class.
 * Configure intake motor direction i.e. FORWARD/REVERSE based on hardware setting.
 */
public class IntakeMotor {
    private DcMotor intakeMotor;
    private String motorName = "intakeMotor";
    private DcMotorSimple.Direction motorDirection = FORWARD;
    private DcMotor.ZeroPowerBehavior motorZeroPowerBehavior = BRAKE;

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private String leftServoName = "leftServo";
    private String rightServoName = "rightServo";
    private DcMotorSimple.Direction leftServoDir = FORWARD;
    private DcMotorSimple.Direction rightServoDir = REVERSE;

    private double maxPower = 1.0;

    public void build(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, motorName);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(motorDirection);
        intakeMotor.setZeroPowerBehavior(motorZeroPowerBehavior);
        intakeMotor.setPower(0.0);
        leftIntakeServo = hardwareMap.get(CRServo.class, leftServoName);
        leftIntakeServo.setDirection(leftServoDir);
        leftIntakeServo.setPower(0.0);
        rightIntakeServo = hardwareMap.get(CRServo.class, rightServoName);
        rightIntakeServo.setDirection(rightServoDir);
        rightIntakeServo.setPower(0.0);
    }

    public IntakeMotor motorName(String motorName) {
        this.motorName = motorName;
        return this;
    }

    public IntakeMotor motorDirection(DcMotorSimple.Direction motorDirection) {
        this.motorDirection = motorDirection;
        return this;
    }

    public IntakeMotor motorUseBrakeMode(boolean useBrakeMode) {
        if (useBrakeMode)
            motorZeroPowerBehavior = BRAKE;
        else
            motorZeroPowerBehavior = FLOAT;
        return this;
    }

    public IntakeMotor leftServoName(String leftServoName) {
        this.leftServoName = leftServoName;
        return this;
    }

    public IntakeMotor rightServoName(String rightServoName) {
        this.rightServoName = rightServoName;
        return this;
    }

    public IntakeMotor leftServoDirection(DcMotorSimple.Direction leftServoDir) {
        this.leftServoDir = leftServoDir;
        return this;
    }

    public IntakeMotor rightServoDirection(DcMotorSimple.Direction rightServoDir) {
        this.rightServoDir = rightServoDir;
        return this;
    }

    public void setMotorDirection(DcMotorSimple.Direction motorDirection) {
        this.motorDirection = motorDirection;
        intakeMotor.setDirection(motorDirection);
    }

    public void setLeftServoDirection(DcMotorSimple.Direction leftServoDir) {
        this.leftServoDir = leftServoDir;
        leftIntakeServo.setDirection(leftServoDir);
    }

    public void setRightServoDirection(DcMotorSimple.Direction rightServoDir) {
        this.rightServoDir = rightServoDir;
        rightIntakeServo.setDirection(rightServoDir);
    }

    public void setIntakeMotorPower(double motorPower) {
        intakeMotor.setPower(motorPower);
    }

    public double getIntakeMotorPower() {
        return intakeMotor.getPower();
    }

    public void setLeftIntakeServoPower(double motorPower) {
        leftIntakeServo.setPower(motorPower);
    }

    public double getLeftIntakeServoPower() {
        return leftIntakeServo.getPower();
    }

    public void setRightIntakeServoPower(double motorPower) {
        rightIntakeServo.setPower(motorPower);
    }

    public double getRightIntakeServoPower() {
        return rightIntakeServo.getPower();
    }

    public void setIntakeOff() {
        intakeMotor.setPower(0.0);
        leftIntakeServo.setPower(0.0);
        rightIntakeServo.setPower(0.0);
    }

    public void run(double motorPower) {
        double usePower = Math.min(Math.abs(motorPower), maxPower);
        if (motorPower < 0) usePower *= -1;

        intakeMotor.setPower(usePower);
        leftIntakeServo.setPower(usePower);
        rightIntakeServo.setPower(usePower);
    }
}
