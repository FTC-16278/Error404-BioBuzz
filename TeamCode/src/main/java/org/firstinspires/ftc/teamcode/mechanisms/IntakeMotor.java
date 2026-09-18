package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

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

    DcMotorSimple.Direction motorDirection = FORWARD;
    DcMotor.ZeroPowerBehavior motorZeroPowerBehavior = BRAKE;

    private enum IntakeDirection { FORWARD, REVERSE }
    private IntakeDirection intakeDirection = IntakeDirection.FORWARD;

    private enum IntakeState { OFF, ON }
    private IntakeState intakeState = IntakeState.OFF;

    private double motorPower = 0.75;

    public void build(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, motorName);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(motorDirection);
        intakeMotor.setZeroPowerBehavior(motorZeroPowerBehavior);
        intakeMotor.setPower(0.0);
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

    public IntakeMotor motorPower(double motorPower) {
        this.motorPower = motorPower;
        return this;
    }

    public String getMotorDirection() {
        return intakeDirection.toString();
    }

    public void setMotorDirectionForward() {
        motorDirection = FORWARD;
        intakeMotor.setDirection(motorDirection);
    }

    public void setMotorDirectionReverse() {
        motorDirection = REVERSE;
        intakeMotor.setDirection(motorDirection);
    }

    public void setMotorBrakeMode() {
        intakeMotor.setZeroPowerBehavior(BRAKE);
    }

    public void setMotorFloatMode() {
        intakeMotor.setZeroPowerBehavior(FLOAT);
    }

    public void setPower(double motorPower) {
        this.motorPower = motorPower;
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public String getState() {
        return intakeState.toString();
    }

    public void setIntakeOff() {
        run(false, true);
    }

    public void setIntakeOn() {
        run(true, false);
    }

    public void setIntakeOn(double setPower) {
        setPower(setPower);
        setIntakeOn();
    }

    public void run(boolean intakeOn, boolean intakeOff) {
        switch (intakeState) {
            case OFF:
                if (intakeOn) {
                    intakeState = IntakeState.ON;
                    intakeDirection = IntakeDirection.FORWARD;
                    intakeMotor.setPower(motorPower);
                }
                break;

            case ON:
                if (intakeOff) {
                    intakeState = IntakeState.OFF;
                    intakeMotor.setPower(0.0);
                }
                break;
        }
    }
}
