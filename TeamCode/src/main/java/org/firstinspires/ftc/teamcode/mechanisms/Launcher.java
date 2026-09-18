package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is the Launcher fly wheel class.
 * Configure launch motor and feeder directions i.e. FORWARD/REVERSE based on hardware setting.
 */
public class Launcher {
    private DcMotorEx launcher;
    private String launcherName = "launcher";
    DcMotorSimple.Direction launcherDirection = FORWARD;
    DcMotor.ZeroPowerBehavior laucherZeroPowerBehavior = BRAKE;

    private double targetVelocity = 1300;
    private double minVelocity    = 1280;

    private CRServo feeder;
    private String feederName = "feeder";
    DcMotorSimple.Direction feederDirection = FORWARD;

    private double feederRunSec = 0.20;

    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING}
    private LaunchState launchState = LaunchState.IDLE;

    private final ElapsedTime feederTimer = new ElapsedTime();

    public void build(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, launcherName);
        launcher.setMode(RUN_USING_ENCODER);
        launcher.setDirection(launcherDirection);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher.setVelocity(0.0);
        feeder = hardwareMap.get(CRServo.class, feederName);
        feeder.setDirection(feederDirection);
        feeder.setPower(0.0);
    }

    public Launcher launcherName(String launcherName) {
        this.launcherName = launcherName;
        return this;
    }

    public Launcher launcherDirection(DcMotorSimple.Direction motorDirection) {
        launcherDirection = motorDirection;
        return this;
    }

    public Launcher launcherUseBrakeMode(boolean useBrakeMode) {
        if (useBrakeMode)
            laucherZeroPowerBehavior = BRAKE;
        else
            laucherZeroPowerBehavior = FLOAT;
        return this;
    }

    public Launcher feederName(String feederName) {
        this.feederName = feederName;
        return this;
    }

    public Launcher feederDirection(DcMotorSimple.Direction motorDirection) {
        feederDirection = motorDirection;
        return this;
    }

    public void setMotorDirection(DcMotorSimple.Direction motorDirection) {
        launcher.setDirection(motorDirection);
    }

    public DcMotorSimple.Direction getMotorDirection() {
        return launcherDirection;
    }

    public void setFeederDirection(DcMotorSimple.Direction motorDirection) {
        feederDirection = motorDirection;
        feeder.setDirection(feederDirection);
    }

    public DcMotorSimple.Direction getFeederDirection() {
        return feederDirection;
    }

    public void setLauncherOff() {
        launcher.setVelocity(0.0);
        feeder.setPower(0.0);
        feeder.setDirection(feederDirection);
        launchState = LaunchState.IDLE;
    }

    public void launch(boolean launch) {
        switch (launchState) {
            case IDLE:
                if (launch) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                launcher.setVelocity(targetVelocity);
                if (launcher.getVelocity() >= minVelocity)
                    launchState = LaunchState.LAUNCH;
                break;

            case LAUNCH:
                feeder.setPower(1.0);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() >= feederRunSec) {
                    feeder.setPower(0.0);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public String getState() {
        return launchState.toString();
    }

    public double getLauncherVelocity() {
        return launcher.getVelocity();
    }

    public void setLauncherVelocity(double target, double min) {
        targetVelocity = target;
        minVelocity = min;
    }

    public void setFeederRunSec(double second) {
        feederRunSec = second;
    }
}