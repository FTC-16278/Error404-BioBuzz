package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

/**
 * This is the Mecanum Drive chain class.
 * Configure each of the 4 driving motors direction i.e. FORWARD/REVERSE based on hardware setting.
 */
public class MecanumDrive {
    private List<DcMotor> drives;
    private DcMotor leftDriveFront;
    private DcMotor rightDriveFront;
    private DcMotor leftDriveBack;
    private DcMotor rightDriveBack;

    private String leftFrontName = "leftFront";
    private String leftBackName = "leftBack";
    private String rightFrontName = "rightFront";
    private String rightBackName = "rightBack";

    DcMotorSimple.Direction leftFrontDirection = REVERSE;
    DcMotorSimple.Direction leftBackDirection = REVERSE;
    DcMotorSimple.Direction rightFrontDirection = FORWARD;
    DcMotorSimple.Direction rightBackDirection = FORWARD;
    DcMotor.ZeroPowerBehavior driveZeroPowerBehavior = BRAKE;

    private double leftPowerFront;
    private double rightPowerFront;
    private double leftPowerBack;
    private double rightPowerBack;

    private double maxPower = 1.0;

    private IMU imu;
    private double driveAngularOffset;

    public MecanumDrive leftFrontName(String driveName) {
        this.leftFrontName = driveName;
        return this;
    }

    public MecanumDrive leftBackName(String driveName) {
        this.leftBackName = driveName;
        return this;
    }

    public MecanumDrive rightFrontName(String driveName) {
        this.rightFrontName = driveName;
        return this;
    }

    public MecanumDrive rightBackName(String driveName) {
        this.rightBackName = driveName;
        return this;
    }

    public MecanumDrive leftFrontDirection(DcMotorSimple.Direction driveDirection) {
        leftFrontDirection = driveDirection;
        return this;
    }

    public MecanumDrive leftBackDirection(DcMotorSimple.Direction driveDirection) {
        leftBackDirection = driveDirection;
        return this;
    }

    public MecanumDrive rightFrontDirection(DcMotorSimple.Direction driveDirection) {
        rightFrontDirection = driveDirection;
        return this;
    }
    public MecanumDrive rightBackDirection(DcMotorSimple.Direction driveDirection) {
        rightBackDirection = driveDirection;
        return this;
    }

    public MecanumDrive useBrakeMode(boolean useBrakeMode) {
        if (useBrakeMode)
            driveZeroPowerBehavior = BRAKE;
        else
            driveZeroPowerBehavior = FLOAT;
        return this;
    }

    public void build(HardwareMap hardwareMap) {
        leftDriveFront = hardwareMap.get(DcMotor.class, leftFrontName);
        leftDriveBack = hardwareMap.get(DcMotor.class, leftBackName);
        rightDriveFront = hardwareMap.get(DcMotor.class, rightFrontName);
        rightDriveBack = hardwareMap.get(DcMotor.class, rightBackName);

        drives = Arrays.asList(leftDriveFront, leftDriveBack, rightDriveFront, rightDriveBack);

        for (DcMotor motor : drives) {
            motor.setMode(RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(driveZeroPowerBehavior);
            motor.setPower(0.0);
        }

        leftDriveFront.setDirection(leftFrontDirection);
        leftDriveBack.setDirection(leftBackDirection);
        rightDriveFront.setDirection(rightFrontDirection);
        rightDriveBack.setDirection(rightBackDirection);
    }

    public void setLeftFrontDirection(DcMotorSimple.Direction driveDirection) {
        leftDriveFront.setDirection(driveDirection);
    }

    public void setLeftBackDirection(DcMotorSimple.Direction driveDirection) {
        leftDriveBack.setDirection(driveDirection);
    }

    public void setRightFrontDirection(DcMotorSimple.Direction driveDirection) {
        rightDriveFront.setDirection(driveDirection);
    }
    public void setRightBackDirection(DcMotorSimple.Direction driveDirection) {
        rightDriveBack.setDirection(driveDirection);
    }

    public void setDrivesToBrake() {
        for (DcMotor motor : drives)
            motor.setZeroPowerBehavior(BRAKE);
    }

    public void setDrivesToFloat() {
        for (DcMotor motor : drives)
            motor.setZeroPowerBehavior(FLOAT);
    }

    public void stopDrives() {
        for (DcMotor motor : drives) {
            motor.setPower(0.0);
        }
    }

    public void restartDrives() {
        for (DcMotor motor : drives) {
            motor.setMode(RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
        }
        setDrivesToBrake();
    }

    public void resetDrives() {
        for (DcMotor motor : drives) {
            motor.setPower(0.0);
            motor.setMode(STOP_AND_RESET_ENCODER);
        }
        setDrivesToBrake();
    }

    public void initRevIMU(HardwareMap hardwareMap,
                           RevHubOrientationOnRobot.LogoFacingDirection logoDir,
                           RevHubOrientationOnRobot.UsbFacingDirection usbDir) {
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(logoDir, usbDir);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void runDrive(double forward, double strafe, double rotate) {
        leftPowerFront  = forward + strafe + rotate;
        rightPowerFront = forward - strafe - rotate;
        leftPowerBack   = forward - strafe + rotate;
        rightPowerBack  = forward + strafe - rotate;

        double drivePowerMax = Math.max(Math.abs(leftPowerFront), Math.max(Math.abs(rightPowerFront),
                Math.max(Math.abs(leftPowerBack), Math.abs(rightPowerBack))));

        if (drivePowerMax > maxPower) {
            leftPowerFront  = maxPower * (leftPowerFront / drivePowerMax);
            rightPowerFront = maxPower * (rightPowerFront / drivePowerMax);
            leftPowerBack   = maxPower * (leftPowerBack / drivePowerMax);
            rightPowerBack  = maxPower * (rightPowerBack / drivePowerMax);
        }

        leftDriveFront.setPower(leftPowerFront);
        rightDriveFront.setPower(rightPowerFront);
        leftDriveBack.setPower(leftPowerBack);
        rightDriveBack.setPower(rightPowerBack);
    }

    public void runDriveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - this.getDriveHeading(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.runDrive(newForward, newStrafe, rotate);
    }

    public void setDriveAngularOffset(double offset) {
        driveAngularOffset = offset;
    }

    public double getDriveAngularOffset(AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.RADIANS)
            return Math.toRadians(driveAngularOffset);
        else
            return driveAngularOffset;
    }

    public double getDriveHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit) + this.getDriveAngularOffset(angleUnit);
    }

    public void resetDriveYaw() {
        imu.resetYaw();
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public double getMaxPower() {
        return this.maxPower;
    }

    public double getLeftPowerFront() {
        return leftPowerFront;
    }

    public double getLeftPowerBack() {
        return leftPowerBack;
    }

    public double getRightPowerFront() {
        return rightPowerFront;
    }

    public double getRightPowerBack() {
        return rightPowerBack;
    }
}