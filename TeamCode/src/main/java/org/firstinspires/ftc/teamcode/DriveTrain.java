package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.PID;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.concurrent.TimeUnit;

public class DriveTrain {

    public DriveTrain(DriveMode driveMode) {
        this.driveMode = driveMode;
        switch (driveMode) {
            case SLIDE:
            case MECANUM_X:
            case MECANUM_O:
            case HOLONOMIC: numOfMotors = 4;
                break;
        }
    }
    public DriveTrain(DriveMode driveMode, int numOfMotors) {
        this.driveMode = driveMode;
        this.numOfMotors = numOfMotors;
    }

    private DcMotor frontRight, frontLeft, backRight, backLeft; //Drive Motors
    private Gamepad gamepad = null;
    private Telemetry telemetry = null;
    private HardwareMap hardwareMap = null;

    static final int COUNTS_PER_REVOLUTION_40 = 1120; //AndyMark 40:1 Motors



    //Class variables for TeleOp Use
    private DriveMode driveMode;
    private int numOfMotors; //Only 2 and 4 are accepted values
    private boolean frontModeOn = true; //flips the front and back of robot for ARCADE & TANK
    private double drivePwrMax = 0.80; //80% by default
    private double turnPwrMax = 0.60; //60% by default, only for slide/mecanum/holonomic
    private double gearRatio = 1; //Driven / Driver, 1 by default
    private double frPower, flPower, brPower, blPower;

    //Code to be run in the init() method of your OpMode
    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }
    public void setNumOfMotors(int num) {
        numOfMotors = num;
    }
    public void setDrivePwrMax(double maxPwr) {
        drivePwrMax = Range.clip(maxPwr, -1, 1);
    }
    public void setTurnPwrMax(double maxPwr) {
        turnPwrMax = Range.clip(maxPwr, -1, 1);
    }
    public void setGearRatio(double ratio) {
        gearRatio = ratio;
    }
    public void setConstants(double drivePwrMax, double turnPwrMax, double gearRatio) {
        setDrivePwrMax(drivePwrMax);
        setTurnPwrMax(turnPwrMax);
        setGearRatio(gearRatio);
    }
    public void setConstants(double drivePwrMax, double gearRatio) {
        setDrivePwrMax(drivePwrMax);
        setGearRatio(gearRatio);
    }
    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void setHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public void syncOpMode(Gamepad gamepad, Telemetry telemetry, HardwareMap hardwareMap) {
        setGamepad(gamepad);
        setTelemetry(telemetry);
        setHardware(hardwareMap);
        startTime = System.nanoTime();
    }
    public void setMotors(DcMotor left, DcMotor right) {
        frontLeft = left;
        frontRight = right;
        telemetry.addData(">", "Drive Train Initialization Successful");
    }
    public void setMotors(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        telemetry.addData(">", "Drive Train Initialization Successful");
    }
    public void setMotors() {
        if (hardwareMap != null) {
            if (numOfMotors == 4) {
                if (driveMode == DriveMode.SLIDE) {
                    frontRight = hardwareMap.dcMotor.get("nd");
                    frontLeft = hardwareMap.dcMotor.get("ed");
                    backRight = hardwareMap.dcMotor.get("wd");
                    backLeft = hardwareMap.dcMotor.get("sd");
                }
                else {
                    frontRight = hardwareMap.dcMotor.get("fr");
                    frontLeft = hardwareMap.dcMotor.get("fl");
                    backRight = hardwareMap.dcMotor.get("br");
                    backLeft = hardwareMap.dcMotor.get("bl");

                }
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            else {
                frontRight = hardwareMap.dcMotor.get("rd");
                frontLeft = hardwareMap.dcMotor.get("ld");
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            telemetry.addData(">", "Drive Train Initialization Successful");
        }
        else {
            if (telemetry != null) telemetry.addData("Error", "Need to Initialize HardwareMap");
        }
    }

    //Code to be run in the loop() method of your OpMode
    public void update() {
        if (gamepad != null) {
            switch (driveMode) {
                case TANK: updateTankDrive();
                    break;
                case ARCADE: updateArcadeDrive();
                    break;
                case SLIDE: updateSlideDrive();
                    break;
                case MECANUM_X: updateMecanumXDrive();
                    break;
                case MECANUM_O: updateMecanumODrive();
                    break;
                case HOLONOMIC: updateHolonomicDrive();
                    break;
            }
        }
        else {
            if (telemetry != null) telemetry.addData("Error", "Need to Initialize Gamepad");
        }
    }

    private void updateTankDrive() {
        if (frontModeOn) {
            flPower = -gamepad.left_stick_y * drivePwrMax;
            blPower = -gamepad.left_stick_y * drivePwrMax;
            frPower = -gamepad.right_stick_y * drivePwrMax;
            brPower = -gamepad.right_stick_y * drivePwrMax;
        }
        else {
            flPower = gamepad.right_stick_y * drivePwrMax;
            blPower = gamepad.right_stick_y * drivePwrMax;
            frPower = gamepad.left_stick_y * drivePwrMax;
            brPower = gamepad.left_stick_y * drivePwrMax;
        }
        if (gamepad.a) {
            frontModeOn = true;
        }
        else if (gamepad.y) {
            frontModeOn = false;
        }
        if (gamepad.b) {
            driveMode = DriveMode.ARCADE;
        }
    }
    private void updateArcadeDrive() {
        if (frontModeOn) {
            flPower = (-gamepad.left_stick_y + gamepad.right_stick_x) * drivePwrMax;
            frPower = (-gamepad.left_stick_y - gamepad.right_stick_x) * drivePwrMax;
            blPower = (-gamepad.left_stick_y + gamepad.right_stick_x) * drivePwrMax;
            brPower = (-gamepad.left_stick_y - gamepad.right_stick_x) * drivePwrMax;
        }
        else {
            flPower = (gamepad.left_stick_y + gamepad.right_stick_x) * drivePwrMax;
            frPower = (gamepad.left_stick_y - gamepad.right_stick_x) * drivePwrMax;
            blPower = (gamepad.left_stick_y + gamepad.right_stick_x) * drivePwrMax;
            brPower = (gamepad.left_stick_y - gamepad.right_stick_x) * drivePwrMax;
        }
        if (gamepad.y) {
            frontModeOn = true;
        }
        else if (gamepad.a) {
            frontModeOn = false;
        }
        if (gamepad.x) {
            driveMode = DriveMode.TANK;
        }
    }
    private void updateSlideDrive() { //FL = East, BR = West, BL = South, FR = North
        if (!(gamepad.left_stick_y < 0.05 && gamepad.left_stick_y > -0.05)) {
            brPower = -gamepad.left_stick_y * drivePwrMax;
            flPower = -gamepad.left_stick_y * drivePwrMax;
        }
        if (!(gamepad.left_stick_x < 0.05 && gamepad.left_stick_x > -0.05)) {
            frPower = gamepad.left_stick_x * drivePwrMax;
            blPower = gamepad.left_stick_x * drivePwrMax;
        }
        if (gamepad.left_stick_y == 0) {
            brPower = 0;
            flPower = 0;
        }
        if (gamepad.left_stick_x == 0) {
            frPower = 0;
            blPower = 0;
        }
        if (!(gamepad.right_stick_x < 0.05 && gamepad.right_stick_x > -0.05)) {
            frPower = gamepad.right_stick_x * turnPwrMax * drivePwrMax;
            blPower = -gamepad.right_stick_x * turnPwrMax * drivePwrMax;
            flPower = -gamepad.right_stick_x * turnPwrMax * drivePwrMax;
            brPower = gamepad.right_stick_x * turnPwrMax * drivePwrMax;
        }

    }
    private void updateMecanumXDrive() {
        frPower = (-gamepad.left_stick_y - gamepad.left_stick_x - gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        flPower = (-gamepad.left_stick_y + gamepad.left_stick_x + gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        brPower = (-gamepad.left_stick_y + gamepad.left_stick_x - gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        blPower = (-gamepad.left_stick_y - gamepad.left_stick_x + gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
    }
    private void updateMecanumODrive() {
        frPower = (-gamepad.left_stick_y + gamepad.left_stick_x - gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        flPower = (-gamepad.left_stick_y - gamepad.left_stick_x + gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        brPower = (-gamepad.left_stick_y - gamepad.left_stick_x - gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
        blPower = (-gamepad.left_stick_y + gamepad.left_stick_x + gamepad.right_stick_x * turnPwrMax) * drivePwrMax;
    }
    private void updateHolonomicDrive() {
        if (!(gamepad.right_stick_x < 0.03 && gamepad.right_stick_x > -0.03)) { //Move Clockwise/Anticlockwise
            flPower = gamepad.right_stick_x * drivePwrMax * turnPwrMax;
            frPower = -gamepad.right_stick_x * drivePwrMax * turnPwrMax;
            blPower = gamepad.right_stick_x * drivePwrMax * turnPwrMax;
            brPower = -gamepad.right_stick_x * drivePwrMax * turnPwrMax;
        }
        else if (!(gamepad.left_stick_y < 0.10 && gamepad.left_stick_y > -0.10) &&
                !(gamepad.left_stick_x < 0.10 && gamepad.left_stick_x > -0.10)) { //Move at diagonal
            if ((gamepad.left_stick_x > 0 && gamepad.left_stick_y < 0) ||
                    (gamepad.left_stick_x < 0 && gamepad.left_stick_y > 0)) { //ForwardRight/BackwardLeft
                flPower = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                frPower = (-gamepad.left_stick_y - gamepad.left_stick_x) * drivePwrMax; //SinA - CosA (A = theta of unit circle)
                blPower = (-gamepad.left_stick_y - gamepad.left_stick_x) * drivePwrMax; //SinA - CosA (A = theta of unit circle)
                brPower = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                if (gamepad.left_stick_x < 0) {
                    flPower *= -1;
                    brPower *= -1;
                }
            }
            else { //ForwardLeft/BackwardRight
                flPower = (-gamepad.left_stick_y + gamepad.left_stick_x) * drivePwrMax; //SinA + CosA (A = theta of unit circle)
                frPower = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                blPower = Math.sqrt(Math.pow(gamepad.left_stick_y, 2) + Math.pow(gamepad.left_stick_x, 2)) * drivePwrMax; //Pythagorean Theorem
                brPower = (-gamepad.left_stick_y + gamepad.left_stick_x) * drivePwrMax; //SinA + CosA (A = theta of unit circle)
                if (gamepad.left_stick_x > 0) {
                    frPower *= -1;
                    blPower *= -1;
                }
            }
        }
        else if (!(gamepad.left_stick_y < 0.10 && gamepad.left_stick_y > -0.10)) { //Move forward/backward
            flPower = -gamepad.left_stick_y * drivePwrMax;
            frPower = -gamepad.left_stick_y * drivePwrMax;
            blPower = -gamepad.left_stick_y * drivePwrMax;
            brPower = -gamepad.left_stick_y * drivePwrMax;
        }
        else if (!(gamepad.left_stick_x < 0.10 && gamepad.left_stick_x > -0.10)) { //Move Right/left
            flPower = gamepad.left_stick_x * drivePwrMax;
            frPower = -gamepad.left_stick_x * drivePwrMax;
            blPower = -gamepad.left_stick_x * drivePwrMax;
            brPower = gamepad.left_stick_x * drivePwrMax;
        }
        else if (gamepad.right_stick_x == 0 && gamepad.left_stick_y == 0 && gamepad.left_stick_x == 0) { //Stop Motion
            flPower = 0;
            frPower = 0;
            blPower = 0;
            brPower = 0;
        }

    }

    public void initialize() {
        if (numOfMotors == 4) {
            flPower = Range.clip(flPower, -drivePwrMax, drivePwrMax);
            frontLeft.setPower(flPower);
            frPower = Range.clip(frPower, -drivePwrMax, drivePwrMax);
            frontRight.setPower(frPower);
            blPower = Range.clip(blPower, -drivePwrMax, drivePwrMax);
            backLeft.setPower(blPower);
            brPower = Range.clip(brPower, -drivePwrMax, drivePwrMax);
            backRight.setPower(brPower);
        }
        else {
            flPower = Range.clip(flPower, -drivePwrMax, drivePwrMax);
            frontLeft.setPower(flPower);
            frPower = Range.clip(frPower, -drivePwrMax, drivePwrMax);
            frontRight.setPower(frPower);
        }
    }

    public void telemetry() {
        if (telemetry != null) {
            switch (driveMode) {
                case TANK:
                    telemetry.addData("TANK DRIVE", "TELEMETRY");
                    telemetry.addData(">>>Left Pwr", frontLeft.getPower());
                    telemetry.addData(">>>Right Pwr", frontRight.getPower());
                    break;
                case ARCADE:
                    telemetry.addData("ARCADE DRIVE", "TELEMETRY");
                    telemetry.addData(">>>Left Pwr", frontLeft.getPower());
                    telemetry.addData(">>>Right Pwr", frontRight.getPower());
                    break;
                case SLIDE:
                    telemetry.addData("SLIDE DRIVE", "TELEMETRY");
                    telemetry.addData(">>>North Pwr", frontRight.getPower());
                    telemetry.addData(">>>South Pwr", backLeft.getPower());
                    telemetry.addData(">>>West Pwr", backRight.getPower());
                    telemetry.addData(">>>East Pwr", frontLeft.getPower());
                    break;
                case MECANUM_X:
                    telemetry.addData("MECANUM X DRIVE", "TELEMETRY");
                    telemetry.addData(">>>Front Left Pwr", frontLeft.getPower());
                    telemetry.addData(">>>Front Right Pwr", frontRight.getPower());
                    telemetry.addData(">>>Back Left Pwr", backLeft.getPower());
                    telemetry.addData(">>>Back Right Pwr", backRight.getPower());
                    break;
                case MECANUM_O:
                    telemetry.addData("MECANUM O DRIVE", "TELEMETRY");
                    telemetry.addData(">>>Front Left Pwr", frontLeft.getPower());
                    telemetry.addData(">>>Front Right Pwr", frontRight.getPower());
                    telemetry.addData(">>>Back Left Pwr", backLeft.getPower());
                    telemetry.addData(">>>Back Right Pwr", backRight.getPower());
                    break;
                case HOLONOMIC:
                    telemetry.addData("HOLONOMIC DRIVE", "TELEMETRY");
                    telemetry.addData(">>>Front Left Pwr", frontLeft.getPower());
                    telemetry.addData(">>>Front Right Pwr", frontRight.getPower());
                    telemetry.addData(">>>Back Left Pwr", backLeft.getPower());
                    telemetry.addData(">>>Back Right Pwr", backRight.getPower());
                    break;
            }
            telemetry.addLine();
        }

    }


    //Class variables for Autonomous Use
    private BNO055IMU imu; //For detecting rotation
    private Orientation angles;
    private PID anglePID = new PID();
    private boolean disableEncoderCalibration = false;
    public boolean encoderTargetReached = false;
    public boolean angleTargetReached = false;
    public EncoderMode encoderMode = EncoderMode.CONSTANT_SPEED;
    private long startTime = 0; // in nanoseconds

    //Methods for Autonomous
    public double getRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }
    public void resetStartTime() {
        startTime = System.nanoTime();
    }
    public void initializeIMU() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);
    }
    public void startIMU() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    private void updateAngles() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public float getHeading() {
        updateAngles();
        telemetry.addData("Heading", -angles.firstAngle);
        return -angles.firstAngle;
    }

    public void resetEncoders() {
        if (disableEncoderCalibration) {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            if (numOfMotors == 4) {
                backRight.setPower(0);
                backLeft.setPower(0);
            }
        }
        else {
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (numOfMotors == 4) {
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
    public void runConstantSpeed() {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (numOfMotors == 4) {
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void runConstantPower() {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (numOfMotors == 4) {
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public double getRevolutionsDriven() {
        telemetry.addData("Revolutions Driven", String.format("%.2f", frontRight.getCurrentPosition() / COUNTS_PER_REVOLUTION_40 / gearRatio));
        return frontRight.getCurrentPosition() / COUNTS_PER_REVOLUTION_40 / gearRatio;
    }

    public void move(double pwr_fr, double pwr_fl, double pwr_br, double pwr_bl) {
        frontRight.setPower(pwr_fr);
        frontLeft.setPower(pwr_fl);
        if (numOfMotors == 4) {
            backRight.setPower(pwr_br);
            backLeft.setPower(pwr_bl);
        }
    }
    public void stopDriveMotors() {
        move(0, 0, 0, 0);
    }

    public void moveForward(double power) {
        switch (encoderMode) {
            case CONSTANT_SPEED: runConstantSpeed();
                break;
            case CONSTANT_POWER: runConstantPower();
                break;
        }
        switch (driveMode) {
            case TANK:
            case ARCADE:
            case MECANUM_X:
            case MECANUM_O:
            case HOLONOMIC: move(power, power, power, power);
                break;
            case SLIDE: move(0, power, power, 0);
                break;
        }
    }
    public void moveForward(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION_40 * gearRatio;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION_40;
        double error = target - frontLeft.getCurrentPosition();
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION_40 / 4) {
                speed = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            moveForward(speed);
        }
        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION_40 / gearRatio));
        }
    }
    public void moveRight(double power) {
        switch (encoderMode) {
            case CONSTANT_SPEED: runConstantSpeed();
                break;
            case CONSTANT_POWER: runConstantPower();
                break;
        }
        switch (driveMode) {
            case TANK:
            case ARCADE: telemetry.addData("Error", "Current Drive Mode Can't Move Right");
                break;
            case MECANUM_O: move(power, -power, -power, power);
                break;
            case MECANUM_X:
            case HOLONOMIC: move(-power, power, power, -power);
                break;
            case SLIDE: move(power, 0, 0, power);
                break;
        }
    }
    public void moveRight(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION_40 * gearRatio;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION_40;
        double error;
        if (driveMode != DriveMode.MECANUM_X || driveMode != DriveMode.HOLONOMIC) {
            error = target - frontRight.getCurrentPosition();
        }
        else {
            error = target - frontLeft.getCurrentPosition();
        }
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION_40 / 2) {
                speed = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            moveRight(speed);
        }
        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION_40 / gearRatio));
        }
    }

    public void moveForwardRight(double power) {
        switch (encoderMode) {
            case CONSTANT_SPEED: runConstantSpeed();
                break;
            case CONSTANT_POWER: runConstantPower();
                break;
        }
        switch (driveMode) {
            case TANK:
            case ARCADE: telemetry.addData("Error", "Current Drive Mode Can't Move Diagonally");
                break;
            case MECANUM_O: move(power, 0, 0, power);
                break;
            case MECANUM_X:
            case HOLONOMIC: move(0, power, power, 0);
                break;
            case SLIDE: move(power, power, power, power);
                break;
        }
    }
    public void moveForwardRight(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION_40 * gearRatio;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION_40;
        double error;
        if (driveMode != DriveMode.MECANUM_X || driveMode != DriveMode.HOLONOMIC) {
            error = target - frontRight.getCurrentPosition();
        }
        else {
            error = target - frontLeft.getCurrentPosition();
        }
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION_40 / 2) {
                speed = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            moveForwardRight(speed);
        }
        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION_40 / gearRatio));
        }
    }
    public void moveForwardLeft(double power) {
        switch (encoderMode) {
            case CONSTANT_SPEED: runConstantSpeed();
                break;
            case CONSTANT_POWER: runConstantPower();
                break;
        }
        switch (driveMode) {
            case TANK:
            case ARCADE: telemetry.addData("Error", "Current Drive Mode Can't Move Diagonally");
                break;
            case MECANUM_O: move(0, power, power, 0);
                break;
            case MECANUM_X:
            case HOLONOMIC: move(power, 0, 0, power);
                break;
            case SLIDE: move(-power, power, power, -power);
                break;
        }
    }
    public void moveForwardLeft(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION_40 * gearRatio;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION_40;
        double error;
        if (driveMode == DriveMode.MECANUM_X || driveMode == DriveMode.HOLONOMIC) {
            error = target - frontRight.getCurrentPosition();
        }
        else {
            error = target - frontLeft.getCurrentPosition();
        }
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION_40 / 2) {
                speed = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            moveForwardLeft(speed);
        }
        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION_40 / gearRatio));
        }
    }

    public void turnClockwise(double power) {
        runConstantSpeed();
        switch (driveMode) {
            case TANK:
            case ARCADE:
            case MECANUM_O:
            case MECANUM_X:
            case HOLONOMIC: move(-power, power, -power, power);
                break;
            case SLIDE: move(power, -power, power, -power);
                break;
        }
    }
    public void turnClockwise(int targetAngle) {
        updateAngles();
        telemetry.addData("Heading", String.format("%.0f", getHeading()));
        double k = 0.005; //experimentally found
        double e = targetAngle + angles.firstAngle; //clockwise is negative for firstAngle
        double power = (0.05 * e / Math.abs(e)) + k * e;
        power = Range.clip(power, -1.0, 1.0);
        if (Math.abs(e) >= 2)
            turnClockwise(power);
        else {
            stopDriveMotors();
            angleTargetReached = true;
        }
    }

    public void turnClockwisePID(int targetAngle) {
        updateAngles();
        telemetry.addData("Heading", String.format("%.0f", getHeading()));
        anglePID.setTargetValue(targetAngle);
        anglePID.update(-angles.firstAngle, getRuntime());
        double power = anglePID.adjustmentValue();
        power = Range.clip(power, -1, 1); //ensure power doesn't exceed max speed
        if (Math.abs(anglePID.getError()) >= 3) //3 degree angle slack / uncertainty
            turnClockwise(power);
        else {
            stopDriveMotors();
            anglePID.reset();
            angleTargetReached = true;
        }

    }
}

//This enum allows us to switch between Drive Modes more elegantly than using boolean variables
enum DriveMode {
    TANK, ARCADE, MECANUM_X, MECANUM_O, SLIDE, HOLONOMIC;
}

//This enum allows us to switch between Encoder Modes for Autonomous
enum EncoderMode {
    CONSTANT_SPEED, CONSTANT_POWER;
}
