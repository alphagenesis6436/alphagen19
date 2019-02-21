package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;

/**
 * Created by Ben on 11/5/2018.
 */

@TeleOp(name = "LinearslideOp", group = "Default")
@Disabled
public class LinearslideOp extends OpMode {
    //Declare any motors, servos, or sensors
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor LA;
    //PrototypeOp
    DcMotor intakeLeft;
    DcMotor intakeRight;
    DcMotor armMotor;
    DcMotor bodyMotor;
    Servo clawArm; //180

    Servo LatchServo;
    Servo Marker;
    BNO055IMU imu; //For detecting rotation
    Orientation angles;

    //Declare any variables & constants pertaining to Drive Train
    final double DRIVE_PWR_MAX = 0.80;
    final int COUNTS_PER_REVOLUTION = 1120; //AndyMark Motors
    final double DRIVE_GEAR_RATIO = 1.0 / 1.0; //Driven / Driver
    double currentLeftPwr = 0.0;
    double currentRightPwr = 0.0;
    DriveMode driveMode = DriveMode.TANK;

    //Declare any variables & constants pertaining to PrototypeOp
    final double CLAW_ARM_START_POS = 0.5;
    final double MAX_CLAW_SPEED = (1.00) * 0.5;
    double clawArmPosition = CLAW_ARM_START_POS;
    final double CLAW_MAX = 1.0;
    final double CLAW_MIN = 0.0;
    final double ARM_PWR_MAX = 0.7;
    final double BODY_PWR_MAX = 0.8;
    double currentBodyPwr = 0.0;
    double currentArmPwr = 0.0;
    //Declare any variables & constants pertaining to Latch System
    final double LATCH_PWR = 0.80;
    double currentLatchPwr = 0.0;
    final double MAX_LATCH_SPEED = (1.00) / 2;
    double currentLatchSpeed = 0.5;

    //Declare any variables & constants pertaining to Marker
    final double START_MARK_POS = 1.0;
    double currentMarkPos = START_MARK_POS;
    final double MIN_MARKER_POS = 0.25; //drop pos
    final double MAX_MARKER_POS = 1.00; //start pos


    public LinearslideOp() {
    }

    @Override
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void init() {
        //Initialize motors & set direction
        FL = hardwareMap.dcMotor.get("fl");
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL = hardwareMap.dcMotor.get("bl");
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR = hardwareMap.dcMotor.get("fr");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR = hardwareMap.dcMotor.get("br");
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        LA = hardwareMap.dcMotor.get("la");
        LA.setDirection(DcMotorSimple.Direction.FORWARD);
        //intakeLeft = hardwareMap.dcMotor.get("il");
        //intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //intakeRight = hardwareMap.dcMotor.get("ir");
        //intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //Initialize Servos
        LatchServo = hardwareMap.servo.get("ls");
        Marker = hardwareMap.servo.get("mk");
        //PrototypeOp
        armMotor = hardwareMap.dcMotor.get("arm");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bodyMotor = hardwareMap.dcMotor.get("body");
        bodyMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //Initialize servos PrototypeOp
        clawArm = hardwareMap.servo.get("ca");
        //Initialize Sensors
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        telemetry.addData(">", "Press Start to continue");
        telemetry();
    }

    @Override
    public void loop() {
        //Update all the data based on driver input
        updateData();

        /* Clip Variables to make sure they don't exceed their
         * ranged values and Set them to the Motors/Servos */
        initialization();

        //Show the Real Values of the Data Using Telemetry
        telemetry();
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateDriveTrain();
        updateLatch();
        //updateDrawBridgeIntake();
        updateMarker();
        updateClaw();
        updateArm();
        updateBody();

    }

    //Controlled by Driver 1
    //step 1: Push up/down the left/right stick to control the left/right drive motors
    void updateTankDrive() {
        currentLeftPwr = -gamepad1.left_stick_y * DRIVE_PWR_MAX;
        currentRightPwr = -gamepad1.right_stick_y * DRIVE_PWR_MAX;
    }

    //Controlled by Driver 1
    //step 1: Push Left Stick up or down to move robot forward or backward. Move Right Stick Left or Right
    void updateArcadeDrive() {
        currentLeftPwr = (-gamepad1.left_stick_y + gamepad1.right_stick_x) * DRIVE_PWR_MAX;
        currentRightPwr = (-gamepad1.left_stick_y - gamepad1.right_stick_x) * DRIVE_PWR_MAX;
    }

    //Controlled by Driver 1
    //step 1: Press left/right bumper to select tank/arcade drive
    void updateDriveTrain() {
        if (gamepad1.left_bumper) {
            driveMode = DriveMode.TANK;
        } else if (gamepad1.right_bumper) {
            driveMode = DriveMode.ARCADE;
        }

        switch (driveMode) {
            case TANK:
                updateTankDrive();
                break;
            case ARCADE:
                updateArcadeDrive();
                break;
        }
    }

    //Controlled by Driver 2
    //step 1: Press DPAD up or down to move latch up or down.
    void updateLatch() {
        currentLatchPwr = -gamepad2.right_stick_y * LATCH_PWR;
        currentLatchSpeed = 0.5;
        if (gamepad2.dpad_right) {
            currentLatchSpeed = 0.5 + MAX_LATCH_SPEED;
        } else if (gamepad2.dpad_left) {
            currentLatchSpeed = 0.5 - MAX_LATCH_SPEED;
        }
    }

    //Controlled by Driver 2
    //step 1: Push Left Stick up or down to move intake.
    //Controlled by Driver 2
    //Press A to move marker arm up, Press Y to move marker arm down
    void updateMarker() {
        if (gamepad2.a) {
            currentMarkPos = MIN_MARKER_POS;
        } else if (gamepad2.y) {
            currentMarkPos = MAX_MARKER_POS;
        }
    }


    void initialization() {
        //Clip and Initialize Robot Mechanisms
        currentLeftPwr = Range.clip(currentLeftPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        FL.setPower(currentLeftPwr);
        BL.setPower(currentLeftPwr);
        currentRightPwr = Range.clip(currentRightPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        FR.setPower(currentRightPwr);
        BR.setPower(currentRightPwr);
        currentLatchPwr = Range.clip(currentLatchPwr, -LATCH_PWR, LATCH_PWR);
        LA.setPower(currentLatchPwr);
        currentMarkPos = Range.clip(currentMarkPos, MIN_MARKER_POS, MAX_MARKER_POS);
        Marker.setPosition(currentMarkPos);
        currentLatchSpeed = Range.clip(currentLatchSpeed, 0.5 - MAX_LATCH_SPEED, 0.5 + MAX_LATCH_SPEED);
        LatchServo.setPosition(currentLatchSpeed);
        //leftIntakePwr = Range.clip(leftIntakePwr, -MAX_INTAKE_PWR, MAX_INTAKE_PWR);
        //intakeLeft.setPower(leftIntakePwr);
        //rightIntakePwr = Range.clip(rightIntakePwr, -MAX_INTAKE_PWR,MAX_INTAKE_PWR);
        //intakeRight.setPower(rightIntakePwr);
        clawArmPosition = Range.clip(clawArmPosition, CLAW_MIN, CLAW_MAX);
        clawArm.setPosition(clawArmPosition);
        currentArmPwr = Range.clip(currentArmPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        armMotor.setPower(currentArmPwr);
        currentBodyPwr = Range.clip(currentBodyPwr, -BODY_PWR_MAX, BODY_PWR_MAX);
        bodyMotor.setPower(currentBodyPwr);
        //PrototypeOp above

    }

    void telemetry() {
        //Show Data for Drive Train
        telemetry.addData("Left Drive Pwr", FL.getPower());
        telemetry.addData("Right Drive Pwr", BR.getPower());
        telemetry.addData("Latch Pwr", LA.getPower());
        telemetry.addData("Latch Servo", LatchServo.getPosition());
        telemetry.addData("Marker Pos", Marker.getPosition());
        //telemetry.addData("Left Intake Pwr", intakeLeft.getPower());
        //telemetry.addData("Right Intake Pwr", intakeRight.getPower());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("IMU Heading", String.format("%.0f", angles.firstAngle));
        //prototypeOp below
        telemetry.addData("Claw Pos", clawArm.getPosition());
        telemetry.addData("ARM Pwr", armMotor.getPower());
        telemetry.addData("Body Pwr", armMotor.getPower());

    }

    //Create Methods that will update the driver data

 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
  */


    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous
    boolean disableEncoderCalibration = false;
    boolean encoderTargetReached = false;
    boolean angleTargetReached = false;
    EncoderMode encoderMode = EncoderMode.CONSTANT_SPEED;


    void resetEncoders() {
        if (disableEncoderCalibration) {
            FR.setPower(0);
            FL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        } else {
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    void runConstantSpeed() {
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void runConstantPower() {
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void move(double pwr_fr, double pwr_fl, double pwr_br, double pwr_bl) {
        FR.setPower(pwr_fr);
        FL.setPower(pwr_fl);
        BR.setPower(pwr_br);
        BL.setPower(pwr_bl);
    }

    void stopDriveMotors() {
        move(0, 0, 0, 0);
    }

    void moveForward(double power) {
        switch (encoderMode) {
            case CONSTANT_SPEED:
                runConstantSpeed();
                break;
            case CONSTANT_POWER:
                runConstantPower();
                break;
        }
        move(power, power, power, power);
    }

    void moveForward(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION * DRIVE_GEAR_RATIO;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION;
        double error = target - FR.getCurrentPosition();
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION / 2) {
                speed = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            moveForward(speed);
        }
        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        } else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION / DRIVE_GEAR_RATIO));
        }
    }

    void turnClockwise(double power) {
        runConstantSpeed();
        move(-power, power, -power, power);
    }

    void turnClockwise(int targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", String.format("%.0f", angles.firstAngle));
        double k = 0.005; //experimentally found
        double e = targetAngle + angles.firstAngle; //clockwise is negative for thirdAngle
        double power = (0.05 * e / Math.abs(e)) + k * e;
        power = Range.clip(power, -1.0, 1.0);
        if (Math.abs(e) >= 2)
            turnClockwise(power);
        else {
            stopDriveMotors();
            angleTargetReached = true;
        }
    }

    void updateClaw() {
        if (gamepad2.dpad_up) {
            clawArmPosition = CLAW_ARM_START_POS + CLAW_MAX;
        }

        if (gamepad2.dpad_down) {
            clawArmPosition = CLAW_ARM_START_POS - CLAW_MAX;
        }
    }


    void updateArm(){
        currentArmPwr = -gamepad2.right_stick_y * ARM_PWR_MAX;
    }
    void updateBody(){
        currentBodyPwr = -gamepad2.left_stick_y * BODY_PWR_MAX;
    }
    void turnClockwisePID(int targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", String.format("%.0f", angles.firstAngle));
        if (true) {
            double kp = 0.010; //proportionality constant (amount to adjust for immediate deviance) experimentally found
            double ki = 0.001; //integral constant (amount to adjust for past errors) experimentally found
            double kd = 0.0022; //derivative constant (amount to adjust for future errors) experimentally found
            double e = targetAngle + angles.firstAngle; //error
            e_list.add(e);
            t_list.add(this.time);
            double power = kp*e + ki*integrate() + kd*differentiate();
            power = Range.clip(power, -DRIVE_PWR_MAX, DRIVE_PWR_MAX); //ensure power doesn't exceed max speed
            if (Math.abs(e) >= 5) //5 degree angle slack / uncertainty
                turnClockwise(power);
            else {
                stopDriveMotors();
                e_list.clear();
                t_list.clear();
                angleTargetReached = true;
            }
        }
        else {
            double k = 3.5; //experimentally found
            double power = k * (targetAngle + angles.firstAngle)
                    / Math.abs(targetAngle);
            if (Math.abs(targetAngle + angles.firstAngle) >= 10)
                turnClockwise(power);
            else {
                stopDriveMotors();
                angleTargetReached = true;
            }
        }
    }
    ArrayList<Double> e_list = new ArrayList<>(); //records past errors
    ArrayList<Double> t_list = new ArrayList<>(); // records times past errors took place
    //integrates error of angle w/ respect to time
    double integrate() {
        double sum = 0; //uses trapezoidal sum approximation method
        if (e_list.size() >= 2) {
            for (int i = 0; i <= e_list.size() - 2; i++) {
                double dt = t_list.get(i+1) - t_list.get(i);
                sum += (e_list.get(i+1) + e_list.get(i))*dt / 2.0;
            }
        }
        return sum;
    }
    //differentiates error of angle w/ respect to time
    double differentiate() {
        double slope = 0; //uses secant line approximation
        if (e_list.size() >= 2) {
            double de = e_list.get(e_list.size() - 1) - e_list.get(e_list.size() - 2);
            double dt = t_list.get(t_list.size() - 1) - t_list.get(t_list.size() - 2);
            slope = de/dt;
        }
        return slope;
    }


    void calibrateAutoVariables() {
        encoderTargetReached = false;
        angleTargetReached = false;
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

    //RICKY WAS HERE LMAO XD HAHAHAHAHAHA

}
