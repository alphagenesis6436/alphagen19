package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ben on 11/5/2018.
 */

@TeleOp(name = "DeathBoxOp", group = "Default")
//@Disabled
public class DeathBoxOp extends OpMode {
    //Declare any motors, servos, or sensors
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor LA;
    DcMotor intakeLeft;
    DcMotor intakeRight;
    Servo LatchServo;
    Servo Marker;

    //Declare any variables & constants pertaining to Drive Train
    final double DRIVE_PWR_MAX = 0.80;
    double currentLeftPwr = 0.0;
    double currentRightPwr = 0.0;
    DriveMode driveMode = DriveMode.TANKDRIVE;

    //Declare any variables & constants pertaining to Intake System
    final double MAX_INTAKE_PWR = 0.8;
    double leftIntakePwr = 0.0;
    double rightIntakePwr = 0.0;

    //Declare any variables & constants pertaining to Latch System
    final double LATCH_PWR = 0.80;
    double currentLatchPwr = 0.0;
    final double MAX_LATCH_SPEED = (1.00) / 2;
    double currentLatchSpeed = 0.5;

    //Declare any variables & constants pertaining to Marker
    final double START_MARK_POS = 0.0;
    double currentMarkPos = START_MARK_POS;
    final double MIN_MARKER_POS = 0;
    final double MAX_MARKER_POS = 0.75;


    public DeathBoxOp() {}

    @Override public void init() {
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

        telemetry();
    }
    @Override public void loop() {
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
        if (gamepad1.left_bumper){
            driveMode = DriveMode.TANKDRIVE;
        }
        else if (gamepad1.right_bumper){
            driveMode = DriveMode.ARCADEDRIVE;
        }

        switch (driveMode) {
            case TANKDRIVE: updateTankDrive();
                break;
            case ARCADEDRIVE: updateArcadeDrive();
                break;
        }
    }
    //Controlled by Driver 2
    //step 1: Press DPAD up or down to move latch up or down.
    void updateLatch() {
        currentLatchPwr = -gamepad2.right_stick_y * LATCH_PWR;
        currentLatchSpeed = 0.5;
        if(gamepad2.dpad_right) {
            currentLatchSpeed = MAX_LATCH_SPEED;
        }
        else if (gamepad2.dpad_left) {
            currentLatchSpeed = -MAX_LATCH_SPEED;
        }
    }
    //Controlled by Driver 2
    //step 1: Push Left Stick up or down to move intake.
    void updateDrawBridgeIntake() {
        leftIntakePwr = -gamepad2.left_stick_y*MAX_INTAKE_PWR;
        rightIntakePwr = -gamepad2.left_stick_y*MAX_INTAKE_PWR;
    }
    //Controlled by Driver 2
    //Press A to move marker arm up, Press Y to move marker arm down
    void updateMarker() {
        if(gamepad2.a) {
            currentMarkPos = MIN_MARKER_POS;
        }
        else if (gamepad2.y) {
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

    void resetEncoders() {

    }
    void runEncoders() {

    }
    void runWithoutEncoders() {

    }
    void resetSensors() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}
