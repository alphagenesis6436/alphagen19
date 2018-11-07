package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
    DcMotor FL; //left if forward
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor LA;
    DcMotor intakeLeft;
    DcMotor intakeRight;
    Servo PulleyServo;
    Servo Marker;
    //Declare any variables & constants pertaining to robot mechanism (i.e drive train)
    final double DRIVE_PWR_MAX = 0.80;
    double currentLeftPwr = 0.0;
    double currentRightPwr = 0.0;
    final double LATCH_PWR = 0.80;
    double currentLatchPwr = 0.0;
    final double MAX_INTAKE_PWR = 0.8;
    double leftPwr = 0.0;
    double rightPwr = 0.0;
    double currentPulleySpeed = 0.5;
    final double START_MARK_POS = 0.0;
    double currentMarkPos = START_MARK_POS;
    final double MIN_MARKER_POS = 0;
    final double MAX_MARKER_POS = 0.75;
    final double MAX_PULLEY_SPEED = (1.00) / 2;
    DriveMode driveMode = DriveMode.TANKDRIVE;

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
        intakeLeft = hardwareMap.dcMotor.get("il");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight = hardwareMap.dcMotor.get("ir");
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //Initialize Servos
        PulleyServo = hardwareMap.servo.get("ps");
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
        updateDrawBridgeIntake();
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
        currentLatchPwr = 0.0;
        if(gamepad2.dpad_up) {
            currentLatchPwr = LATCH_PWR;
        }
        else if (gamepad2.dpad_down) {
            currentLatchPwr = -LATCH_PWR;
        }
    }
    //Controlled by Driver 2
    //step 1: Push Left Stick up or down to move intake.
    void updateDrawBridgeIntake() {
        leftPwr = -gamepad2.left_stick_y*MAX_INTAKE_PWR;
        rightPwr = -gamepad2.left_stick_y*MAX_INTAKE_PWR;
        currentPulleySpeed = 0.5;
        if(gamepad2.dpad_right) {
            currentPulleySpeed = MAX_PULLEY_SPEED;
        }
        else if (gamepad2.dpad_left) {
            currentPulleySpeed = -MAX_PULLEY_SPEED;
        }
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
        leftPwr = Range.clip(leftPwr, -MAX_INTAKE_PWR, MAX_INTAKE_PWR);
        intakeLeft.setPower(leftPwr);
        rightPwr = Range.clip(rightPwr, -MAX_INTAKE_PWR,MAX_INTAKE_PWR);
        intakeRight.setPower(rightPwr);

    }
    void telemetry() {
        //Show Data for Drive Train
        telemetry.addData("Left Drive Pwr", FL.getPower());
        telemetry.addData("Right Drive Pwr", BR.getPower());
        telemetry.addData("Latch Pwr", LA.getPower());
        telemetry.addData("Left Intake Pwr", intakeLeft.getPower());
        telemetry.addData("Right Intake Pwr", intakeRight.getPower());

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
