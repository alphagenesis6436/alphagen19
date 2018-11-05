package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Alex on 9/22/2018.
 * This program is for the DrawBridge Intake Prototype
 * The prototype was built by Alex, Michelle, and Sabrina
 * The prototype uses two rev hex motors for a wheel intake
 */

@TeleOp(name = "DrawBridgeIntakeOp", group = "Default")
//@Disabled
public class DrawBridgeIntakeOp extends OpMode {
    //Declare any motors, servos, and sensors
    //Rev Motors used
    DcMotor intakeLeft;
    DcMotor intakeRight;

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final double MAX_INTAKE_PWR = 0.8;
    double leftPwr = 0.0;
    double rightPwr = 0.0;


    public DrawBridgeIntakeOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        intakeLeft = hardwareMap.dcMotor.get("il");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight = hardwareMap.dcMotor.get("ir");
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);

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
        updateDrawBridgeIntake();
    }

    void updateDrawBridgeIntake() {
        leftPwr = -gamepad1.left_stick_y*MAX_INTAKE_PWR;
        rightPwr = -gamepad1.left_stick_y*MAX_INTAKE_PWR;
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        leftPwr = Range.clip(leftPwr, -MAX_INTAKE_PWR, MAX_INTAKE_PWR);
        intakeLeft.setPower(leftPwr);
        rightPwr = Range.clip(rightPwr, -MAX_INTAKE_PWR,MAX_INTAKE_PWR);
        intakeRight.setPower(rightPwr);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
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
    void runConstantSpeed() {

    }
    void runConstantPower() {

    }
    void resetSensors() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}