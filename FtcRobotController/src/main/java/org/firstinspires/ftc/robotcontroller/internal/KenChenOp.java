package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Alex on 6/1/2017.
 */

@TeleOp(name = "OpMode Template", group = "Default")
@Disabled
public class KenChenOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor leftMotor;
    DcMotor rightMotor;

    Servo clawArm;
    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    double leftPower = 0.0;
    double rightPower = 0.0;
    double MaxPower = 0.8;

    double CLAW_ARM_START_POS = 0.2;
    double clawArmPosition= CLAW_ARM_START_POS;
    double clawDelta = 0.2;



    public KenChenOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        leftMotor = hardwareMap.get(DcMotor.class, "lm");
        rightMotor = hardwareMap.get(DcMotor.class, "rm");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //Initialize servos
        //clawArm.setPosition(); //if this is a 180 degree servo, you can only input from 0.0 - 1.0
        //clawArm.setPosition(); //If this is a 360 degree servo, you can input from
        //Initialize sensors

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
        leftPower = -gamepad1.left_stick_y * MaxPower;
        rightPower = -gamepad1.right_stick_y * MaxPower;

    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        leftPower = Range.clip(leftPower, -MaxPower, MaxPower);
        rightPower = Range.clip(rightPower, -MaxPower, MaxPower);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("rightPower", rightPower);
        telemetry.addData("leftPower", leftPower);

    }

    //Create Methods that will update the driver data
 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
  */

    void updateClaw() {
        //Driver 1
        //Step 1 Press the up / down button to move the clawservo towards 180 / 0 deg.
        if (gamepad1.dpad_up) {
            clawArmPosition += clawDelta; //how much distance we are going to increase it per click of button

        }
        else if(gamepad1.dpad_down){
            clawArmPosition = clawDelta;
        }

    }


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

