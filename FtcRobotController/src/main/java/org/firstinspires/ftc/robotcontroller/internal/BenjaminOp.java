package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Benji on 8/29/18.
 * rEd RoBot
 */

@TeleOp(name = "BenjaminOp", group = "Default")
//@Disabled
public class BenjaminOp extends OpMode {
    //Declare any motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo clawArm; //180


    //Declare any variables & constants pertaining to drive train
    double maxPwr=0.8;
    double leftPwr=0.0;
    double rightPwr=0.0;
    final double CLAW_ARM_START_POS = 0.5;
    double clawArmPosition = CLAW_ARM_START_POS;
    double clawDelta = 0.0001;
    final double CLAW_MIN = 0.0;
    final double CLAW_MAX = 1.0;
    public BenjaminOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor=hardwareMap.get(DcMotor.class,"lm");
        rightMotor=hardwareMap.get(DcMotor.class,"rm");
        //Initialize Servos
        clawArm = hardwareMap.servo.get("ca");



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
        updateClaw();
        updateDriveTrain();
    }

    void initialization() {
        //Clip and Initialize Drive Train
        leftPwr = Range.clip(leftPwr, -maxPwr, maxPwr);
        rightPwr = Range.clip(rightPwr, -maxPwr, maxPwr);
        leftMotor.setPower(leftPwr);
        rightMotor.setPower(rightPwr);
        clawArmPosition = Range.clip(clawArmPosition,CLAW_MIN,CLAW_MAX);
        clawArm.setPosition(clawArmPosition);
    }
    void telemetry() {
        //Show Data for Drive Train
        telemetry.addData("leftMotor",leftPwr);
        telemetry.addData("rightMotor",rightPwr);
        telemetry.addData("Claw Pos",clawArm.getPosition());

    }

    //Create Methods that will update the driver data

 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         void.updateClaw()
  */
    //driver1 controller
    //Step 1: Press the Up/Down Button to move the claw servo towards 180/0
    //Step 2:

    //Controlled by Driver 1
    //step 1: Push up/down the left/right stick to control the left/right drive motors
    void updateDriveTrain() {
        leftPwr=-gamepad1.left_stick_y*maxPwr;
        rightPwr=-gamepad1.right_stick_y*maxPwr;
    }


    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous

    void updateClaw() {
        if (gamepad1.dpad_up) {
            clawArmPosition += clawDelta;
        }
        else if (gamepad1.dpad_down) {
            clawArmPosition -= clawDelta;
        }
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


