package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Kimberly on 8/29/2018.
 * This program is for the red ranger bot
 */

@TeleOp(name = "KimberlyOp", group = "Default")
//@Disabled
public class KimberlyOp extends OpMode {
    //Declare any motors, servos, and sensors
    Servo clawArm; //180

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final double CLAW_ARM_START_POS = 0.5;
    double clawArmPosition= CLAW_ARM_START_POS;
    double clawDelta = 0.001;
    final double CLAW_MIN = 0.0;
    final double CLAW_MAX = 1.0;

    public KimberlyOp() {}

    @Override public void init() {
        //Initialize motors & set direction

        //Initialize servos
        clawArm = hardwareMap.servo.get("ca");
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
        updateClaw();

    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms

        clawArmPosition = Range.clip(clawArmPosition,CLAW_MIN,CLAW_MAX);
        clawArm.setPosition(clawArmPosition);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms

        telemetry.addData("Claw Pos", clawArm.getPosition());
    }

    //Create Methods that will update the driver data

 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
  */

    //driver 1
    //step 1: press the up/down button to move claw towards 180/0
    void updateClaw () {
        if (gamepad1.dpad_up) {
            clawArmPosition+=clawDelta;
        }
        else if (gamepad1.dpad_down){
            clawArmPosition-=clawDelta;
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

