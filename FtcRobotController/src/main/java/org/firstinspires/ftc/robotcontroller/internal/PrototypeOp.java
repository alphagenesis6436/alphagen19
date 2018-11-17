package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Alex on 6/1/2017.
 */

@TeleOp(name = "PrototypeOp", group = "Default")
@Disabled
public class PrototypeOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor armMotor;
    Servo clawArm; //180

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)

    final double CLAW_ARM_START_POS = 0.5;
    final double MAX_CLAW_SPEED = (1.00) * 0.5;
    double clawArmPosition = CLAW_ARM_START_POS;
    final double CLAW_MAX = 1.0;
    final double CLAW_MIN = 0.0;
    final double DRIVE_PWR_MAX = 0.7;
    double currentArmPwr = 0.0;




    public PrototypeOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        armMotor = hardwareMap.dcMotor.get("arm");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
        updateArm();


    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        clawArmPosition = Range.clip(clawArmPosition,CLAW_MIN,CLAW_MAX);
        clawArm.setPosition(clawArmPosition);
        currentArmPwr = Range.clip(currentArmPwr,-DRIVE_PWR_MAX,DRIVE_PWR_MAX);
        armMotor.setPower(currentArmPwr);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms

        telemetry.addData("Claw Pos",clawArm.getPosition());
        telemetry.addData("ARM Pwr",armMotor.getPower());

    }

    //Create Methods that will update the driver data

    /*
        All update methods should be commented with:
            //Controlled by Driver (1 or 2)
            //Step 1:  Press the up/down button to move the clawservo towards 180/0
            //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
            //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
     */
    void updateClaw(){
        clawArmPosition = -gamepad2.left_stick_y * MAX_CLAW_SPEED + CLAW_ARM_START_POS;
    }
    void updateArm(){
        currentArmPwr = -gamepad2.right_stick_y * DRIVE_PWR_MAX;
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
    void updateDriveTrain() {

    }
    void resetSensors() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}

