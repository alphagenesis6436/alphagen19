package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "ArcadeOp", group = "Default")
//@Disabled
public class ArcadeOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor FR;
    DcMotor FL;



    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final double DRIVE_PWR_MAX = 0.8;
    double currentLeftPwr = 0;
    double currentRightPwr = 0;


    public ArcadeOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        FR = hardwareMap.dcMotor.get("fr");
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL = hardwareMap.dcMotor.get("fl");
        FL.setDirection(DcMotorSimple.Direction.FORWARD);


        //Initialize servos

        //Initialize sensors

        telemetry.addData(">", "Press Start to continue");
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

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        currentLeftPwr = Range.clip(currentLeftPwr,-DRIVE_PWR_MAX,DRIVE_PWR_MAX);
        FL.setPower(currentLeftPwr);
        currentRightPwr = Range.clip(currentRightPwr,-DRIVE_PWR_MAX,DRIVE_PWR_MAX);
        FR.setPower(currentRightPwr);

    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("FR_PWR",FR.getPower());
        telemetry.addData("FL_PWR",FL.getPower());

    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateArcadeDrive();

    }

    //Create Methods that will update the driver data
    void updateArcadeDrive() {
        currentLeftPwr = (-gamepad1.left_stick_y + gamepad1.right_stick_x) * DRIVE_PWR_MAX;
        currentRightPwr = (-gamepad1.left_stick_y - gamepad1.right_stick_x) * DRIVE_PWR_MAX;
    }
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
    String stateGoal = ""; //Overwrite this as the specific step used in Autonomous

    void advanceState() {
        state++;
        setTime = this.time;
    }

    void advanceState(int skipState) {
        state += (skipState * 2) + 1;
        setTime = this.time;
    }

    void resetEncoders() {

    }
    void runConstantSpeed() {

    }
    void runConstantPower() {

    }
    void calibrateAutoVariables() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}



