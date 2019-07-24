package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.DriveTrain;

/**
 * Modified by Alex on 1/3/2019.
 */


@TeleOp(name = "Drive Train Test", group = "Default")
@Disabled
public class DriveTrainTestOp extends OpMode {
    //Declare any motors
    DriveTrain driveTrain = new DriveTrain(DriveMode.TANK, 2);

    //Declare any variables & constants pertaining to drive train
    final double DRIVE_PWR_MAX = 0.80;

    public DriveTrainTestOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        driveTrain.syncOpMode(gamepad1, telemetry, hardwareMap);
        driveTrain.setDrivePwrMax(DRIVE_PWR_MAX);
        driveTrain.setMotors();

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
        //Clip and Initialize Drive Train
        driveTrain.initialize();
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        driveTrain.telemetry();
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateDriveTrain();
    }

    //Controlled by Driver 1
    //step 1: Press left/right bumper to select tank/arcade drive
    void updateDriveTrain() {
        if (gamepad1.left_bumper){
            driveTrain.setDriveMode(DriveMode.TANK);
        }
        else if (gamepad1.right_bumper){
            driveTrain.setDriveMode(DriveMode.ARCADE);
        }
        driveTrain.update();
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
    void calibrateAutoVariables() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}



