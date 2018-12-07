package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Brooklyn on 11/11/2017.
 */


@TeleOp(name = "VuforiaTestOp", group = "Default")
@Disabled
public class APIKey extends OpMode {
        //Declare any motors, servos, and sensors
        public static String apiKey = "Abq58BL/////AAAAGasgZWQZgEa8kdxDd4q3veYICfWGwtOkrJnksc53QgcFLFCK6+mXfyuabs/fykXC9lIgnHHgICk13d1UACoFXxpMB7xB6w9jfa/UnjVwIh6es1wlVs3E0TgBubyUOJ/tgBMVwzg9J9OY7vKsgp8K7kMYzGECtK2XtVgYbjQygytVT93x268X5UKIAMpvbEzIa2o6Dko0j+ERWyls3Y+wuhuovQkbPzWcaV88DSQGLdFqQd70uMdmg+cAsPvfb5uU+K9AuXWc3vM6pbaSfpGn4sZ3zU20MzaksR/LXAflL9iTa/6P+VJDmIl/0HRPVh2N2PqkEEeDGq4iKcQ2AwX5tY8U7K+hzYDZsdYNG5fTyGlh";

        //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)


        public APIKey() {}

        @Override public void init() {
            //Initialize motors & set direction

            //Initialize servos

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

        }

        void initialization() {
            //Clip and Initialize Specific Robot Mechanisms

        }
        void telemetry() {
            //Show Data for Specific Robot Mechanisms

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
