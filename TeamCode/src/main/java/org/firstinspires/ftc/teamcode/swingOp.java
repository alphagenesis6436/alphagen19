package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by Michael 4/24/19
 */


@TeleOp(name = "swingOp", group = "Default")
//@Disabled
public class swingOp extends OpMode {
    //Declare any motors, servos, and sensors
    DriveTrain driveTrain = new DriveTrain(DriveMode.ARCADE, 2);
    DcMotor ltMotor;
    DcMotor rtMotor;
    DcMotor scMotor;


    //Declare constants pertaining to drive train
    final double DRIVE_PWR_MAX = 0.80;
    double currentLtPwr = 0.0;
    double currentRtPwr = 0.0;

    final double scPwrMax = 0.80;
    double currentScPwr = 0.0;

    @Override
    public void init() {
        //Initialize motors & set direction
        ltMotor = hardwareMap.dcMotor.get("lm");
        ltMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rtMotor = hardwareMap.dcMotor.get("rm");
        rtMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        scMotor = hardwareMap.dcMotor.get("sc");
        scMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize Max/Min power
        currentRtPwr = Range.clip(currentRtPwr,-DRIVE_PWR_MAX,DRIVE_PWR_MAX);
        rtMotor.setPower(currentRtPwr);
        currentLtPwr = Range.clip(currentLtPwr,-DRIVE_PWR_MAX,DRIVE_PWR_MAX);
        ltMotor.setPower(currentLtPwr);

        currentScPwr = Range.clip(currentScPwr,-scPwrMax,scPwrMax);
        scMotor.setPower(currentScPwr);
    }

    @Override
    public void loop() {
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
        initializeDriveTrain();
        initializeScoring();
    }


    void initializeDriveTrain() {
        driveTrain.initialize();
    }

    void initializeScoring(){
        currentScPwr = Range.clip(currentScPwr,-scPwrMax,scPwrMax);
        scMotor.setPower(currentScPwr);
    }


    void telemetry() {
        telemetryDriveTrain();
        telemetryScoring();
    }

    void telemetryDriveTrain() {
        driveTrain.telemetry();
    }
    void telemetryScoring(){
        telemetry.addData("SC Pwr",scMotor.getPower());
    }


    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateDriveTrain();
        updateScoring();
    }


    void updateDriveTrain() {
        driveTrain.update();
    }
    void updateScoring() {
        if (gamepad1.dpad_up) {
            currentScPwr = scPwrMax;
        }
        if (gamepad1.dpad_down) {
            currentScPwr = -scPwrMax;
        }
    }
}


