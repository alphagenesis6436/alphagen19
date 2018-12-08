package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Updated by Alex on 12/7/2018.
 */

@TeleOp(name = "VuforiaRangerOp", group = "Default")
//@Disabled
public class VuforiaRangerOp extends OpMode {
    //Declare any motors
    DcMotor leftDrive;
    DcMotor rightDrive;
    //Declare Sensors
    BNO055IMU imu; //For detecting rotation
    Orientation angles;

    //Declare any variables & constants pertaining to drive train
    final double DRIVE_PWR_MAX = 0.80;
    double currentLeftPwr = 0.0;
    double currentRightPwr = 0.0;
    final int COUNTS_PER_REVOLUTION = 1120; //AndyMark Motors
    final double DRIVE_GEAR_RATIO = 1.0 / 1.0; //Driven / Driver


    //Vuforia Stuff
    //Elapsed time and measurement constants
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    //Vuforia variables
    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    //Detector object
    GoldAlignDetector detector;

    public VuforiaRangerOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        leftDrive = hardwareMap.dcMotor.get("ld");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive = hardwareMap.dcMotor.get("rd");
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData(">", "Drive Train Initialization Successful");

        //Initialize Sensors
        initializeIMU();
        telemetry.addData(">", "IMU Initialization Successful");
        initializeDogeforia();
        telemetry.addData(">", "Vuforia Initialization Successful");

        telemetry.addData(">", "Press Start to continue");
    }
    @Override public void start() {
        runtime.reset(); //Reset timer
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
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
    @Override public void stop() {
        vuforia.stop();
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateDriveTrain();
        updateAngles();
    }

    void updateAngles() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    void initialization() {
        //Clip and Initialize Drive Train
        currentLeftPwr = Range.clip(currentLeftPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        leftDrive.setPower(currentLeftPwr);
        currentRightPwr = Range.clip(currentRightPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        rightDrive.setPower(currentRightPwr);
    }
    void telemetry() {
        //Show Data for Drive Train
        telemetry.addData("DRIVE TRAIN", "TELEMETRY");
        telemetry.addData(">>>Left Drive Pwr", leftDrive.getPower());
        telemetry.addData(">>>Right Drive Pwr", rightDrive.getPower());
        telemetry.addData("IMU", "TELEMETRY");
        telemetry.addData(">>>First Angle", angles.firstAngle);
        telemetry.addData(">>>Second Angle", angles.secondAngle);
        telemetry.addData(">>>Third Angle", angles.thirdAngle);
        telemetry.addData("VUFORIA", "TELEMETRY");
        telemetry.addData(">>>Gold Mineral X pos", detector.getXPosition());
    }

    //Create Methods that will update the driver data

 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
  */

    //Controlled by Driver 1
    //step 1: Push up/down the left/right stick to control the left/right drive motors
    void updateDriveTrain() {
        currentLeftPwr = -gamepad1.left_stick_y * DRIVE_PWR_MAX;
        currentRightPwr = -gamepad1.right_stick_y * DRIVE_PWR_MAX;
    }


    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous

    boolean disableEncoderCalibration = false;
    boolean encoderTargetReached = false;
    boolean angleTargetReached = false;
    EncoderMode encoderMode = EncoderMode.CONSTANT_SPEED;


    void resetEncoders() {
        if (disableEncoderCalibration) {
            rightDrive.setPower(0);
            leftDrive.setPower(0);
        }
        else {
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    void runConstantSpeed() {
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void runConstantPower() {
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void move(double pwr_fr, double pwr_fl) {
        rightDrive.setPower(pwr_fr);
        leftDrive.setPower(pwr_fl);
    }
    void stopDriveMotors() {
        move(0, 0);
    }

    void moveForward(double power) {
        switch (encoderMode) {
            case CONSTANT_SPEED: runConstantSpeed();
                break;
            case CONSTANT_POWER: runConstantPower();
                break;
        }
        move(power, power);
    }

    void moveForward(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION * DRIVE_GEAR_RATIO;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION;
        double error = target - rightDrive.getCurrentPosition();
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION / 2) {
                speed = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            moveForward(speed);
        }
        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION / DRIVE_GEAR_RATIO));
        }
    }

    float getHeading() {
        updateAngles();
        telemetry.addData("Heading", -angles.firstAngle);
        return -angles.firstAngle;
    }
    void turnClockwise(double power) {
        runConstantSpeed();
        move(-power, power);
    }
    void turnClockwise(int targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", String.format("%.0f", angles.firstAngle));
        double k = 0.005; //experimentally found
        double e = targetAngle + angles.firstAngle; //clockwise is negative for thirdAngle
        double power = (0.05 * e / Math.abs(e)) + k * e;
        power = Range.clip(power, -1.0, 1.0);
        if (Math.abs(e) >= 2)
            turnClockwise(power);
        else {
            stopDriveMotors();
            angleTargetReached = true;
        }
    }

    void turnClockwisePID(int targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", String.format("%.0f", angles.firstAngle));
        if (true) {
            double kp = 0.010; //proportionality constant (amount to adjust for immediate deviance) experimentally found
            double ki = 0.001; //integral constant (amount to adjust for past errors) experimentally found
            double kd = 0.0022; //derivative constant (amount to adjust for future errors) experimentally found
            double e = targetAngle + angles.firstAngle; //error
            e_list.add(e);
            t_list.add(this.time);
            double power = kp*e + ki*integrate() + kd*differentiate();
            power = Range.clip(power, -DRIVE_PWR_MAX, DRIVE_PWR_MAX); //ensure power doesn't exceed max speed
            if (Math.abs(e) >= 5) //5 degree angle slack / uncertainty
                turnClockwise(power);
            else {
                stopDriveMotors();
                e_list.clear();
                t_list.clear();
                angleTargetReached = true;
            }
        }
        else {
            double k = 3.5; //experimentally found
            double power = k * (targetAngle + angles.firstAngle)
                    / Math.abs(targetAngle);
            if (Math.abs(targetAngle + angles.firstAngle) >= 10)
                turnClockwise(power);
            else {
                stopDriveMotors();
                angleTargetReached = true;
            }
        }
    }
    ArrayList<Double> e_list = new ArrayList<>(); //records past errors
    ArrayList<Double> t_list = new ArrayList<>(); // records times past errors took place
    //integrates error of angle w/ respect to time
    double integrate() {
        double sum = 0; //uses trapezoidal sum approximation method
        if (e_list.size() >= 2) {
            for (int i = 0; i <= e_list.size() - 2; i++) {
                double dt = t_list.get(i+1) - t_list.get(i);
                sum += (e_list.get(i+1) + e_list.get(i))*dt / 2.0;
            }
        }
        return sum;
    }
    //differentiates error of angle w/ respect to time
    double differentiate() {
        double slope = 0; //uses secant line approximation
        if (e_list.size() >= 2) {
            double de = e_list.get(e_list.size() - 1) - e_list.get(e_list.size() - 2);
            double dt = t_list.get(t_list.size() - 1) - t_list.get(t_list.size() - 2);
            slope = de/dt;
        }
        return slope;
    }


    void calibrateAutoVariables() {
        encoderTargetReached = false;
        angleTargetReached = false;
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

    void initializeIMU() {
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);
    }
    void initializeDogeforia() {
        // Setup camera and Vuforia parameters
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Set Vuforia parameters
        parameters.vuforiaLicenseKey = APIKey.apiKey;
        parameters.fillCameraMonitorViewParent = true;

        // Init Dogeforia
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        // Set target names
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        // Set trackables' location on field
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        //Set camera displacement
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        // Set phone location on robot
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        //Set info for the trackables
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        //Activate targets
        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector(); // Create a gold aligndetector
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);

        detector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100); // Create new filter
        detector.useDefaults(); // Use default settings
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // Uncomment if using PERFECT_AREA scoring

        //Setup Vuforia
        vuforia.setDogeCVDetector(detector); // Set the Vuforia detector
        vuforia.enableDogeCV(); //Enable the DogeCV-Vuforia combo
        vuforia.showDebug(); // Show debug info
        vuforia.start(); // Start the detector
    }

    boolean goldAligned() {
        boolean isAligned = false;
        int centerValue = 275;
        int uncertainty = 25;
        if (Math.abs(detector.getXPosition() - centerValue) <= uncertainty)
            isAligned = true;
        return isAligned;
    }

}
enum EncoderMode {
    CONSTANT_SPEED, CONSTANT_POWER;
}



