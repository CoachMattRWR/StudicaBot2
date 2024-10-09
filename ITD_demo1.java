/* ITD_demo1 - example autonomous mode program for the INTO THE DEEP game.

    A Red autonomous program using April Tags to navigate the field.
    Also make use of the IMU to turn and drive straight.
    Setup is on Tile F3 flat against the wall. 
    Note: the webcam needs to see tag 16 from the starting position.
    
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal.StreamFormat;

import android.util.Size;
import java.util.List;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;

@Autonomous(name="ITD_demo1", group = "auto")
//@Disabled
public class ITD_demo1 extends LinearOpMode
{
     
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.030  ;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.025 ;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.025  ;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.75;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID = 16;                // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private int     myExposure  ;
    private int     myGain      ;
    private boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    
    private WebcamName webcam1 /*, webcam2*/;
    
    private ElapsedTime runtime = new ElapsedTime();
    private int     currentStep            = 1;
    private double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    private double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    private double  turn            = 0;        // Desired turning power/speed (-1 to +1)
    private double  timeout         = 5000;     // timeout in ms

    IMU imu;
    YawPitchRollAngles orientation;
    
    @Override public void runOpMode()
    {
        // Initialize TensorFlow, April Tag, and Vision Portal
        initVisionPortal();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "drive_leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "drive_leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "drive_rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "drive_rightBack");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        
        // init IMU
        imu = hardwareMap.get(IMU.class, "imu");
        double targetYaw = 0;
        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        waitForStart();

        visionPortal.stopLiveView();  // turn off liveView while robot moving

        if (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            orientation = imu.getRobotYawPitchRollAngles();
            
            // push sample into net zone, -10 to move away from wall, then drive forward.
            currentStep = 1;
            imuTurn(-10.0);             // turn to -10 which is a clockwise rotation
            imuMove(0.4, -10.0, 800);   // drive by time 800ms using -10 heading and 0.4 motor power.
            
            // turn so we can see tag 16
            currentStep = 2;
            imuTurn(-45.0);
            //imuMove(-0.4, 0.0, 2000);
            
            //sleep(5000);
            
            // align on April Tag 16 - red front wall, the robot will strafe to the right.
            currentStep = 3;
            aprilTagDrive(DESIRED_TAG_ID,20.0,0.0,0.0);

            // strafe right pointing at tag 16, lined up on a second sample
            currentStep = 4;
            aprilTagDrive(DESIRED_TAG_ID,28.0,0.0,-50.0);

            // rotate to point at net zone
            currentStep = 5;
            //aprilTagDrive(DESIRED_TAG_ID,28.0,-30.0,-60.0);
            imuTurn(70.0);

            // push second sample into net zone
            currentStep = 6;
            imuMove(0.4, 70.0, 1900);
            
            //  backup
            currentStep = 7;
            imuMove(-0.4, 70.0, 1100);
            
            // turn to face tag 16
            currentStep = 8;
            imuTurn(40.0);
 
            //  line up to go towards parking zone
            currentStep = 9;
            aprilTagDrive(DESIRED_TAG_ID,36.0,0.0,0.0);
            
            // backup half way
            currentStep = 10;
            aprilTagDrive(DESIRED_TAG_ID,72.0,0.0,0.0);
     
            // rotate to face tag 14
            currentStep = 11;
            imuTurn(160.0);
            DESIRED_TAG_ID = 14;
            
            // use tag 14 to park, 20" away with -25 yaw. The robot will strafe to the right.
            currentStep = 12;
            aprilTagDrive(DESIRED_TAG_ID,20.0,0.0,-20.0);
        }
    }
    
    private void displayTelemetry() {
        telemetry.addData("current step", currentStep);
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("tag target", DESIRED_TAG_ID);
        telemetry.addData("tag found", targetFound);
        if (targetFound)  {
            telemetry.addData("tag range", desiredTag.ftcPose.range);
            telemetry.addData("tag bearing", desiredTag.ftcPose.bearing);
            telemetry.addData("target yaw", desiredTag.ftcPose.yaw);
        } 
        
        telemetry.update();
    }

    private void initVisionPortal() {
        
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
        //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        //.setLensIntrinsics(1439.41944052, 1439.41944052, 970.51421863, 537.612825157) // logitech 920
        .build();
        
        //aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                    .setCamera(webcam1)
                    //.setCameraResolution(new Size(1920, 1080))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTag)
                    .build();
    }
    
    /**
     * Move robot to a designated position from an April Tag
     * targetTag is the tag to look for
     * targetRange is the desired target range in inches
     * targetBearing is the desired target bearing in degrees
     *  - set zero so camera is looking at the tag
     * targetYaw is the desired target yaw in degrees
     *  - set zero so robot lines up in front of the tag
     * 
     */
    public void aprilTagDrive(int targetTag, double targetRange, double targetBearing, double targetYaw) {
        double drive, strafe, turn;
        boolean done = false;
        runtime.reset();  // start timer 
        
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        targetFound = false;
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == targetTag)){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }
        
        while ( opModeIsActive() && !done && runtime.milliseconds() < timeout ) {
            if (targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - targetRange);
                double  headingError    = desiredTag.ftcPose.bearing - targetBearing;
                double  yawError        = desiredTag.ftcPose.yaw - targetYaw;
                              
                if ((Math.abs(rangeError) < 3) && (Math.abs(headingError) < 5) && (Math.abs(yawError) < 5)) {
                    done = true;
                    telemetry.addData("reached done", "done");
                    telemetry.update();
                } 
                else {
                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        
                    displayTelemetry();
                    // Apply desired axes motions to the drivetrain.
                    moveRobot(drive, strafe, turn);
                }
            }
            else moveRobot(0, 0, 0); 
            
            currentDetections = aprilTag.getDetections();
            targetFound = false;
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id == targetTag)){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }
        }
        moveRobot(0, 0, 0); // ensure we stop when done
    }
    
    /**
     * Move robot using a power level, and heading
     * Positive powerLevel is forward
     * duration assume to be positive and in milliseconds
     * heading is direction in IMU coordinate system
     * 
     */
    public void imuMove(double powerLevel, double heading, double duration) {
        YawPitchRollAngles orientation;
        double turn, headingError;

        runtime.reset();  // start timer
        while ( opModeIsActive() && runtime.milliseconds() < duration ) {
            orientation = imu.getRobotYawPitchRollAngles();
            headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            if (powerLevel < 0) {
                turn = turn * -1;  // reverse turn if going backwards
            }
            moveRobot(powerLevel, 0, turn);
            displayTelemetry();
        }
        moveRobot(0,0,0);
    }

    /**
     * Turn robot using a heading.
     * Heading is a direction in IMU coordinate system
     * where negative values indicate a clockwise heading.
     * We assume the turn completes within 2 seconds, there's timeout.
     */
    public void imuTurn(double heading) {
        YawPitchRollAngles orientation;
        double turn, headingError;
        boolean done = false;
        runtime.reset();  // start timer 
        
        while (opModeIsActive() && !done && runtime.milliseconds() < 2000) {
            orientation = imu.getRobotYawPitchRollAngles();
            headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
            if (Math.abs(headingError) > 5) {
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                moveRobot(0, 0, turn);
            }
            else done = true;
            displayTelemetry();
        }
        moveRobot(0, 0, 0);
    }

    /**
     * Move robot according to desired axes motions
     * NOTE: uses a robot centric system, NOT the FTC field coordinate system.
     */
    public void moveRobot(double x, double y, double yaw) {
        /* positive values of x move forward
           positive values of y move sideways to the right 
           positive values of yaw rotate clockwise
        */
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        sleep(10);
    }

}
