/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name="Test2_otos_driving", group="test")

public class Test2_otos_driving extends LinearOpMode {
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.035  ;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.15;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.035  ;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    
    // Create an instance of the sensor
    SparkFunOTOS myOtos;
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "drive_leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "drive_leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "drive_rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "drive_rightBack");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "SparkFun");
        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();
        sleep(1000);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        
        //moveRobotTest();
        otosDrive(22, -6, 0);// left spike mark
        otosDrive(22, -6, -30);// lturn
        sleep(500);
        otosDrive(12, -4, 0);     // backup
        otosDrive(12, -20, 0);  // move to front wall
        otosDrive(50, -20, 0);  // drive forward
        //sleep(500);
        otosDrive(50, -20, 0);  // turn to face toward backdrop, 
        //sleep(500);
        otosDrive(50, 68, 0);   // turn and drive towards backdrop
        //sleep(500);
        otosDrive(50, 68, 135); // turn and face backdrop
                
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // step 1 - forward 24"
            //otosDrive(24, 0, 0);
                
            // step 2 - right 24"
            //otosDrive(0, 24, 0);
            
            //otosDrive(0, 0, 90);    // should be counter clockwise
            
            //otosDrive(24,24,0);     // forward 24, over 24.
            // note this doesn't go at a 45 degree angle, more forward at the start, more strafing at the end
            
            //otosDrive(24,24,45);     // forward 24, over 24 and angled at 45. this was a little wonky
            
            //otosDrive(12, 0, 0);
            //otosDrive(12, 0, 30);   // face spike mark
            //otosDrive(24, -6, 30);  // drive up to it
            //otosDrive(4, 0, 0);     // return to starting tile
            //otosDrive(4, 0, -90);   // turn to face backstage
            //otosDrive(4, 72, -90); // drive backstage  -- goes wonky 
        }
    }
    
        private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // stored in the sensor, it's part of the library, so you need to set at the
        // start of all your programs.
        // myOtos.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        myOtos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        // myOtos.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
        myOtos.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.75, -7.5, 90); // should be -3.75 & -7.5 and 90
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.008);
        myOtos.setAngularScalar(0.992);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
    
    /**
     * Move robot to a designated X,Y position and heading
     */
     void otosDrive(double targetX, double targetY, double targetHeading) {
        double drive, strafe, turn;
        double currentRange, targetRange, initialBearing, targetBearing, xError, yError, yawError;
        double opp, adj;

        SparkFunOTOS.Pose2D currentPos = myPosition();
        xError = targetX-currentPos.x; 
        yError = targetY-currentPos.y;
        yawError = targetHeading-currentPos.h;
                          
        while(opModeIsActive() && ((Math.abs(xError) > 1.5) || (Math.abs(yError) > 1.5) 
                                    || (Math.abs(yawError) > 4)) ) {
        // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(xError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            // current x,y swapped due to 90 degree rotation
            telemetry.addData("current X coordinate", currentPos.x);
            telemetry.addData("current Y coordinate", currentPos.y);
            telemetry.addData("current Heading angle", currentPos.h);
            telemetry.addData("target X coordinate", targetX);
            telemetry.addData("target Y coordinate", targetY);
            telemetry.addData("target Heading angle", targetHeading);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("yawError", yawError);
            telemetry.update();
                
            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            
            // then recalc error
            currentPos = myPosition();
            xError = targetX-currentPos.x;  
            yError = targetY-currentPos.y;
            yawError = targetHeading-currentPos.h;
        }
        moveRobot(0,0,0);
        currentPos = myPosition();
        telemetry.addData("current X coordinate", currentPos.x);
        telemetry.addData("current Y coordinate", currentPos.y);
        telemetry.addData("current Heading angle", currentPos.h);
        telemetry.update();
    }

    /* the reported OTOS values are based on sensor orientation, convert to robot centric
        by swapping x and y and changing the sign of the heading
        */
    SparkFunOTOS.Pose2D myPosition() {
        pos = myOtos.getPosition();
        SparkFunOTOS.Pose2D myPos = new SparkFunOTOS.Pose2D(pos.y, pos.x, -pos.h);
        return(myPos);
    }
    /**
     * Move robot according to desired axes motions assuming robot centric point of view
     * Positive X is forward
     * Positive Y is strafe right
     * Positive Yaw is clockwise
     */
     void moveRobot(double x, double y, double yaw) {
        /* positive values of x move forward
           positive values of y move sideways to the right 
           positive values of yaw rotate counter clockwise
        */
        
        // Calculate wheel powers.
        double leftFrontPower    =  x +y +yaw;
        double rightFrontPower   =  x -y -yaw;
        double leftBackPower     =  x -y +yaw;
        double rightBackPower    =  x +y -yaw;

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
    
    void moveRobotTest() {
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            pos = myOtos.getPosition();
            telemetry.addData("Status", "move forward");
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("h", pos.h);
            telemetry.update();
            moveRobot(0.25,0,0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 1000) {
            pos = myOtos.getPosition();
            telemetry.addData("Status", "pause");
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("h", pos.h);
            telemetry.update();
            moveRobot(0,0,0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            pos = myOtos.getPosition();
            telemetry.addData("Status", "move back");
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("h", pos.h);
            telemetry.update();
            moveRobot(-0.25,0,0);
        }
                
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            pos = myOtos.getPosition();
            telemetry.addData("Status", "move left");
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("h", pos.h);
            telemetry.update();
            moveRobot(0,0.25,0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 1000) {
            pos = myOtos.getPosition();
            telemetry.addData("Status", "pause");
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("h", pos.h);
            telemetry.update();
            moveRobot(0,0,0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            pos = myOtos.getPosition();
            telemetry.addData("Status", "move back");
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("h", pos.h);
            telemetry.update();
            moveRobot(0,-0.25,0);
        }
        
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            pos = myOtos.getPosition();
            telemetry.addData("Status", "rotate counter clockwise");
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("h", pos.h);
            telemetry.update();
            moveRobot(0,0,0.25);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 1000) {
            pos = myOtos.getPosition();
            telemetry.addData("Status", "pause");
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("h", pos.h);
            telemetry.update();
            moveRobot(0,0,0);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            pos = myOtos.getPosition();
            telemetry.addData("Status", "rotate back");
            telemetry.addData("x", pos.x);
            telemetry.addData("y", pos.y);
            telemetry.addData("h", pos.h);
            telemetry.update();
            moveRobot(0,0,-0.25);
        }

        moveRobot(0,0,0);   //stop
        telemetry.addData("Status", "finished");
        telemetry.addData("x", pos.x);
        telemetry.addData("y", pos.y);
        telemetry.addData("h", pos.h);
        telemetry.update();
        sleep(1000);
    }
}
