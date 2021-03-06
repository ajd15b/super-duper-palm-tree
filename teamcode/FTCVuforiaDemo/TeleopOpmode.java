/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.FTCVuforiaDemo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * This example is designed to show how to identify a target, get the robot's position, and then plan
 * and execute an approach path to the target.
 *
 * This OpMode uses two "utility" classes to abstract (hide) the hardware and navigation GUTs.
 * These are:  Robot_OmniDrive and Robot_Navigation.
 *
 * This LinearOpMode uses basic hardware and nav calls to drive the robot in either manual or auto mode.
 * AutoMode is engaged by pressing and holding the Left Bumper.  Release the Bumper to return to Manual Mode.
 *
 *  *ManualMode* simply uses the joysticks to move the robot in three degrees of freedom.
 *  - Left stick X (translate left and right)
 *  - Left Stick Y (translate forward and backwards)
 *  - Right Stick X (rotate CW and CCW)
 *
 *  *AutoMode* will approach the image target and attempt to reach a position directly in front
 *  of the center line of the image, with a predefined stand-off distance.
 *
 *  To simplify this example, a gyro is NOT used.  Therefore there is no attempt being made to stabilize
 *  strafing motions, or to perform field-centric driving.
 *
 */

@TeleOp(name="Vuforia Tracking Demo", group="main")
public class TeleopOpmode extends LinearOpMode {

    final double TURN_SPEED = .5;

    /* Declare OpMode members. */
    ColorSensor colorSensor;
    Robot_OmniDrive     robot    = new Robot_OmniDrive();   // Use Omni-Directional drive system
    Robot_Navigation    nav      = new Robot_Navigation();  // Use Image Tracking library

    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        robot.initDrive(this);
        nav.initVuforia(this, robot);

        // Activate Vuforia (this takes a few seconds)
        nav.activateTracking();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");

            // Display any Nav Targets while we wait for the match to start
            nav.targetIsVisible(1);
            nav.addNavTelemetry();
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        int step = 0;
        int lastStep = 7;
        long lastNotSeen=System.currentTimeMillis();
        int imageInt = 0;
        int distance = 100;
        boolean turnLeft = true;
        long time;
        int moveDuration = 1000;
        while (opModeIsActive() && step<=lastStep) {

            //telemetry.addData(">", "Press Left Bumper to track target");

            // auto drive or manual drive?
            // In auto drive, the robot will approach any target it can see and then press against it
            // In manual drive the robot responds to the Joystick.
            //if (nav.targetsAreVisible() && gamepad1.left_bumper) {
                // Calculate automatic target approach

            //wheels - beacon
            //lego face lego
            // turn left
            //Slam into stick
            //Gears - beacon

            switch(step)
            {
                case 0:
                    //get off the ramp
                    step++;
                    break;
                case 1:
                    //go to wheels pic
                    if(goToPicture(step, imageInt, distance, turnLeft)){
                        step++;
                    }
                    break;
                case 2:
                    step++;
                    //press wheels beacon
                    if(colorSensor.red() > 0)
                    {
                        //go farther forward and push the button
                        robot.moveRobot(0,1,0);
                    }
                    else if(colorSensor.blue() > 0) {
                        //robot would need to back up and either go farther to the left or right
                        robot.moveRobot(0, -1, 0);
                        robot.moveRobot(1, 1, 0);
                        //decrement the step to check to see if it sees red now (retry this step)
                        step--;
                    }
                    break;
                case 3:
                    //go to lego picture
                    imageInt= 2;
                    distance = 200;
                    if(goToPicture(step, imageInt, distance,turnLeft)){
                        step++;
                    }
                    break;
                case 4:
                    //turn to stick
                    time = System.currentTimeMillis();
                    while (System.currentTimeMillis() - time < moveDuration) {
                        robot.moveRobot(0, 0, TURN_SPEED);
                    }
                    step++;
                    break;
                case 5:
                    //drive over stick
                    time = System.currentTimeMillis();
                    moveDuration = 5000;
                    while (System.currentTimeMillis() - time < moveDuration) {
                        robot.moveRobot(1,0, 0);
                    }
                    step++;
                    break;
                case 6:
                    //go to gears picture
                    turnLeft = false;
                    imageInt = 3;
                    distance = 100;
                    if(goToPicture(step,imageInt,distance,turnLeft)){
                        step++;
                    }
                    break;
                case 7:
                    //press gears beacon
                    step++;
                    break;
            }
            nav.addNavTelemetry();
            telemetry.update();

           // } else {
                // Drive the robot using the joysticks
             //   robot.manualDrive();
            //}

            // Build telemetry messages with Navigation Information;


        }

        telemetry.addData(">", "Shutting Down. Bye!");
        telemetry.update();
    }


    private boolean goToPicture(int step, int imageInt, int distance, boolean turnLeft) {
        boolean done=false;
            if (nav.targetIsVisible(imageInt) ) {
                done = nav.cruiseControl(distance);
                if (!done) {

                    //  Move the robot according to the pre-determined axis motions
                    robot.moveRobot();
                } else {
                    robot.moveRobot(0, 0, 0);
                }
            } else {
                if(turnLeft) {
                    robot.moveRobot(0, 0, TURN_SPEED);
                }else{
                    robot.moveRobot(0,0,-TURN_SPEED);
                }
                //System.currentTimeMillis();
            }
        return done;
    }
}
