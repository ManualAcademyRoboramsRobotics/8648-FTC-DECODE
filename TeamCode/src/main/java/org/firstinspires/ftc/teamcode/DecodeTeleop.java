/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@TeleOp(name = "DECODE Teleop")
//@Disabled
public class DecodeTeleop extends DecodeControl {


    double requestedVelocity = 2100;

    double launcherTarget = requestedVelocity; //These variables allow
    double launcherMin = requestedVelocity - 100;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        mecanumDrive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.y) {
            launcherSpinUp();
        }

        if (gamepad1.b) {
            launcherStop();
        }

        if (gamepad1.dpadDownWasPressed()) {
            diverterDirectionToggle();
        }

        if (gamepad1.aWasPressed()){
            intakeStateToggle();
        }

//        if (gamepad1.dpadUpWasPressed()) {
//            launcherVelocityToggle();
//        }
        if (gamepad1.dpadLeftWasPressed()) {
            requestedVelocity -= 100;
            setLauncherVelocity(requestedVelocity);
        }

        if (gamepad1.dpadRightWasPressed()) {
            requestedVelocity += 100;
            setLauncherVelocity(requestedVelocity);
        }

        launchLeft(gamepad1.leftBumperWasPressed());
        launchRight(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("Left State", leftLauncherState);
        telemetry.addData("Right State", rightLauncherState);
        telemetry.addData("Requested Velocity", requestedVelocity);
        telemetry.addData("Launcher Velocity", launcherVelocity);
        telemetry.addData("Left Launcher Velocity", Math.abs(leftLauncher.getVelocity()));
        telemetry.addData("Right Launcher Velocity", Math.abs(leftLauncher.getVelocity()));
        telemetry.addData("Diverter Direction", diverterDirection);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

//    void mecanumDrive(double forward, double strafe, double rotate){
//
//        /* the denominator is the largest motor power (absolute value) or 1
//         * This ensures all the powers maintain the same ratio,
//         * but only if at least one is out of the range [-1, 1]
//         */
//        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
//
//        leftFrontPower = (forward + strafe + rotate) / denominator;
//        rightFrontPower = (forward - strafe - rotate) / denominator;
//        leftBackPower = (forward - strafe + rotate) / denominator;
//        rightBackPower = (forward + strafe - rotate) / denominator;
//
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//    }


}