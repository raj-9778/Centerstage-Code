/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriverControlla ", group="The Bot")

public class DriverControlled extends OpMode{

        /* Declare OpMode members. */
        
        // DECLARE ALL MOTORS AND SERVOS 
     public DcMotorEx frontLeft = null; // Port 0 CH
     public DcMotorEx frontRight = null; // Port 3 EH
     public DcMotorEx rearLeft = null; // Port 1 CH
     public DcMotorEx rearRight = null; // Port 2 EH
     public DcMotorEx viperLift = null;
     public DcMotorEx robotLift1 = null;
     public DcMotorEx robotLift2 = null;


     //public DcMotorEx rightLift = null; 
     //public DcMotorEx leftLift = null; 
     public Servo   leftIntake = null; // Servo Port 2 CH
     public Servo   rightIntake = null; // Servo Port 2 CH
     public Servo   endDump     = null; // Servo Port idk
     public Servo   thaShooter = null;
    
     
     //IDENTIFY MAXIMUMS AND MINIMUMS 
     public static final double CLAW_HOME = .43; 
     public static final double CLAW_MIN = .43; 
     public static final double CLAW_MAX = .93;
     

        double  armPosition = CLAW_MIN;
        final double ARM_SPEED = 1; 
    
        // Run Once on init ()                        
        @Override
        public void init() {
                
                 // DEFINE EACH DECLARED MOTOR AND SERVO
                frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
                frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
                rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
                rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
                //rightLift = hardwareMap.get(DcMotorEx. class, "rightLift");
                //leftLift = hardwareMap.get(DcMotorEx. class, "leftLift");
                viperLift = hardwareMap.get(DcMotorEx. class, "viperLift");
                robotLift1 = hardwareMap.get(DcMotorEx.class, "robotLift1");
                robotLift2 = hardwareMap.get(DcMotorEx.class, "robotLift2");


                thaShooter= hardwareMap.get(Servo. class,  "thaShooter") ;
                leftIntake= hardwareMap.get(Servo. class, "leftIntake");
                rightIntake= hardwareMap.get(Servo. class, "rightIntake");
                endDump= hardwareMap.get(Servo. class,  "endDump") ;
             
               //SET POWER MOTOR POWER UPON INITILIZATION 
                frontLeft.setPower(0);
                frontRight.setPower(0);
                rearLeft.setPower(0);
                rearRight.setPower(0);
                viperLift.setPower(0);              
                robotLift1.setPower(0);
                robotLift2.setPower(0);

                
        
                
                // SET MOTOR BEHAVIOR  
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            viperLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            
             // SET MOTOR DIRECTION
            frontRight.setDirection(DcMotorEx.Direction.FORWARD);
            frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            rearRight.setDirection(DcMotorEx.Direction.FORWARD);
            rearLeft.setDirection(DcMotorEx.Direction.REVERSE);
           // rightLift.setDirection(DcMotor.Direction.FORWARD);
           // leftLift.setDirection(DcMotor.Direction.REVERSE);
            viperLift.setDirection(DcMotorEx.Direction.REVERSE);
            //robotLift1.setDirection(DcMotorEx.Direction.FORWARD) ;
            
            
            //DEFINE SERVO HOME POSITIONS 
         leftIntake.setPosition(CLAW_HOME);
         rightIntake.setPosition(CLAW_HOME);
         endDump.setPosition(.8);
         
                // Send telemetry message to signify robot waiting;
                telemetry.addData("STAUS", "INTIALIZED");        //
        }

    

        // LOOP ON START
        @Override
        public void loop() {
                double G1rightStickY = gamepad1.right_stick_y;
                double G1leftStickY = -gamepad1.left_stick_y;
                double G1rightStickX = gamepad1.right_stick_x;
                double G1leftStickX = -gamepad1.left_stick_x;
                double G2rightStickY = -gamepad2.right_stick_y;
                double G2leftStickY= -gamepad2.left_stick_y;
                double G2rightStickX = -gamepad2.right_stick_x;
                double G2leftStickX= -gamepad2.left_stick_x;
             
                
                
            // FORMULA FOR DRIVETRAIN  
              //  frontRight.setPower(-G1leftStickY+gamepad1.left_trigger-gamepad1.right_trigger );
            //    frontLeft.setPower(G1rightStickY-gamepad1.left_trigger+gamepad1.right_trigger );
              //  rearRight.setPower(-G1leftStickY-gamepad1.left_trigger+gamepad1.right_trigger);
        //        rearLeft.setPower(G1rightStickY+gamepad1.left_trigger-gamepad1.right_trigger);
                
                // this is POV Mode or Field-centric or Drone mode
                frontRight.setPower(G1rightStickY-G1rightStickX+G1leftStickX);
                frontLeft.setPower(G1rightStickY+G1rightStickX-G1leftStickX );
                rearRight.setPower(G1rightStickY+G1rightStickX+G1leftStickX);
                rearLeft.setPower(G1rightStickY-G1rightStickX-G1leftStickX);
                
                // diagonal strafe
                if (gamepad1.dpad_left) {
                frontRight.setPower(.9); 
                rearLeft.setPower(.9);
                 } else if (gamepad1.right_bumper) {
                frontRight.setPower(-.9); 
                rearLeft.setPower(-.9); }
                
                if (gamepad1.dpad_right) {
                frontLeft.setPower(.9); 
                rearRight.setPower(.9);
                 } else if (gamepad1.left_bumper) {
                frontLeft.setPower(-.9); 
                rearRight.setPower(-.9); }
                
                
            // Robot Lift Controls 
                viperLift.setPower(G2rightStickY);
                
                robotLift1.setPower(gamepad2.left_trigger);
                robotLift1.setPower(-gamepad2.right_trigger);
                
                robotLift2.setPower(-gamepad2.left_trigger);
                robotLift2.setPower(gamepad2.right_trigger);


            if (gamepad1.a)
                thaShooter.setPosition(-1); 
            else if (gamepad1.y)
                thaShooter.setPosition(1);      


                //GRIPPER CONTROLS 
          if (gamepad2.a)
                armPosition -= ARM_SPEED; 
          else if (gamepad2.y)
                armPosition += ARM_SPEED; 
                
                
                
            if (gamepad2.dpad_down)
                 viperLift.setPower(G2rightStickY+.215) ;
            else
                 viperLift.setPower(G2rightStickY) ; 

             //DEFINE RANGE CLIPS       
          armPosition = Range.clip(armPosition, CLAW_MIN, CLAW_MAX );
          leftIntake.setPosition(armPosition);
         rightIntake.setPosition(armPosition);
                
            
                 
        }
}

