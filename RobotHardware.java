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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotHardware
{
   // DECLARE ALL MOTORS AND SERVOS 
     public DcMotorEx frontLeft = null; // Port 0 CH
     public DcMotorEx frontRight = null; // Port 3 EH
     public DcMotorEx rearLeft = null; // Port 1 CH
     public DcMotorEx rearRight = null; // Port 2 EH
     //public DcMotor leftLift = null; 
     //public DcMotor rightLift = null;
     public DcMotorEx viperLift = null; // Port 1 EH
     public DcMotorEx robotLift1 = null; // Port 0 EH
     public DcMotorEx robotLift2 = null;


     public Servo   leftIntake = null; // Servo Port 0 CH
     public  Servo   rightIntake = null; // Servo Port 1 CH
    
     
     //IDENTIFY MAXIMUMS AND MINIMUMS 
     public static final double CLAW_HOME = .5; 
     public static final double CLAW_MIN = .5; 
     public static final double CLAW_MAX = 1;
     

    
        /* local OpMode members. */
        HardwareMap hardwareMap                     =    null;
        private ElapsedTime period    = new ElapsedTime();

    
        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap hardwareMap) {
            
    
                // DEFINE EACH DECLARED MOTOR AND SERVO
                frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
                frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
                rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
                rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
                viperLift = hardwareMap.get(DcMotorEx.class,  "viperLift") ;
                robotLift1 = hardwareMap.get(DcMotorEx.class,  "robotLift1") ;
                robotLift2 = hardwareMap.get(DcMotorEx.class,  "robotLift2") ;

                //rightLift = hardwareMap.get(DcMotorEx. class, "rightLift");
               // leftLift = hardwareMap.get(DcMotorEx. class, "leftLift");
                leftIntake = hardwareMap.get(Servo. class, "leftIntake");
                rightIntake = hardwareMap.get(Servo. class, "rightIntake");
             
               //SET POWER MOTOR POWER UPON INITILIZATION 
                frontLeft.setPower(0);
                frontRight.setPower(0);
                rearLeft.setPower(0);
                rearRight.setPower(0);
                //rightLift.setPower(0);
                //leftLift.setPower(0);
                viperLift.setPower(0);
                robotLift1.setPower(0);
                robotLift2.setPower(0);

        
                
                // SET MOTOR BEHAVIOR  
            frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            //rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            viperLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robotLift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robotLift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            
             // SET MOTOR DIRECTION
            frontRight.setDirection(DcMotorEx.Direction.FORWARD);
            frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            rearRight.setDirection(DcMotorEx.Direction.FORWARD);
            rearLeft.setDirection(DcMotorEx.Direction.REVERSE);
            viperLift.setDirection(DcMotorEx.Direction.REVERSE);
            robotLift1.setDirection(DcMotorEx.Direction.FORWARD);
            
           // rightLift.setDirection(DcMotor.Direction.REVERSE);
            //leftLift.setDirection(DcMotor.Direction.FORWARD);


              //DEFINE SERVO HOME POSITIONS 
         leftIntake.setPosition(CLAW_HOME);
         rightIntake.setPosition(CLAW_HOME);
            
    }
}
