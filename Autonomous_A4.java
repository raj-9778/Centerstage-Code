package org.firstinspires.ftc.teamcode;
  
  import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
  import com.qualcomm.robotcore.hardware.DcMotor;
  import com.qualcomm.robotcore.hardware.DcMotorSimple;
  import com.qualcomm.robotcore.hardware.CRServo;
  import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
  import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
  import com.qualcomm.robotcore.eventloop.opmode.OpMode;
  import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
  import org.firstinspires.ftc.vision.VisionPortal;
  import android.util.Size;
  
 
  @Autonomous
  public class Autonomous_A4 extends LinearOpMode { 
      private DcMotor frontLeft;
      private DcMotor rearLeft;
      private DcMotor frontRight;
      private DcMotor rearRight;
      private CRServo leftServo;
      private CRServo rightServo;
      private DcMotor Happy;
      private CSVisionProcessor visionProcessor;
      private VisionPortal visionPortal;
      private VisionPortal.Builder visionPortalBuilder;

       @Override
       public void runOpMode() {
          frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");          rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
          frontRight = hardwareMap.get(DcMotor.class, "frontRight");
          rearRight = hardwareMap.get(DcMotor.class, "rearRight");
          leftServo = hardwareMap.get(CRServo.class, "left_hand");
          rightServo = hardwareMap.get(CRServo.class, "right_hand");
          Happy = hardwareMap.get(DcMotor.class, "Happy");
          rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
          frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
          
          visionProcessor = new CSVisionProcessor(); 
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam1"), visionProcessor);
        visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class,"Webcam1"));
        visionPortalBuilder.addProcessor(visionProcessor);
        visionPortalBuilder.setCameraResolution(new Size(720, 480));
        visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        
        visionPortal = visionPortalBuilder.build();
        
        CSVisionProcessor.StartingPosition startingPos = CSVisionProcessor.StartingPosition.NONE;

        
        
       // visionPortal.startStreaming();

        while(!this.isStarted() && !this.isStopRequested()) {
            startingPos = visionProcessor.getStartingPosition();
            telemetry.addData("Identified", visionProcessor.getStartingPosition());
            telemetry.update();
        }
       
          waitForStart();
     if (startingPos == startingPos.CENTER) {
     
          drive("STRAIGHT", 0.4, 1265);
          drive("STRAIGHT", -0.4, 400);
          drive("LEFT", 92, 1800);
          drive("STRAIGHT", 0.4, 1140);
          drive("STRAFERight", 0.4, 300);
          moveHappy();
          drive("STRAIGHT", -0.4, 300);
          moveHappyUP();
          drive("STRAIGHT", 0.1, 450);
          drive("STRAFELeft", 0.4, 2000);
          
     }
      else if (startingPos == startingPos.RIGHT) {
          
          drive("STRAIGHT", 0.4, 900);
          drive("RIGHT45", 45, 1350);
 //       drive("RIGHT", 92, 1525);
          drive("STRAIGHT", -0.2, 900);
          drive("RIGHT45", 45, 800);
          drive("STRAFERight", 0.1, 400);
          drive("STRAIGHT", 0.4, 1200);
 //       drive("STRAFERight", 0.1, 500);
 //       drive("STRAFELeft", 0.4, 300);
          moveHappy();
          drive("STRAIGHT", -0.1, 400);
          moveHappyUP();
          drive("STRAIGHT", 0.1, 500);
          drive("STRAFERight", 0.4, 4000);
          
     }
           
      else {
          drive("STRAIGHT", 0.4, 900);
          drive("LEFT", 92, 1800);
          drive("STRAIGHT", 0.4, 1140);
          drive("STRAFELeft", 0.4, 300);
          moveHappy();
          drive("STRAIGHT", -0.2, 400);
          moveHappyUP();
          drive("STRAIGHT", 0.1, 450);
          drive("STRAFELeft", 0.4, 2000); 
/*          drive("STRAIGHT", -0.3, 1000);
          drive("RIGHT", 180, 2450);
          drive("STRAFELeft", 0.1, 1550);
          drive("STRAIGHT", 0.4, 800);
          moveHappy();
          drive("STRAIGHT", -0.2, 400);
          moveHappyUP();
          drive("STRAIGHT", 0.1, 500);
          drive("STRAFERight", 0.4, 20000);
          */
       
      }
     }
     public void drive(String turn, double turnBy, int time) {
           if (turn == "STRAIGHT") { 
            /*  FrontLeftDCMotor.setPower(turnBy);
              FrontRightDCMotor.setPower(turnBy);
              BackLeftDCMotor.setPower(turnBy);
              BackRightDCMotor.setPower(turnBy);
              sleep(time);
              FrontLeftDCMotor.setPower(0);
              FrontRightDCMotor.setPower(0);
              BackLeftDCMotor.setPower(0);
              BackRightDCMotor.setPower(0); */
              
           }
           if (turn == "RIGHT") {
           /*   FrontLeftDCMotor.setPower(0.5);
              FrontRightDCMotor.setPower(0.25*-1);
              BackLeftDCMotor.setPower(0.25);
              BackRightDCMotor.setPower(0.25*-1);
              sleep(time);
              FrontLeftDCMotor.setPower(0);
              FrontRightDCMotor.setPower(0);
              BackLeftDCMotor.setPower(0);
              BackRightDCMotor.setPower(0);  */
           }
           if (turn == "RIGHT45") {
          /*    FrontLeftDCMotor.setPower(0.5);
              FrontRightDCMotor.setPower(0.15*-1);
              BackLeftDCMotor.setPower(0.15);
              BackRightDCMotor.setPower(0.15*-1);
              sleep(time);
              FrontLeftDCMotor.setPower(0);
              FrontRightDCMotor.setPower(0);
              BackLeftDCMotor.setPower(0);
              BackRightDCMotor.setPower(0);    */
           }
           if (turn == "STRAFELeft") {
             /* FrontLeftDCMotor.setPower(0.25*-1);
              FrontRightDCMotor.setPower(0.25);
              BackLeftDCMotor.setPower(0.25);
              BackRightDCMotor.setPower(0.25*-1);
              sleep(time);
              FrontLeftDCMotor.setPower(0);
              FrontRightDCMotor.setPower(0);
              BackLeftDCMotor.setPower(0);
              BackRightDCMotor.setPower(0);  */
           }
           if (turn == "STRAFERight") {
         /*     FrontLeftDCMotor.setPower(0.25);
              FrontRightDCMotor.setPower(0.25*-1);
              BackLeftDCMotor.setPower(0.25*-1);
              BackRightDCMotor.setPower(0.25);
              sleep(time);
              FrontLeftDCMotor.setPower(0);
              FrontRightDCMotor.setPower(0);
              BackLeftDCMotor.setPower(0);
              BackRightDCMotor.setPower(0);  */
           }
           if (turn == "LEFT45") {
           /*   FrontLeftDCMotor.setPower(0.15*-1);
              FrontRightDCMotor.setPower(0.5);
              BackLeftDCMotor.setPower(0.15*-1);
              BackRightDCMotor.setPower(0.15);
              sleep(time);
              FrontLeftDCMotor.setPower(0);
              FrontRightDCMotor.setPower(0);
              BackLeftDCMotor.setPower(0);
              BackRightDCMotor.setPower(0);   */
           }
           if (turn == "LEFT") {
        /*      FrontLeftDCMotor.setPower(0.15*-1);
              FrontRightDCMotor.setPower(0.5);
              BackLeftDCMotor.setPower(0.15*-1);
              BackRightDCMotor.setPower(0.15);
              sleep(time);
              FrontLeftDCMotor.setPower(0);
              FrontRightDCMotor.setPower(0);
              BackLeftDCMotor.setPower(0);
              BackRightDCMotor.setPower(0);   */
           }
          
           
        }   
       public void moveHappy() {
        
        for(int i=0;i<5;i++){
           Happy.setPower(1);
           sleep(1300);
         
         }
         Happy.setPower(0);
         for(int i=0;i<2;i++){
         leftServo.setPower(-1);
         rightServo.setPower(1);
         sleep(1001);
         
         }
         leftServo.setPower(0);
         rightServo.setPower(0);
       }
       public void moveHappyUP() {
        
        for(int i=0;i<5;i++){
           Happy.setPower(-1);
           sleep(600);
         
         }
         Happy.setPower(0);
       }
       }
     
