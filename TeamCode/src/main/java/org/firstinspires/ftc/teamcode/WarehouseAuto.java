package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.CapMech;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.vision.SkystoneStyleThreshold;

@Autonomous
public class WarehouseAuto extends LinearOpMode {
    // Pre-init
    Camera webcam  = new Camera();
    SkystoneStyleThreshold pipeline = new SkystoneStyleThreshold();
    SampleMecanumDrive drive;
    ArmSystem armSystem = new ArmSystem();
    Deposit deposit = new Deposit();
    Intake intake = new Intake();
    CapMech capMech = new CapMech();

    int hubActiveLevel = 0;

    int side; // Red alliance is 1, blue is -1
    boolean deepPark = true;

    final double originToWall = 141.0/2.0; // I guess the field is actually 141 inches wide
    final double wallDistance = originToWall - 6.5; // Center of bot is 6.5in from wall

    // Realative to warehouse
    private Pose2d farTsePosition;
    private Pose2d middleTsePosition;
    private Pose2d closeTsePosition;
    private Pose2d closeTsePosPreMove;

    Pose2d startPos = new Pose2d(11.4,-(originToWall-9), Math.toRadians(-90));
    Pose2d depositPos;
    Pose2d tsePos;
    Trajectory depositPreLoad;
    TrajectorySequence pickUpTSE;
    TrajectorySequence park;

    @Override
    public void runOpMode() {
        // Init
        webcam.init(hardwareMap);
        webcam.webcam.setPipeline(pipeline);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);
        armSystem.init(hardwareMap);
        armSystem.setArmPosition(0,0);
        deposit.init(hardwareMap);
        intake.init(hardwareMap);
        capMech.init(hardwareMap);

        ElapsedTime depositTimer = new ElapsedTime();
        ElapsedTime pipelineThrottle = new ElapsedTime();

        FtcDashboard.getInstance().startCameraStream(webcam.webcam, 1); // Stream to dashboard at 1 fps

        AutoToTele.allianceSide = 1;

        tsePos = new Pose2d(12,-50*side,Math.toRadians(225*side));

        while (!isStarted()&&!isStopRequested()){ // Init loop

            // Select alliance with gamepad and display it to telemetry
            if (gamepad1.b) AutoToTele.allianceSide = 1;
            if (gamepad1.x) AutoToTele.allianceSide = -1;
            // Select park depth
            if (gamepad1.dpad_up) deepPark = true;
            if (gamepad1.dpad_down) deepPark = false;

            side = AutoToTele.allianceSide;
            switch (side) {
                case 1:
                    telemetry.addLine("red alliance");
                    break;
                case -1:
                    telemetry.addLine("blue alliance");
                    break;
            }

            telemetry.addData("going to level", hubActiveLevel);
            telemetry.addData("deepPark", deepPark);
            telemetry.update();

           if (pipelineThrottle.milliseconds() > 1000) {// Throttle loop times to 1 second
               // Update startpos to match side
               startPos = new Pose2d(11.4,(-(originToWall-9))*side, Math.toRadians(-90*side));
               drive.setPoseEstimate(startPos); // Set pose estimate to match which side of the field we're on

               farTsePosition = new Pose2d(2.5,-48*side,Math.toRadians(-90*side));
               middleTsePosition = new Pose2d(11.8,-48*side,Math.toRadians(-90*side));
               closeTsePosition = new Pose2d(9.5,-44*side,Math.toRadians(225*side));
               closeTsePosPreMove = new Pose2d(4,-48*side,tsePos.getHeading());

               switch (pipeline.getAnalysis()){
                   case LEFT:
                       hubActiveLevel = 1;
                       break;
                   case MIDDLE:
                       hubActiveLevel = 2;
                       break;
                   case RIGHT:
                       hubActiveLevel = 3;
                       break;
               }

               switch (hubActiveLevel) {
                   case 1:
                       depositPos = new Pose2d(-2, -41*side, Math.toRadians(-70*side));
                       if (side == 1){ // Switch close and far positions on blue alliance
                           tsePos = farTsePosition;
                           pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                                   .lineToSplineHeading(new Pose2d(tsePos.getX(),-55*side,tsePos.getHeading())) // Pre move
                                   .lineToSplineHeading(tsePos)
                                   .build();
                       }
                       else {
                           tsePos = closeTsePosition;
                           pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                                   .lineToSplineHeading(closeTsePosPreMove) // Pre move
                                   .lineToSplineHeading(tsePos)
                                   .build();
                       }
                       break;
                   case 2:
                       depositPos = new Pose2d(-2, -42*side, Math.toRadians(-70*side));
                       tsePos = middleTsePosition;
                       pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                               .lineToSplineHeading(new Pose2d(tsePos.getX(),-55*side,tsePos.getHeading())) // Pre move
                               .lineToSplineHeading(tsePos)
                               .build();
                       break;
                   case 3:
                       depositPos = new Pose2d(-2, -42*side, Math.toRadians(-70*side));
                       if (side == 1){  // Switch close and far positions on blue alliance
                           tsePos = closeTsePosition;
                           pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                                   .lineToSplineHeading(closeTsePosPreMove) // Pre move
                                   .lineToSplineHeading(tsePos)
                                   .build();
                       }
                       else {
                           tsePos = farTsePosition;
                           pickUpTSE = drive.trajectorySequenceBuilder(startPos)
                                   .lineToSplineHeading(new Pose2d(tsePos.getX(),-55*side,tsePos.getHeading())) // Pre move
                                   .lineToSplineHeading(tsePos)
                                   .build();
                       }
                       break;
               }

               // Deposit trajectory
               depositPreLoad = drive.trajectoryBuilder(pickUpTSE.end())
                       .lineToSplineHeading(depositPos)
                       .build();

               // Park trajectory
               if (deepPark) park = drive.trajectorySequenceBuilder(depositPreLoad.end())
                       .addTemporalMarker(0.7, () -> armSystem.setArmPosition(0,0))
                       .lineToSplineHeading(new Pose2d(0, -wallDistance*side, Math.toRadians(0*side)))
                       .addTemporalMarker(0.5, () -> armSystem.setArmPosition(0,0))
                       .lineToSplineHeading(new Pose2d(36, -wallDistance*side, Math.toRadians(0*side))) // Go into warehouse
                       .lineTo(new Vector2d(36,(-(originToWall-34))*side))
                       .lineToSplineHeading(new Pose2d(63,-(originToWall-32.5)*side,Math.toRadians(-90*side)))
                       .build();
               else {
                   park = drive.trajectorySequenceBuilder(depositPreLoad.end())
                           .addTemporalMarker(0.7, () -> armSystem.setArmPosition(0,0))
                           .lineToSplineHeading(new Pose2d(0, -wallDistance*side, Math.toRadians(0*side)))
                           .addTemporalMarker(0.5, () -> armSystem.setArmPosition(0,0))
                           .lineToSplineHeading(new Pose2d(38, -wallDistance*side, Math.toRadians(0*side))) // Go into warehouse
                           .build();
               }

               // Telemetry
               telemetry.addData("going to level", hubActiveLevel);
               telemetry.update();
               pipelineThrottle.reset(); // Reset the throttle timer so the whole thing loops
           } // End of throttled section
        }// End of init loop

        waitForStart();
        // Pre-run
    
        if (opModeIsActive()) {
            // Autonomous instructions
            capMech.openGripper();
            capMech.levelArm();
            drive.followTrajectorySequence(pickUpTSE);
            sleep(1000);
            capMech.closeGripper();
            sleep(500);
            capMech.retract();
            armSystem.runToLevel(hubActiveLevel); // Extend 4b before driving
            drive.followTrajectory(depositPreLoad); // Drive to spot where we'll deposit from
            depositTimer.reset();
            deposit.dump(depositTimer); // Dump
            sleep(700); // Wait for that dump to finish
            deposit.dump(depositTimer);
            drive.followTrajectorySequence(park); // Park in warehouse
            intake.dropIntake();

            AutoToTele.endOfAutoPose = drive.getPoseEstimate();
            AutoToTele.endOfAutoHeading = drive.getExternalHeading();
            // Save this information to a class so we can use it in tele to calibate feild centric

            // For debug purposes
            telemetry.addData("endheading",Math.toDegrees( AutoToTele.endOfAutoHeading));
            telemetry.update();
            sleep(2*1000);
        }
    }
}
