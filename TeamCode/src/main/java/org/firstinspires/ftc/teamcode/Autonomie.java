package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




@Autonomous(name="Autonomie")
public class Autonomie extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory trajectory1= drive.trajectoryBuilder(startPose)
                                .forward(5)
                                .strafeLeft(15)
                                .build();
        Trajectory trajectory2= drive.trajectoryBuilder(startPose)
                                .back(5)
                                .strafeRight(15)
                                .build();
        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectory(trajectory1);
        drive.followTrajectory(trajectory2);
    }
}
