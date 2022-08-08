package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class CapMech {
    private Servo gripper;
    private Servo armPivot;

    public static double GRIPPER_CLOSED_POSITION = 0;
    public static double GRIPPER_OPEN_POSITION = 1;

    public static double BASE_PIVOT_RETRACTED_POSITION = 0.05;
    public static double BASE_PIVOT_LEVEL_POSITION = 0.88;

    public void init(HardwareMap hwmap){
        gripper = hwmap.get(Servo.class, "capGripper");
        armPivot = hwmap.get(Servo.class, "capBasePivot");
        retract();
    }

    public void closeGripper(){
        gripper.setPosition(GRIPPER_CLOSED_POSITION);
    }
    public void openGripper(){
        gripper.setPosition(GRIPPER_OPEN_POSITION);
    }

    public void retractArm(){
        armPivot.setPosition(BASE_PIVOT_RETRACTED_POSITION);
    }
    public void levelArm(){
        armPivot.setPosition(BASE_PIVOT_LEVEL_POSITION);
    }

    public void retract(){
        retractArm();
        closeGripper();
    }
}
