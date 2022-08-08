package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.AutoToTele;

public class TeleopDrive {
    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;

    private BNO055IMU imu;
    public double heading;

    private double rotX;
    private double rotY;

    BNO055IMU.Parameters imuParameters;

    public void init(HardwareMap hwmap){
        lf = hwmap.get(DcMotor.class,"lf");
        lb = hwmap.get(DcMotor.class,"lb");
        rf = hwmap.get(DcMotor.class,"rf");
        rb = hwmap.get(DcMotor.class,"rb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        //brake when you stop gamepad input
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize imu
        imu = hwmap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);
    }

    public void driveFieldCentric(double x, double y, double turn, double multiplier){

        multiplier = (-0.65*multiplier)+1;

        heading = -(imu.getAngularOrientation().firstAngle + (AutoToTele.endOfAutoHeading-Math.toRadians(90 * AutoToTele.allianceSide)));

        rotX = x * Math.cos(heading) - -y * Math.sin(heading);
        rotY = x * Math.sin(heading) + -y * Math.cos(heading);

        double lfPower = rotY + rotX + turn;
        double lbPower = rotY - rotX + turn;
        double rfPower = rotY - rotX - turn;
        double rbPower = rotY + rotX - turn;

        lf.setPower(lfPower*multiplier);
        lb.setPower(lbPower*multiplier);
        rf.setPower(rfPower*multiplier);
        rb.setPower(rbPower*multiplier);
    }

    public void driveRobotCentric(double x, double y, double turn, double multiplier){
        multiplier = (-0.65*multiplier)+1;

        double lfPower = y + x + turn;
        double lbPower = y - x + turn;
        double rfPower = y - x - turn;
        double rbPower = y + x - turn;

        lf.setPower(lfPower*multiplier);
        lb.setPower(lbPower*multiplier);
        rf.setPower(rfPower*multiplier);
        rb.setPower(rbPower*multiplier);
    }

    public void resetHeading(){
        AutoToTele.endOfAutoHeading = (Math.PI/2)*AutoToTele.allianceSide; // Unit circle coming in handy
        imu.initialize(imuParameters);
    }

    public void setWheelPowers(double LF, double LB, double RF, double RB){
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setPower(LF);
        lb.setPower(LB);
        rf.setPower(RF);
        rb.setPower(RB);
    }
}
