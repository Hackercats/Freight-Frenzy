package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Utility;

public class ArmSystem {

    private DcMotor fourBar;
    private DcMotor turret;
    Utility utility = new Utility();

    private final double FOURBAR_TICKS_PER_REV = 1425.1 * 1.5625; // *1.56 because of the external gear ratio
    private final double FOURBAR_TICKS_PER_DEGREE = FOURBAR_TICKS_PER_REV /360.0;

    private final double TURRET_TICKS_PER_REV = 384.5 * 5.3636; // *5.36 because of gear redection
    private final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV/360.0;

    // Define the safe range of the 4b
    private double fourBarSafeRangeMin = 0; // This will be increased if the turret is rotated
    private final double FOURBAR_SAFERANGE_MAX = 120;
    private final double FOURBAR_SAFE_TO_SPIN_TURRET_ANGLE = 25;

    // Define the safe range for the turret
    private final double TURRET_SAFERANGE_MIN = -90;
    private final double TURRET_SAFERANGE_MAX = 90;
    private final double TURRET_SAFE_TO_RETRACT_FOURBAR_RANGE = 3;

    private final double FOURBAR_MAX_SPEED = 0.6; // The speed the 4b will always run at while doing any movement
    private final double TURRET_MAX_SPEED = 0.6; // Max speed of the turret

    // Positions of the 4b, in degrees from zero, of the 3 levels we want it to run to
    public double level1 = 27;
    public double level2 = 54;
    public double level3 = 90;

    public int activeLevel; // The level the 4b will run to when told to raise, and the level whose offset is edited
    public double turretTargetAngle;

    public void init(HardwareMap hwmap){ //
        fourBar = hwmap.get(DcMotor.class,"fourbar");
        turret = hwmap.get(DcMotor.class,"turret");
        activeLevel = 3;
        turretTargetAngle = 0;
        zeroFourbar();
        zeroTurret();
    }

    public double FourbarAngle(){
        return (fourBar.getCurrentPosition()/FOURBAR_TICKS_PER_DEGREE);
    }

    public double TurretAngle(){
        return (turret.getCurrentPosition()/TURRET_TICKS_PER_DEGREE);
    }

    public void updateFourBarMin(){
        if (Math.abs(TurretAngle()) < TURRET_SAFE_TO_RETRACT_FOURBAR_RANGE){
            fourBarSafeRangeMin = 0;
        }
        else {
            fourBarSafeRangeMin = FOURBAR_SAFE_TO_SPIN_TURRET_ANGLE;
        }
    }

    public boolean TurretSafeToMove(){
        if (FourbarAngle() > FOURBAR_SAFE_TO_SPIN_TURRET_ANGLE) return true;
        else return false;
    }

    public void fourbarRunToAngle(double angle){// Converts angle input to ticks and runs the motor there after checking if it's safe
        updateFourBarMin();
        fourBar.setTargetPosition((int) (utility.clipValue(fourBarSafeRangeMin,FOURBAR_SAFERANGE_MAX, angle) * FOURBAR_TICKS_PER_DEGREE));
        fourBar.setPower(FOURBAR_MAX_SPEED);
        fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turretRunToAngle(double angle){
        // Check if the fourbar is high enough and that the input angle is within the safe limit before moving
        turretTargetAngle = utility.clipValue(TURRET_SAFERANGE_MIN, TURRET_SAFERANGE_MAX, turretTargetAngle);
        if (TurretSafeToMove()) {
            turret.setTargetPosition ((int) ((int) (utility.clipValue(TURRET_SAFERANGE_MIN, TURRET_SAFERANGE_MAX, angle)) * TURRET_TICKS_PER_DEGREE));
            turret.setPower(TURRET_MAX_SPEED);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void runToLevel(int level){ // Run to a level specified by an integer
        switch (level){
            case 1:
                fourbarRunToAngle(level1);
                break;
            case 2:
                fourbarRunToAngle(level2);
                break;
            case 3:
                fourbarRunToAngle(level3);
                break;
        }
    }

    public double levelToAngle(int level) {
        double output;
        switch (level){
            case 1:
                output = level1;
                break;
            case 2:
                output = level2;
                break;
            case 3:
                output = level3;
                break;
            default: // Should never happen
                output = 0;
        }
        return output;
    }

    // edit the positions of each level so drivers can account for shipping hub wobble
    public void editFourbarLevelOffset(double change){
        switch (activeLevel){
            case 1: level1 += change;
                break;

            case 2: level2 += change;
                break;

            case 3: level3 +=change;
                break;
        }
    }

    // Control both 4b and turret with one method
    public void setArmPosition(double fourbarAngle, double turretAngle){
        fourbarRunToAngle(fourbarAngle);
        turretRunToAngle(turretAngle);
    }

    public void retract(){
        setArmPosition(0,0);
    }

    public void zeroFourbar(){ // Reset the zero point (the angle where it's fully retracted)
        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void zeroTurret(){
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
