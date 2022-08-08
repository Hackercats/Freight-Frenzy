package org.firstinspires.ftc.teamcode.util;

public class Utility {
    // Takes an input and if it is outside the range, make it inside the range
    public double clipValue(double min, double max, double input){
        return Math.max(min, Math.min(input, max)); // Copied from stack overflow
    }
}
