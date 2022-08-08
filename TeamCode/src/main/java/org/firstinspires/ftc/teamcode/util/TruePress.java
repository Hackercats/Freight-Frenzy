package org.firstinspires.ftc.teamcode.util;

// To make something only run once a button is pressed for the firt time
public class TruePress {
    private boolean prevInput = false;

    public boolean trueInput(boolean input){
        boolean output;
        output = !prevInput && input;
        prevInput = input;
        return output;
    }
}
