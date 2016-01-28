package com.team254.frc2016;


import com.team254.lib.util.ConstantsBase;

public class Constants extends ConstantsBase {

    // Do not change anything after this line!
    // Talons
    public static int kTurretTalonId = 1;

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

    static {
        new Constants().loadFromFile();
    }
}
