package org.firstinspires.ftc.teamcode.profiles_and_base_settings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Profiles {

    public Settings defaultProfile; // profile declaration


    LinearOpMode opMode;
    public Profiles(LinearOpMode opModeTemp){

        opMode = opModeTemp;

        defaultProfile = new Settings(opMode); // initiates the new profile based on the default settings

        // example change:      defaultProfile.forwardSensitivity *= 1.5f;
        //
        // changes to 150% of default value
        // it is often better to make changes based on a percentage of their preset values rather than setting exact values for readability









    }


}
