package org.firstinspires.ftc.teamcode.lib.tests;


import org.firstinspires.ftc.teamcode.util.Sorting.COLORS;

public class ColorFunctions {
    public static COLORS getColor(int r, int g, int b) {
        float total = r + g + b;
        if (total < 1)
            total = 1;
        float normRed = r / total;
        float normGreen = g / total;
        float normBlue = b / total;

        if (normRed > 0.29 && normBlue > 0.29 && normGreen > 0.29 && total > 570)
            return COLORS.GREEN;
        else if (normRed > 0.5 && normRed > normGreen && normRed > normBlue)

            return COLORS.PURPLE;
        else
            return COLORS.EMPTY;
    }

}

