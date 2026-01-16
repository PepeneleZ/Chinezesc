package org.firstinspires.ftc.teamcode.util;

public class Constants_Enums {

    /// /////////////// SORTING /////////////// ///
    public enum TRANSFER_POS{
        UP(0.85),DOWN(0);
        final double val;
        TRANSFER_POS(double val) {
            this.val = val;
        }
    }
    public enum MOVING_STATES {
        WAITING_INTAKE("waiting_intake"), NOTHING("nothing"), SHOOTING("shooting");
        final String val;
        MOVING_STATES(String val){this.val = val;}
    }

    public enum COLORS{
        GREEN("green"), PURPLE("parpal"), EMPTY("empty");
        public final String val;
        COLORS(String val) {
            this.val = val;
        }
    }




    /// /////////////// TURRET /////////////// ///
    public enum TURRET_LAUNCH_SPEEDS{
        CLOSE(1100),FAR(1645),STOPPED(0);
        final double val;
        TURRET_LAUNCH_SPEEDS(double val) {
            this.val = val;
        }
    }
    public enum VERTICAL_TURRET_POSITIONS{
        DOWN(0),MIDDLE(0.2),UP(0.4), OTHER(0.3333);
        final double val;
        VERTICAL_TURRET_POSITIONS(double val){this.val = val;}
    }

    public enum MOTIF {
        GPP(1), PGP(2), PPG(3);
        final int val;
        MOTIF(int val){this.val = val;}
    }


    /// /////////////// INTAKE /////////////// ///
    public enum INTAKE_STATES{
        STOPPED(0),COLLECTING(1),SPITTING_OUT(-1),SLIGHTLY_MOVING(0.4);
        final double val;
        INTAKE_STATES(double val){this.val = val;}
    }
}
