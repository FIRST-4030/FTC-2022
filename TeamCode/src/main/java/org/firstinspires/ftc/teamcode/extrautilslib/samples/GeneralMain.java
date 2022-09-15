package org.firstinspires.ftc.teamcode.extrautilslib.samples;

import com.icaras84.extrautilslib.core.cmdsys.EULCommandSys;

public class GeneralMain {

    public static void main(String[] args) {
        //KTOverloadTest<Vector4d> ktot = new KTOverloadTest<>(new Vector4d(10, 5, 0, -5), new Vector4d(3, 3, 3, 3));

        //System.out.println(ktot.process());

        /*
        SecondOrderDynamics<Vector2d> sodV2d = new SecondOrderDynamics<>(1, 1, 1, new Vector2d(0, 0));

        System.out.println(sodV2d.update(0.5, new Vector2d(0, 3), null));
        System.out.println(sodV2d.update(0.5, new Vector2d(0, 0), null));
        System.out.println(sodV2d.update(0.5, new Vector2d(0, 3), null));

         */


        /*
        System.out.println(EULStrings.crawlToChar("[HELLO] it's me. [TEST2]", 0, ']'));
        String test = "[HI]";
        System.out.println(test.substring(1, test.length() - 1));
         */

        EULCommandSys sys = new EULCommandSys("/>");

        System.out.println(sys.parseCmd("/>hi: 0 1 2"));
        System.out.println(sys.parseCmd("/>ZA_WARUDO"));
        System.out.println(sys.parseCmd("/>root.node.motor: hello 0 45 0.3"));
    }
}
