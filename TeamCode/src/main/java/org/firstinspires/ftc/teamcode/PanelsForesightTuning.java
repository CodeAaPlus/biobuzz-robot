package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
public class PanelsForesightTuning {

    public static double headingKP = 14.201941246246868;

    public static double forwardPrimaryKP = 1.3154302879171047;
    public static double forwardSecondaryKP = 0.4860163424407298;

    public static double strafePrimaryKP = 2.6218461576982968;
    public static double strafeSecondaryKP = 0.9687020982499283;

    // Hold Test scaling
    public static double holdTranslationScale = 0.45;
    public static double holdHeadingScale = 0.35;

    private static double lastHeading = Double.NaN;
    private static double lastForwardP = Double.NaN;
    private static double lastForwardS = Double.NaN;
    private static double lastStrafeP = Double.NaN;
    private static double lastStrafeS = Double.NaN;
    private static double lastHoldT = Double.NaN;
    private static double lastHoldH = Double.NaN;

    public static void apply() {
        ForesightConfig c = Constants.foresightConfig;

        if (valid(headingKP) && headingKP != lastHeading) {
            c.headingFeedback.set(
                Controller.proportional(headingKP)
            );
            lastHeading = headingKP;
        }

        if (valid(forwardPrimaryKP) &&
            valid(forwardSecondaryKP) &&
            (forwardPrimaryKP != lastForwardP ||
             forwardSecondaryKP != lastForwardS)) {

            c.forwardTranslational.set(
                Controller.piecewise(
                    Controller.proportional(forwardSecondaryKP)
                ).put(
                    2.5,
                    Controller.proportional(forwardPrimaryKP)
                )
            );

            lastForwardP = forwardPrimaryKP;
            lastForwardS = forwardSecondaryKP;
        }

        if (valid(strafePrimaryKP) &&
            valid(strafeSecondaryKP) &&
            (strafePrimaryKP != lastStrafeP ||
             strafeSecondaryKP != lastStrafeS)) {

            c.strafeTranslational.set(
                Controller.piecewise(
                    Controller.proportional(strafeSecondaryKP)
                ).put(
                    2.5,
                    Controller.proportional(strafePrimaryKP)
                )
            );

            lastStrafeP = strafePrimaryKP;
            lastStrafeS = strafeSecondaryKP;
        }

        if (valid(holdTranslationScale) &&
            holdTranslationScale != lastHoldT) {
            c.holdPointTranslationalScaling.set(
                holdTranslationScale
            );
            lastHoldT = holdTranslationScale;
        }

        if (valid(holdHeadingScale) &&
            holdHeadingScale != lastHoldH) {
            c.holdPointHeadingScaling.set(
                holdHeadingScale
            );
            lastHoldH = holdHeadingScale;
        }
    }

    private static boolean valid(double value) {
        return Double.isFinite(value) && value >= 0;
    }
}
