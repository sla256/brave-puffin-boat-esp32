#include <math.h>
#include "power.h"
#include "power_budget.h"

// Power budget math based on one of the signmoid functions: hyberbolic tangent (tanh)
// Tanh curve goes to infinity left and right, but has a "useful" shape from -3 to +3 units on X axis
// We use tanh curve in that range, and map it to battery voltage on X, then compute desired (optimal) power
// on Y axis. Some basic math is required to translate natural tanh X and Y ranges to what we need

const int batVoltageBudgetCurveMaxMv = 12600;   // max V w/o load; o/w can drop to ~12.0; upper limit for the budget curve on X
const int batVoltageBudgetCurveMinMv = 11000;   // lower limit for the budget curve X; minimum power when voltage drops lower
const int batVoltageBudgetCurveMidpointMv = batVoltageBudgetCurveMinMv + (batVoltageBudgetCurveMaxMv - batVoltageBudgetCurveMinMv) / 2;

const int desiredPowerBudgetCurveMinW = 11;
const int desiredPowerBudgetCurveMaxW = 100;
const int desiredPowerBudgetCurveHalfRangeW = (desiredPowerBudgetCurveMaxW - desiredPowerBudgetCurveMinW) / 2;

const int budgetCurveVoltageRangeMv = batVoltageBudgetCurveMaxMv - batVoltageBudgetCurveMinMv;

const float tanhNaturalRangeX = 6.0;   // infinity really, but we only care about the useful -3 to 3 range on X axis
const float budgetCurveTanhCoefficient = tanhNaturalRangeX / budgetCurveVoltageRangeMv;

int optimalPowerBudgetTargetW = desiredPowerBudgetCurveMinW;  // cached value from the last calc; min is a starting val. on boot

void handlePowerBudget() {
    // first convert current battery voltage into tanh's "natural" X range to -3 to +3
    // could actually get beyond this range if the voltage is lower or higher - it's OK
    float tanhNaturalX = (getLastBatteryVoltageMv() - batVoltageBudgetCurveMidpointMv) * budgetCurveTanhCoefficient;

    // tanh Y values range from -1 to 1.
    // first, multiplying by half of Y range stretches the Y value;
    //    e.g. if power range was 100 W, tanh's -1 becomes -50 W; and +1 == 50 W
    // then we shift that value to always be positive: minimum plus another half range
    //    e.g. -1 becomes the minimum (10 W) and +1 becomes the max
    optimalPowerBudgetTargetW = desiredPowerBudgetCurveHalfRangeW * tanhf(tanhNaturalX) +
        desiredPowerBudgetCurveHalfRangeW + desiredPowerBudgetCurveMinW;
}

// returns desired power to spend in deciwatts
int getOptimalPowerBudgetTargetW() {
    return optimalPowerBudgetTargetW;
}