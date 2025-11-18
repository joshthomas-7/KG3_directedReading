#include "mex.h"
#include "sharedLibrary.h" // Make sure this includes the updated header
#include <vector>
#include <cstring> // Include for memcpy if needed, though loop is used

extern "C" void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Restore default output creation: empty matrix
    if (nlhs > 0) {
        plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
    }

    try {
        // Check arguments: hapticWrench = twistPDControl(xDesired, xdotDesired, pedalState)
        if (nrhs != 3) {
            mexErrMsgIdAndTxt("TwistPDControl:InvalidArgs",
                "Usage: hapticWrench[6x1] = twistPDControl(xDesired[6x1], xdotDesired[6x1], pedalState[scalar])");
            return;
        }

        // --- Input validation (same as before) ---
        for (int i = 0; i < 2; i++) { // Validate xDesired, xdotDesired
            if (!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i]) || mxGetNumberOfElements(prhs[i]) != 6) {
                 mexErrMsgIdAndTxt("TwistPDControl:InvalidInput", "xDesired and xdotDesired must be 6-element double arrays.");
                 return;
            }
        }
        if (!mxIsNumeric(prhs[2]) || mxIsComplex(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1) { // Validate pedalState
             mexErrMsgIdAndTxt("TwistPDControl:InvalidInput", "pedalState must be a scalar numeric value (0 or 1).");
             return;
        }

        // --- Get inputs (same as before) ---
        double* xDesired = mxGetPr(prhs[0]);
        double* xdotDesired = mxGetPr(prhs[1]);
        double pedalStateDouble = mxGetScalar(prhs[2]);
        int pedalState = static_cast<int>(pedalStateDouble);
        if (pedalState != 0 && pedalState != 1) {
             mexErrMsgIdAndTxt("TwistPDControl:InvalidInput", "pedalState must be 0 or 1.");
             return;
        }

        // --- Get API instance (same as before) ---
        auto& api = KinovaApiWrapper::getInstance();
        if (!api.isConnected()) {
            mexErrMsgIdAndTxt("TwistPDControl:NotConnected", "API is not connected. Please call createAPI() first.");
            return;
        }

        // Call the C++ function
        KinovaApiWrapper::RobotStateNoQuat state = api.twistPDcontrolNoQuat(xDesired, xdotDesired, pedalState);

        if (nlhs >= 1) { // Current position [3x1]
            plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
            double* out = mxGetPr(plhs[0]);  // FIXED: Match array index
            if (state.current_pos.size() == 3) {
                for (size_t i = 0; i < 3; ++i) {
                    out[i] = state.current_pos[i];
                }
            }
        }

        if (nlhs >= 2) { // Current RPY angles [3x1]
            plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL);
            double* out = mxGetPr(plhs[1]);  // FIXED: Match array index
            if (state.current_rpy.size() == 3) {
                for (size_t i = 0; i < 3; ++i) {
                    out[i] = state.current_rpy[i];
                }
            }
        }

        if (nlhs >= 3) { // Current linear velocity [3x1]
            plhs[2] = mxCreateDoubleMatrix(3, 1, mxREAL);
            double* out = mxGetPr(plhs[2]);  // FIXED: Match array index
            if (state.current_vel.size() == 3) {
                for (size_t i = 0; i < 3; ++i) {
                    out[i] = state.current_vel[i];
                }
            }
        }

        if (nlhs >= 4) { // Current angular velocity [3x1]
            plhs[3] = mxCreateDoubleMatrix(3, 1, mxREAL);
            double* out = mxGetPr(plhs[3]);  // FIXED: Match array index
            if (state.current_omega.size() == 3) {
                for (size_t i = 0; i < 3; ++i) {
                    out[i] = state.current_omega[i];
                }
            }
        }

        if (nlhs >= 5) { // Twist command [6x1]
            plhs[4] = mxCreateDoubleMatrix(6, 1, mxREAL);
            double* out = mxGetPr(plhs[4]);  // FIXED: Match array index
            if (state.twist_command.size() == 6) {
                for (size_t i = 0; i < 6; ++i) {
                    out[i] = state.twist_command[i];
                }
            }
        }
    }
    catch (std::exception& e) {
        // Set all outputs to empty on error
        for (int i = 0; i < nlhs; i++) {
            plhs[i] = mxCreateDoubleMatrix(0, 0, mxREAL);
        }
        mexErrMsgIdAndTxt("TwistPDControl:Error", "Unexpected error: %s", e.what());
    }
}