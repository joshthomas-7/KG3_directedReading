#include "mex.h"
#include "sharedLibrary.h"

extern "C" void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    try {
        // Get API instance
        auto& api = KinovaApiWrapper::getInstance();
        if (!api.isConnected()) {
            mexErrMsgIdAndTxt("GetTorqueCalibration:NotConnected", 
                "API is not connected. Please call createAPI() first.");
            return;
        }

        int size = 0;
        auto calibration = new KinovaApiWrapper::TorqueCalibration[7];  // Max 7 actuators
        
        if (!api.getTorqueCalibration(calibration, &size)) {
            delete[] calibration;
            mexErrMsgIdAndTxt("GetTorqueCalibration:Error",
                "Failed to get torque calibration data");
            return;
        }

        // Create output structure array
        const char* fieldnames[] = {"current", "torque_gain", "torque_offset"};
        plhs[0] = mxCreateStructMatrix(1, size, 3, fieldnames);

        // Fill structure array with calibration data
        for (int i = 0; i < size; i++) {
            mxArray* current = mxCreateDoubleScalar(calibration[i].current);
            mxArray* gain = mxCreateDoubleScalar(calibration[i].torque_gain);
            mxArray* offset = mxCreateDoubleScalar(calibration[i].torque_offset);

            mxSetFieldByNumber(plhs[0], i, 0, current);
            mxSetFieldByNumber(plhs[0], i, 1, gain);
            mxSetFieldByNumber(plhs[0], i, 2, offset);
        }

        delete[] calibration;
    }
    catch (std::exception& e) {
        mexErrMsgIdAndTxt("GetTorqueCalibration:Error",
            "Unexpected error: %s", e.what());
    }
}