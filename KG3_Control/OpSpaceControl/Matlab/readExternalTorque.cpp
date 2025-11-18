#include "mex.h"
#include "sharedLibrary.h"

extern "C" void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    try {
        mexPrintf("Starting readExternalTorque...\n");
        
        auto& api = KinovaApiWrapper::getInstance();
        if (!api.isConnected()) {
            mexErrMsgIdAndTxt("ReadTorque:NotConnected", 
                "API is not connected. Please call createAPI() first.");
            return;
        }

        // Get external torques
        int size = 0;
        double* torques = api.getExternalTorques(&size);
        
        if (!torques || size == 0) {
            mexErrMsgIdAndTxt("ReadTorque:Error", 
                "Failed to read torque values");
            return;
        }

        // Create output array
        plhs[0] = mxCreateDoubleMatrix(1, size, mxREAL);
        double* output = mxGetPr(plhs[0]);

        //Copy torque values and print
        for(int i = 0; i < size; i++) {
            output[i] = torques[i];
            // mexPrintf("Actuator %d torque: %f\n", i+1, torques[i]);
        }

        // Clean up allocated memory
        delete[] torques;
    }
    catch (std::exception& e) {
        mexErrMsgIdAndTxt("ReadTorque:Error", "Unexpected error: %s", e.what());
    }
}