#include "mex.h"
#include "sharedLibrary.h"

extern "C" void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    try {
        // Create output variable (boolean)
        plhs[0] = mxCreateLogicalScalar(false);

        // Check arguments
        if (nrhs != 1) {
            mexErrMsgIdAndTxt("SendPosition:InvalidArgs", 
                "One input argument required (position array).");
            return;
        }

        // Validate input array
        if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0])) {
            mexErrMsgIdAndTxt("SendPosition:InvalidInput",
                "Input must be a real double array.");
            return;
        }

        // Get input dimensions
        mwSize rows = mxGetM(prhs[0]);
        mwSize cols = mxGetN(prhs[0]);
        
        // Ensure input is a 1D array
        if (rows != 1 && cols != 1) {
            mexErrMsgIdAndTxt("SendPosition:InvalidDimension",
                "Input must be a 1D array.");
            return;
        }

        // Get position values from input
        double* positions = mxGetPr(prhs[0]);
        int size = (rows > cols) ? rows : cols;

        // Get API instance and check connection
        auto& api = KinovaApiWrapper::getInstance();
        if (!api.isConnected()) {
            mexErrMsgIdAndTxt("SendPosition:NotConnected", 
                "API is not connected. Please call createAPI() first.");
            return;
        }

        // Send positions and set return value
        bool success = api.sendAngles(positions, size);
        mxLogical* out = mxGetLogicals(plhs[0]);
        *out = success;

        if (!success) {
            mexErrMsgIdAndTxt("SendPosition:Error", 
                "Failed to send positions to robot.");
        }
    }
    catch (std::exception& e) {
        mexErrMsgIdAndTxt("SendPosition:Error", 
            "Unexpected error: %s", e.what());
    }
}