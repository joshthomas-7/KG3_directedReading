#include "mex.h"
#include "sharedLibrary.h"

extern "C" void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    try {
        auto& api = KinovaApiWrapper::getInstance();
        if (!api.isConnected()) {
            mexPrintf("API is not connected. Nothing to clean up.\n");
            return;
        }

        if (api.deleteApi()) {
            mexPrintf("API disconnected successfully.\n");
        } else {
            mexErrMsgIdAndTxt("DeleteAPI:Error", "Failed to delete API connection");
        }
    }
    catch (std::exception& e) {
        mexErrMsgIdAndTxt("DeleteAPI:Error", "Error: %s", e.what());
    }
}