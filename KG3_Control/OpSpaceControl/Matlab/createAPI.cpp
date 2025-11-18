#include "mex.h"
#include "sharedLibrary.h"

extern "C" void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    try {
        auto& api = KinovaApiWrapper::getInstance();
        if (api.isConnected()) {
            mexPrintf("API is already connected. Call deleteAPI() first.\n");
            return;
        }

        if (api.createApi("192.168.1.10")) {
            mexPrintf("API connected successfully.\n");
        } else {
            mexErrMsgIdAndTxt("CreateAPI:Error", "Failed to create API connection");
        }
    }
    catch (std::exception& e) {
        mexErrMsgIdAndTxt("CreateAPI:Error", "Error: %s", e.what());
    }
}