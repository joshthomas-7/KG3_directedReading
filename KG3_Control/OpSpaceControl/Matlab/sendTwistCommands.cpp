#include "mex.h"
#include "sharedLibrary.h"

// mex_sendTwistCommand - Send Cartesian twist commands to the robot
// Usage from MATLAB:
//   success = mex_sendTwistCommand(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
//
// Inputs:
//   linear_x, linear_y, linear_z: Linear velocity components (m/s)
//   angular_x, angular_y, angular_z: Angular velocity components (rad/s)
//
// Output:
//   success: Boolean indicating if the command was sent successfully

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Check the number of input and output arguments
    if (nrhs != 6) {
        mexErrMsgIdAndTxt("MATLAB:mex_sendTwistCommand:invalidNumInputs",
                          "Six inputs required: linear_x, linear_y, linear_z, angular_x, angular_y, angular_z");
    }
    
    if (nlhs > 1) {
        mexErrMsgIdAndTxt("MATLAB:mex_sendTwistCommand:invalidNumOutputs",
                          "Only one output supported");
    }
    
    // Validate input types
    for (int i = 0; i < 6; i++) {
        if (!mxIsDouble(prhs[i]) || mxIsComplex(prhs[i]) || mxGetNumberOfElements(prhs[i]) != 1) {
            mexErrMsgIdAndTxt("MATLAB:mex_sendTwistCommand:invalidInput",
                             "All inputs must be real scalar doubles");
        }
    }
    
    // Get the twist command values from MATLAB
    double linear_x = mxGetScalar(prhs[0]);
    double linear_y = mxGetScalar(prhs[1]);
    double linear_z = mxGetScalar(prhs[2]);
    double angular_x = mxGetScalar(prhs[3]);
    double angular_y = mxGetScalar(prhs[4]);
    double angular_z = mxGetScalar(prhs[5]);
    
    // Call the KinovaApiWrapper function
    KinovaApiWrapper& api = KinovaApiWrapper::getInstance();
    bool success = api.sendTwistCommand(linear_x, linear_y, linear_z, 
                                        angular_x, angular_y, angular_z);
    
    // Return success flag to MATLAB
    plhs[0] = mxCreateLogicalScalar(success);
}