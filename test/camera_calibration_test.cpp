#include "CameraCalibration/CameraCalibration.h"
using namespace xb;

int main()
{
    CameraCalibration calibration(6, 9, 6, 12, "/home/xubin/Desktop/project/concentric/*.JPG");
    calibration(Concentric_Pattern());
    return 0;
}