

// system includes
#include <fstream>
#include <QApplication>
#include <QVBoxLayout>
#include "iu/iugui.h"
#include <cuda.h>

#include "event.h"
#include "scopedtimer.h"
#include "trackingmainwindow.h"
#include "common.h"

int main(int argc, char**argv)
{

    int numDevices = 0;
    CudaSafeCall(cudaGetDeviceCount(&numDevices));
    std::cout << "found " << numDevices << " devices" << std::endl;

    for (int device = 0; device < numDevices; ++device)
    {
        cudaDeviceProp deviceProperties;
        CudaSafeCall(cudaGetDeviceProperties(&deviceProperties, device));
        std::cout << "device number=" << device << " info= " << deviceProperties.name << std::endl;
    }

    int deviceNumber = 0;
    if (numDevices > 1)
        deviceNumber = 1;

    QApplication app(argc, argv);
    TrackingMainWindow window(argv[1],deviceNumber);
    window.show();

    return app.exec();



}
