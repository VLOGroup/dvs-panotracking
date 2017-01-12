#ifndef COMMON_CUH
#define COMMON_CUH

#include <iu/iucore.h>
#include <iu/iucutil.h>


namespace cuda {

  enum UpsampleMethod {
    UPSAMPLE_LINEAR,
    UPSAMPLE_NEAREST
  };

  void setEvents(iu::ImageGpu_32f_C1 *output, iu::LinearDeviceMemory_32f_C2 *events_gpu);
  void upsample(iu::ImageGpu_32f_C1 *in, iu::ImageGpu_32f_C1 *out, UpsampleMethod method, bool exponentiate = false);

}
#endif
