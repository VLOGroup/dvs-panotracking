#include "direct.cuh"
#include "iu/iuhelpermath.h"

__constant__ float2 const_pp;
__constant__ float3 const_Kcaminv[3];
__constant__ float3 const_Kcam[3];
__constant__ float  const_scale;


__device__ float3 ProjectLine(float2 pos)
{
    float3 posh = make_float3(pos.x,pos.y,1.f);
    float3 p;
    p.x = dot(posh,const_Kcaminv[0]);
    p.y = dot(posh,const_Kcaminv[1]);
    p.z = dot(posh,const_Kcaminv[2]);
    return p;
}

__device__ __host__ float3 RotatePoint(float3 pos, float3* rotation)
{
    float3 point;

    point.x = dot(pos,rotation[0]);
    point.y = dot(pos,rotation[1]);
    point.z = dot(pos,rotation[2]);
    return point;
}

inline __device__ __host__ void eye3(float3* out, float scale)
{
    out[0] = make_float3(scale,0.0f,0.0f);
    out[1] = make_float3(0.0f,scale,0.0f);
    out[2] = make_float3(0.0f,0.0f,scale);
}

inline __device__ __host__ void crossmat(float3* out, float3 vec, float scale)
{
    vec*=scale;
    out[0] = make_float3(0.0,-vec.z,vec.y);
    out[1] = make_float3(vec.z,0.0,-vec.x);
    out[2] = make_float3(-vec.y,vec.x,0.0);
}

inline __device__ __host__ void outerproduct(float3* out, float3 vec, float scale)
{
    out[0] = make_float3(vec.x*vec.x, vec.x*vec.y, vec.x*vec.z)*scale;
    out[1] = make_float3(vec.y*vec.x, vec.y*vec.y, vec.y*vec.z)*scale;
    out[2] = make_float3(vec.z*vec.x, vec.z*vec.y, vec.z*vec.z)*scale;
}


__device__ __host__ void rodrigues(float3 in, float3* out)
{
    float theta = length(in);
    if(theta<1e-8f)
    {
        eye3(out,1);
        return;
    }
    float3 omega = in/theta;
    float alpha = cos(theta);
    float beta = sin(theta);
    float gamma = 1 - alpha;

    // R = eye(3)*alpha + crossmat(omega)*beta + omega*omega'*gamma
    float3 tempmat[3];
    eye3(tempmat,alpha);
    out[0] = tempmat[0];
    out[1] = tempmat[1];
    out[2] = tempmat[2];
    crossmat(tempmat,omega,beta);
    out[0] += tempmat[0];
    out[1] += tempmat[1];
    out[2] += tempmat[2];
    outerproduct(tempmat,omega,gamma);
    out[0] += tempmat[0];
    out[1] += tempmat[1];
    out[2] += tempmat[2];
}

inline __device__ int2 round(float2 p)
{
    return make_int2(round(p.x), round(p.y));
}

__device__ int2 InsideImage(float2 point, int width, int height)
{
    int2 retval = round(point);
    if(retval.x<0 || retval.x>width || retval.y<0 || retval.y>height)
    {
        retval = make_int2(-1,-1);
    }
    return retval;
}

inline __device__ __host__ float3 RotatePointSpherical(float3 pos, float3* rotation)
{
    return RotatePoint(make_float3(pos.z,pos.x,pos.y),rotation);
}

__device__ float2 ProjectMapSpherical(float3 pos)
{
    float2 point;
    float el = pos.z/sqrt(pos.x*pos.x+pos.y*pos.y);
    float az = atan2(pos.y,pos.x);

    point.x = az/M_PI*const_pp.x*const_scale+const_pp.x;
    point.y = el*const_pp.y*const_scale/(const_pp.x/const_pp.y)+const_pp.y;

    return point;
}


__global__ void updateOccurences_kernel(iu::ImageGpu_32f_C1::KernelData occurences, iu::LinearDeviceMemory_32f_C2::KernelData events, float3 pose){
    int event_id = blockIdx.x*blockDim.x + threadIdx.x;

    if(event_id<events.numel_) {
        // get last template point
        float3 R[3];
        rodrigues(pose,R);
        float2 p = ProjectMapSpherical(RotatePointSpherical(ProjectLine(events(event_id)),R));
        int2 idx = InsideImage(p,occurences.width_,occurences.height_);
        if(idx.x>=0)
            occurences(idx.x,idx.y)++;
    }
}

__global__ void updateNormalization_kernel(iu::ImageGpu_32f_C1::KernelData normalization, float3 pose, float3 old_pose, int cam_width, int cam_height){
    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x<cam_width && y < cam_height)
    {
        float3 R[3];
        rodrigues(pose,R);
        float2 p_m_curr = ProjectMapSpherical(RotatePointSpherical(ProjectLine(make_float2(x,y)),R));

        int2 curr_idx = InsideImage(p_m_curr,normalization.width_,normalization.height_);
        if(curr_idx.x>=0){
            rodrigues(old_pose,R);
            float2 p_m_old = ProjectMapSpherical(RotatePointSpherical(ProjectLine(make_float2(x,y)),R));
            normalization(curr_idx.x,curr_idx.y)+=length(p_m_old-p_m_curr);
        }
    }
}

__global__ void updateMap_kernel(iu::ImageGpu_32f_C1::KernelData map, iu::ImageGpu_32f_C1::KernelData occurences, iu::ImageGpu_32f_C1::KernelData normalization) {
    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x<map.width_ && y<map.height_) {
        map(x,y) = min(1.f,occurences(x,y)/normalization(x,y));
    }
}

__global__ void getGradients_kernel(iu::LinearDeviceMemory_32f_C4::KernelData output, cudaTextureObject_t map, iu::LinearDeviceMemory_32f_C2::KernelData events, float3 pose){
    int event_id = blockIdx.x*blockDim.x + threadIdx.x;

    if(event_id<events.numel_) {
        // get last template point
        float3 R[3];
        rodrigues(pose,R);
        float2 p = ProjectMapSpherical(RotatePointSpherical(ProjectLine(events(event_id)),R));
        const float xx = p.x+0.5;
        const float yy = p.y+0.5;
        output(event_id) = make_float4(tex2D<float>(map,xx+0.5f,yy) - tex2D<float>(map,xx-0.5f,yy),
                                       tex2D<float>(map,xx,yy+0.5f) - tex2D<float>(map,xx,yy-0.5f),
                                       tex2D<float>(map,xx,yy),
                                       0.f);
    }
}

__global__ void createOutput1_kernel(iu::ImageGpu_8u_C4::KernelData output, iu::ImageGpu_32f_C1::KernelData map)
{
    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;

    if(x<output.width_ && y < output.height_)
    {
        float in = 1.0f-min(1.0f,map(x,y));
        output(x,y) = make_uchar4(in*255,in*255,in*255,255);;
    }
}

__global__ void createOutput2_kernel(iu::ImageGpu_8u_C4::KernelData output, float3 pose, int cam_width, int cam_height, float quality)
{
    int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;

    if((x==0 && y<cam_height) || (x==cam_width-1 && y<cam_height) || y==0 || y==cam_height-1)
    {
        float3 R[3];
        rodrigues(pose,R);
        float2 p = ProjectMapSpherical(RotatePointSpherical(ProjectLine(make_float2(x,y)),R));

        int2 idx = InsideImage(p,output.width_,output.height_);
        if(idx.x>=0)
            output(idx.x,idx.y) = make_uchar4(255*(1.f-quality),255*quality,0,255);
    }
}

__global__ void createOutput3_kernel(iu::ImageGpu_8u_C4::KernelData output, iu::LinearDeviceMemory_32f_C2::KernelData events, float3 pose)
{
    int event_id = blockIdx.x*blockDim.x + threadIdx.x;;

    if(event_id<events.numel_) {
        // get last template point
        float3 R[3];
        rodrigues(pose,R);
        float2 p = ProjectMapSpherical(RotatePointSpherical(ProjectLine(events(event_id)),R));
        int2 idx = InsideImage(p,output.width_,output.height_);
        if(idx.x>=0)
            output(idx.x,idx.y) = make_uchar4(0,255,0,255);
    }
}

namespace cuda{

// -------------Interface functions-------------------------------
void setCameraMatrices(Eigen::Matrix<float, 3, 3, Eigen::RowMajor>& Kcam, Eigen::Matrix<float, 3, 3, Eigen::RowMajor>& Kcaminv, float p_x, float p_y, float scale)
{
    cudaMemcpyToSymbol(const_Kcam, Kcam.data(), 3 * sizeof(float3));
    CudaCheckError();
    cudaMemcpyToSymbol(const_Kcaminv, Kcaminv.data(), 3 * sizeof(float3));
    CudaCheckError();
    cudaMemcpyToSymbol(const_scale,&scale, sizeof(float));
    CudaCheckError();
    float2 pp = make_float2(p_x,p_y);

    cudaMemcpyToSymbol(const_pp, &pp, sizeof(float2));
    CudaCheckError();

}

void updateMap(iu::ImageGpu_32f_C1 *map, iu::ImageGpu_32f_C1 *occurences, iu::ImageGpu_32f_C1 *normalization, iu::LinearDeviceMemory_32f_C2 *events, float3 pose, float3 old_pose, int cam_width, int cam_height)
{
    int gpu_block_x = GPU_BLOCK_SIZE*GPU_BLOCK_SIZE;
    int gpu_block_y = 1;

    // compute number of Blocks
    int nb_x = iu::divUp(events->numel(),gpu_block_x);
    int nb_y = 1;

    dim3 dimBlock(gpu_block_x,gpu_block_y);
    dim3 dimGrid(nb_x,nb_y);

    updateOccurences_kernel<<<dimGrid,dimBlock>>>(*occurences,*events,pose);
    CudaCheckError();

    gpu_block_x = GPU_BLOCK_SIZE;
    gpu_block_y = GPU_BLOCK_SIZE;

    // compute number of Blocks
    nb_x = iu::divUp(cam_width,gpu_block_x);
    nb_y = iu::divUp(cam_height,gpu_block_y);

    dimBlock = dim3(gpu_block_x,gpu_block_y);
    dimGrid = dim3(nb_x,nb_y);

    updateNormalization_kernel<<<dimGrid,dimBlock>>>(*normalization,pose,old_pose,cam_width,cam_height);
    CudaCheckError();

    nb_x = iu::divUp(map->width(),gpu_block_x);
    nb_y = iu::divUp(map->height(),gpu_block_y);

    dimBlock = dim3(gpu_block_x,gpu_block_y);
    dimGrid = dim3(nb_x,nb_y);

    updateMap_kernel<<<dimGrid,dimBlock>>>(*map,*occurences,*normalization);
    CudaCheckError();
}

void getGradients(iu::LinearDeviceMemory_32f_C4 *output, iu::ImageGpu_32f_C1* map, iu::LinearDeviceMemory_32f_C2 *events, float3 pose) {
    int gpu_block_x = GPU_BLOCK_SIZE*GPU_BLOCK_SIZE;
    int gpu_block_y = 1;

    // compute number of Blocks
    int nb_x = iu::divUp(events->numel(),gpu_block_x);
    int nb_y = 1;

    dim3 dimBlock(gpu_block_x,gpu_block_y);
    dim3 dimGrid(nb_x,nb_y);

    getGradients_kernel<<<dimGrid,dimBlock>>>(*output,map->getTexture(),*events,pose);
    CudaCheckError();
}

void createOutput(iu::ImageGpu_8u_C4 *out, iu::ImageGpu_32f_C1 *map, iu::LinearDeviceMemory_32f_C2 *events, float3 pose, int cam_width, int cam_height, float quality){
    int nb_x = iu::divUp(out->width(),GPU_BLOCK_SIZE);
    int nb_y = iu::divUp(out->height(),GPU_BLOCK_SIZE);

    dim3 dimBlock(GPU_BLOCK_SIZE,GPU_BLOCK_SIZE);
    dim3 dimGrid(nb_x,nb_y);

    createOutput1_kernel<<<dimGrid,dimBlock>>>(*out,*map);

    nb_x = iu::divUp(cam_width,GPU_BLOCK_SIZE);
    nb_y = iu::divUp(cam_height,GPU_BLOCK_SIZE);

    dimBlock = dim3(GPU_BLOCK_SIZE,GPU_BLOCK_SIZE);
    dimGrid = dim3(nb_x,nb_y);
    if(quality>0)
        createOutput2_kernel<<<dimGrid,dimBlock>>>(*out,pose,cam_width,cam_height,min(quality,1.f));

    if(events) {
         nb_x = iu::divUp(events->numel(),GPU_BLOCK_SIZE);
         nb_y = 1;
         dimBlock = dim3(GPU_BLOCK_SIZE*GPU_BLOCK_SIZE,1);
         dimGrid = dim3(nb_x,nb_y);
         createOutput3_kernel<<<dimGrid,dimBlock>>>(*out,*events,pose);
    }
    CudaCheckError();
}

}
