#pragma once
#include "stdafx.h"
#include "BufferHelper.hpp"

// Helper class to create and update a structured buffer.
// Usage: 
//    StructuredBuffer<...> sb;
//    sb.Create(...);
//    sb[index].var = ... ; 
//    sb.CopyStagingToGPU(...);
//    Set...View(..., sb.GputVirtualAddress());
template <class T>
class StructuredBuffer : public GpuUploadBuffer
{
    T* m_mappedBuffers;
    std::vector<T> m_staging;
    UINT m_numInstances;

public:
    // Performance tip: Align structures on sizeof(float4) boundary.
    // Ref: https://developer.nvidia.com/content/understanding-structured-buffer-performance
    static_assert(sizeof(T) % 16 == 0, L"Align structure buffers on 16 byte boundary for performance reasons.");

    StructuredBuffer() : m_mappedBuffers(nullptr), m_numInstances(0) {}

    void Create(ID3D12Device* device, UINT numElements, UINT numInstances = 1, LPCWSTR resourceName = nullptr)
    {
        m_staging.resize(numElements);
        UINT bufferSize = numInstances * numElements * sizeof(T);
        Allocate(device, bufferSize, resourceName);
        m_mappedBuffers = reinterpret_cast<T*>(MapCpuWriteOnly());
    }

    void CopyStagingToGpu(UINT instanceIndex = 0)
    {
        memcpy(m_mappedBuffers + instanceIndex * NumElementsPerInstance(), &m_staging[0], InstanceSize());
    }

    // Accessors
    T& operator[](UINT elementIndex) { return m_staging[elementIndex]; }
    size_t NumElementsPerInstance() { return m_staging.size(); }
    UINT NumInstances() { return m_staging.size(); }
    size_t InstanceSize() { return NumElementsPerInstance() * sizeof(T); }
    D3D12_GPU_VIRTUAL_ADDRESS GpuVirtualAddress(UINT instanceIndex = 0)
    {
        return m_resource->GetGPUVirtualAddress() + instanceIndex * InstanceSize();
    }
};