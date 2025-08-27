#pragma once
#include "stdafx.h"

namespace BufferHelper
{
    struct D3DBuffer
    {
        ComPtr<ID3D12Resource> resource;
        D3D12_CPU_DESCRIPTOR_HANDLE srvCpuDescriptorHandle;
        D3D12_GPU_DESCRIPTOR_HANDLE srvGpuDescriptorHandle;

        D3D12_CPU_DESCRIPTOR_HANDLE uavCpuDescriptorHandle;
        D3D12_GPU_DESCRIPTOR_HANDLE uavGpuDescriptorHandle;
    };
}
namespace ConstantBufferTypes
{
    using namespace DirectX;

    struct SceneConstantBuffer
    {
        XMMATRIX viewI;
        XMMATRIX projectionI;
        UINT triangeObjectCount;
        UINT sdfObjectCount;
        UINT triangleInstanceCount;
        UINT sdfInstanceCount;
		UINT32 frameIndex;
        UINT surfaceMode;
        UINT colorMode;
		UINT padding;
    };

    struct ComputeConstantBuffer
    {
		/*UINT candidateVoxelCount;
        XMFLOAT3 padding;*/
        UINT triangeObjectCount;
        UINT sdfObjectCount;
        UINT triangleInstanceCount;
        UINT sdfInstanceCount;
        UINT32 frameIndex;
        UINT padding[3];
	};

    struct ObjectConstantBuffer
    {
        XMMATRIX worldI;
    };
}

class GpuUploadBuffer
{
public:
    ComPtr<ID3D12Resource> GetResource() { return m_resource; }
    virtual void Release() { m_resource.Reset(); }
protected:
    ComPtr<ID3D12Resource> m_resource;

    GpuUploadBuffer() {}
    ~GpuUploadBuffer()
    {
        if (m_resource.Get())
        {
            m_resource->Unmap(0, nullptr);
        }
    }

    void Allocate(ID3D12Device* device, UINT bufferSize, LPCWSTR resourceName = nullptr)
    {
        auto uploadHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);

        auto bufferDesc = CD3DX12_RESOURCE_DESC::Buffer(bufferSize);
        ThrowIfFailed(device->CreateCommittedResource(
            &uploadHeapProperties,
            D3D12_HEAP_FLAG_NONE,
            &bufferDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&m_resource)));
        m_resource->SetName(resourceName);
    }

    uint8_t* MapCpuWriteOnly()
    {
        uint8_t* mappedData;
        // We don't unmap this until the app closes. Keeping buffer mapped for the lifetime of the resource is okay.
        CD3DX12_RANGE readRange(0, 0);        // We do not intend to read from this resource on the CPU.
        ThrowIfFailed(m_resource->Map(0, &readRange, reinterpret_cast<void**>(&mappedData)));
        return mappedData;
    }
};