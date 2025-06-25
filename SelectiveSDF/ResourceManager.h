#pragma once

#include <set>
#include "BufferHelper.hpp"

class ResourceManager
{
public:
	void Initialize(ID3D12Device* device, UINT srvCapacity, UINT rtvCapacity = 0, UINT samplerCapacity = 0);
	UINT AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse = UINT_MAX);
    UINT CreateBufferSRV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer, UINT size, UINT stride);

	ID3D12DescriptorHeap* GetSRVDescriptorHeap() const { return m_srvDescriptorHeap.Get(); }
	UINT GetSRVDescriptorSize() const { return m_srvDescriptorSize; }

private:
	ComPtr<ID3D12DescriptorHeap> m_srvDescriptorHeap, m_rtvDescriptorHeap, m_samplerDescriptorHeap;
	UINT m_srvDescriptorSize, m_rtvDescriptorSize, m_samplerDescriptorSize;
	UINT m_srvDescriptorsAllocated = 0, m_rtvDescriptorsAllocated = 0, m_samplerDescriptorsAllocated = 0;
};

// staging
inline void AllocateBuffer(ID3D12Device* pDevice, D3D12_HEAP_TYPE heapType, UINT64 bufferSize, ID3D12Resource** ppResource, D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_FLAGS resourceFlags = D3D12_RESOURCE_FLAG_NONE, void* pData = nullptr, const wchar_t* resourceName = nullptr)
{
    auto uploadHeapProperties = CD3DX12_HEAP_PROPERTIES(heapType);
    auto bufferDesc = CD3DX12_RESOURCE_DESC::Buffer(bufferSize, resourceFlags);
    ThrowIfFailed(pDevice->CreateCommittedResource(
        &uploadHeapProperties,
        D3D12_HEAP_FLAG_NONE,
        &bufferDesc,
        initialResourceState,
        nullptr,
        IID_PPV_ARGS(ppResource)));
    if (resourceName)
    {
        (*ppResource)->SetName(resourceName);
    }
    
    if (pData != nullptr)
    {
        void* pMappedData;
        (*ppResource)->Map(0, nullptr, &pMappedData);
        memcpy(pMappedData, pData, bufferSize);
        (*ppResource)->Unmap(0, nullptr);
    }
}