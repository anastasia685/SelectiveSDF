#pragma once

#include <set>
#include "BufferHelper.hpp"

class ResourceManager
{
public:
	void Initialize(ID3D12Device* device, UINT srvCapacity, UINT rtvCapacity = 0, UINT samplerCapacity = 0);
	UINT AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse = UINT_MAX);
    UINT CreateBufferSRV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer, UINT size, UINT stride);
    UINT CreateBufferUAV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer, UINT size, UINT stride);
    UINT CreateTexture3DSRV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer);
    UINT CreateTexture2DArraySRV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer, UINT arraySize, DXGI_FORMAT format = DXGI_FORMAT_R32_FLOAT);
    UINT CreateTexture2DArrayUAV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer, UINT arraySize, DXGI_FORMAT format = DXGI_FORMAT_R32_FLOAT);

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
    auto heapProperties = CD3DX12_HEAP_PROPERTIES(heapType);
    auto bufferDesc = CD3DX12_RESOURCE_DESC::Buffer(bufferSize, resourceFlags);
    ThrowIfFailed(pDevice->CreateCommittedResource(
        &heapProperties,
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
inline void AllocateTexture(
    ID3D12Device* pDevice, const DirectX::XMUINT3& resolution, ID3D12Resource** ppResource, 
    DXGI_FORMAT format = DXGI_FORMAT_R32_FLOAT, D3D12_RESOURCE_DIMENSION dimension = D3D12_RESOURCE_DIMENSION_TEXTURE3D, 
    D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_FLAGS resourceFlags = D3D12_RESOURCE_FLAG_NONE, 
    const wchar_t* resourceName = nullptr)
{
	auto heapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);

	D3D12_RESOURCE_DESC textureDesc = {};
	textureDesc.DepthOrArraySize = resolution.z;
	textureDesc.Dimension = dimension;
	textureDesc.Format = format;
	textureDesc.Flags = D3D12_RESOURCE_FLAG_NONE;
	textureDesc.Width = resolution.x;
	textureDesc.Height = resolution.y;
	textureDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
	textureDesc.MipLevels = 1;
	textureDesc.SampleDesc.Count = 1;
	textureDesc.Flags = resourceFlags;

	ThrowIfFailed(pDevice->CreateCommittedResource(
		&heapProperties,
		D3D12_HEAP_FLAG_NONE,
		&textureDesc,
		initialResourceState,
		nullptr,
		IID_PPV_ARGS(ppResource)));
    if (resourceName)
    {
        (*ppResource)->SetName(resourceName);
    }
}