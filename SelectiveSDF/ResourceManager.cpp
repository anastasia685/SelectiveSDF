#include "stdafx.h"

#include "ResourceManager.h"

void ResourceManager::Initialize(ID3D12Device* device, UINT srvCapacity, UINT rtvCapacity, UINT samplerCapacity)
{
    D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
    srvHeapDesc.NumDescriptors = srvCapacity;
    srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    srvHeapDesc.NodeMask = 0;
    device->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&m_srvDescriptorHeap));
    NAME_D3D12_OBJECT(m_srvDescriptorHeap);
	m_srvDescriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

    if (rtvCapacity > 0)
    {
        D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
        rtvHeapDesc.NumDescriptors = rtvCapacity;
        rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        rtvHeapDesc.NodeMask = 0;
        device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&m_rtvDescriptorHeap));
        NAME_D3D12_OBJECT(m_rtvDescriptorHeap);
        m_rtvDescriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    }
    if (samplerCapacity > 0)
    {
        D3D12_DESCRIPTOR_HEAP_DESC samplerHeapDesc = {};
        samplerHeapDesc.NumDescriptors = samplerCapacity;
        samplerHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER;
        samplerHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        samplerHeapDesc.NodeMask = 0;
        device->CreateDescriptorHeap(&samplerHeapDesc, IID_PPV_ARGS(&m_samplerDescriptorHeap));
        NAME_D3D12_OBJECT(m_samplerDescriptorHeap);
        m_samplerDescriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER);
	}
}

UINT ResourceManager::AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse)
{
    auto descriptorHeapCpuBase = m_srvDescriptorHeap->GetCPUDescriptorHandleForHeapStart();
    if (descriptorIndexToUse >= m_srvDescriptorHeap->GetDesc().NumDescriptors)
    {
        ThrowIfFalse(m_srvDescriptorsAllocated < m_srvDescriptorHeap->GetDesc().NumDescriptors, L"Ran out of SRV descriptors on the heap!");
        descriptorIndexToUse = m_srvDescriptorsAllocated++;
    }
    *cpuDescriptor = CD3DX12_CPU_DESCRIPTOR_HANDLE(descriptorHeapCpuBase, descriptorIndexToUse, m_srvDescriptorSize);
    return descriptorIndexToUse;
}

UINT ResourceManager::CreateBufferSRV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer, UINT elementNumber, UINT elementSize)
{
    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Buffer.NumElements = elementNumber;
    if (elementSize == 0)
    {
        srvDesc.Format = DXGI_FORMAT_R32_TYPELESS;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_RAW;
        srvDesc.Buffer.StructureByteStride = 0;
    }
    else
    {
        srvDesc.Format = DXGI_FORMAT_UNKNOWN;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_NONE;
        srvDesc.Buffer.StructureByteStride = elementSize;
    }
    UINT descriptorIndex = AllocateDescriptor(&buffer->srvCpuDescriptorHandle);
    device->CreateShaderResourceView(buffer->resource.Get(), &srvDesc, buffer->srvCpuDescriptorHandle);
    buffer->srvGpuDescriptorHandle = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_srvDescriptorHeap->GetGPUDescriptorHandleForHeapStart(), descriptorIndex, m_srvDescriptorSize);
    return descriptorIndex;
}

UINT ResourceManager::CreateTexture3DSRV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer)
{
    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE3D;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = DXGI_FORMAT_R32_FLOAT;
    srvDesc.Texture3D.MipLevels = 1;  // All mip levels
    srvDesc.Texture3D.MostDetailedMip = 0;

    UINT descriptorIndex = AllocateDescriptor(&buffer->srvCpuDescriptorHandle);
    device->CreateShaderResourceView(buffer->resource.Get(), &srvDesc, buffer->srvCpuDescriptorHandle);
    buffer->srvGpuDescriptorHandle = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_srvDescriptorHeap->GetGPUDescriptorHandleForHeapStart(), descriptorIndex, m_srvDescriptorSize);
    return descriptorIndex;
}

UINT ResourceManager::CreateTexture2DArraySRV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer, UINT arraySize, DXGI_FORMAT format)
{
    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DARRAY;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = format;
    srvDesc.Texture3D.MipLevels = 1;  // All mip levels
    srvDesc.Texture3D.MostDetailedMip = 0;
    srvDesc.Texture2DArray.FirstArraySlice = 0;
    srvDesc.Texture2DArray.ArraySize = arraySize;

    UINT descriptorIndex = AllocateDescriptor(&buffer->srvCpuDescriptorHandle);
    device->CreateShaderResourceView(buffer->resource.Get(), &srvDesc, buffer->srvCpuDescriptorHandle);
    buffer->srvGpuDescriptorHandle = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_srvDescriptorHeap->GetGPUDescriptorHandleForHeapStart(), descriptorIndex, m_srvDescriptorSize);
    return descriptorIndex;
}

UINT ResourceManager::CreateTexture2DArrayUAV(ID3D12Device* device, BufferHelper::D3DBuffer* buffer, UINT arraySize, DXGI_FORMAT format)
{
    D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc = {};
    uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2DARRAY;
    uavDesc.Format = format;
    uavDesc.Texture2DArray.MipSlice = 0;
    uavDesc.Texture2DArray.FirstArraySlice = 0;
    uavDesc.Texture2DArray.ArraySize = arraySize;
    uavDesc.Texture2DArray.PlaneSlice = 0;

    UINT descriptorIndex = AllocateDescriptor(&buffer->uavCpuDescriptorHandle);
    device->CreateUnorderedAccessView(buffer->resource.Get(), nullptr, &uavDesc, buffer->uavCpuDescriptorHandle);
    buffer->uavGpuDescriptorHandle = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_srvDescriptorHeap->GetGPUDescriptorHandleForHeapStart(), descriptorIndex, m_srvDescriptorSize);
    return descriptorIndex;
}
