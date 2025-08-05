#include "stdafx.h"

#include "SelectiveSDF.h"

using namespace std;
using namespace DX;

// Shader entry points.
const wchar_t* SelectiveSDF::c_raygenShaderName = L"RayGen";
const wchar_t* SelectiveSDF::c_intersectionShaderNames[] =
{
    L"Intersection_SDF",
};
const wchar_t* SelectiveSDF::c_closestHitShaderNames[] =
{
    L"ClosestHit_Triangle",
    L"ClosestHit_AABB",
};
const wchar_t* SelectiveSDF::c_missShaderNames[] =
{
    L"Miss"
};
// Hit groups.
const wchar_t* SelectiveSDF::c_hitGroupNames_TriangleGeometry[] =
{
    L"HitGroup_Triangle"
};
const wchar_t* SelectiveSDF::c_hitGroupNames_AABBGeometry[][RayType::Count] =
{
    { L"HitGroup_SDF" },
};

SelectiveSDF::SelectiveSDF(UINT width, UINT height, std::wstring name) : DXSample(width, height, name)
{
    m_resourceManager = make_unique<ResourceManager>();

	unique_ptr<Plane> plane = make_unique<Plane>();
    // plane instances
    {
        m_instances.push_back({ 0, { 0, -0.5, 0 }, { 0, 0, 0 }, 50 });
    }

    unique_ptr<Box> box = make_unique<Box>();
    // box instances
    {
        /*m_instances.push_back({ 1, { -0.7, 0, 0 }, { 0, 0, 0 }, 1 });

        SDFInstanceCount += 1;*/
    }

    unique_ptr<Sphere> sphere = make_unique<Sphere>();
    // sphere instances
    {
        /*m_instances.push_back({ 1, {  0.7, 0, 0}, {0, 0, 0}, 0.3 });

        SDFInstanceCount += 1;*/
    }

	unique_ptr<AModel> aModel = make_unique<AModel>("monkey");
    // monkey instances
    {
        /*m_instances.push_back({ 3, { 0, 0, 0 }, {0, 0, 0}, 1 });
        SDFInstanceCount += 1;*/

        m_instances.push_back({ 1, { -1.2, 0, 0 }, { 0, 0, 0 }, 1 });
        m_instances.push_back({ 1, { 0, 0, 0 }, { 0, 0, 0 }, 1 });

        SDFInstanceCount += 2;

        /*for (int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 10; j++)
            {
                m_instances.push_back({ 1, { -5.4f + i * 0.7f, 0, -5.4f + j * 0.7f }, { 0, 0, 0 }, 1 });
            }
        }
        SDFInstanceCount += 100;*/
    }


    m_objects.push_back(move(plane));
    //m_objects.push_back(move(box));
    //m_objects.push_back(move(sphere));
    m_objects.push_back(move(aModel));
    SDFObjectCount += 1;
	SDFModelObjectCount += 1; // AModel is a model type

    //BuildSpatialHashGrid();
    BuildSDfBVH();

	UpdateForSizeChange(width, height);
}

void SelectiveSDF::OnInit()
{
    m_deviceResources = std::make_unique<DeviceResources>(
        DXGI_FORMAT_R8G8B8A8_UNORM,
        DXGI_FORMAT_UNKNOWN,
        FrameCount,
        D3D_FEATURE_LEVEL_11_0,
        // Sample shows handling of use cases with tearing support, which is OS dependent and has been supported since TH2.
        // Since the sample requires build 1809 (RS5) or higher, we don't need to handle non-tearing cases.
        DeviceResources::c_RequireTearingSupport,
        m_adapterIDoverride
    );
    m_deviceResources->RegisterDeviceNotify(this);
    m_deviceResources->SetWindow(Win32Application::GetHwnd(), m_width, m_height);
    m_deviceResources->InitializeDXGIAdapter();

    ThrowIfFalse(CheckRaytracingSupport(m_deviceResources->GetAdapter()),
        L"ERROR: DirectX Raytracing is not supported by your OS, GPU and/or driver.\n\n");

    m_deviceResources->CreateDeviceResources();
    m_deviceResources->CreateWindowSizeDependentResources();

    ThrowIfFalse(CheckRayQuerySupport(m_deviceResources->GetD3DDevice()), L"ERROR: RayQuery inside intersection shaders by your OS, GPU and/or driver.\n\n");

    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();

    InitializeScene();
}

void SelectiveSDF::OnUpdate()
{
    m_timer.Tick();
    CalculateFrameStats();

    float translationX = sinf(m_timer.GetTotalSeconds() * 0.4) * 1.0;
    m_instances[1].position = XMFLOAT3(translationX, 0, 0);

    //TODO: reconstruct camera matrices and update camera cb after implementing camera movement

    UpdateSDFInstancesData();
}

void SelectiveSDF::OnRender()
{
    if (!m_deviceResources->IsWindowVisible())
    {
        return;
    }

    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();

    // Begin frame.
    m_deviceResources->Prepare();


    Compute();
    Raytrace();
    CopyRaytracingOutputToBackbuffer();

    // End frame.

    m_deviceResources->Present(D3D12_RESOURCE_STATE_PRESENT);
}

void SelectiveSDF::OnDestroy()
{
    m_deviceResources->WaitForGpu();
    OnDeviceLost();
}

void SelectiveSDF::OnDeviceLost()
{
	ReleaseWindowSizeDependentResources();
	ReleaseDeviceDependentResources();
}

void SelectiveSDF::OnDeviceRestored()
{
	CreateDeviceDependentResources();
	CreateWindowSizeDependentResources();
}

void SelectiveSDF::CreateDeviceDependentResources()
{
    CreateAuxilaryDeviceResources();

    // Create raytracing interfaces: raytracing device and commandlist.
    CreateRaytracingInterfaces();

    // Create root signatures for the shaders.
    CreateRootSignatures();
    CreateComputeRootSignatures();

    // Create a raytracing pipeline state object which defines the binding of shaders, state and resources to be used during raytracing.
    CreateRaytracingPipelineStateObject();

    CreateComputePipelineStateObject();

    // Create a heap for descriptors.
    m_resourceManager->Initialize(m_deviceResources->GetD3DDevice(), 32);

    // Create constant buffers for the scene.
    CreateConstantBuffers();

    CreateStructuredBuffers();

    // Build geometry to be used in the sample
    BuildGeometry();

    //BuildBrickAtlas();
    //_BuildBrickAtlas();

    CreateComputeOutputResource();

    // Build raytracing acceleration structures from the generated geometry.
    BuildAccelerationStructures();

    // Build shader tables, which define shaders and their local root arguments.
    BuildShaderTables();
}

void SelectiveSDF::CreateWindowSizeDependentResources()
{
    // Create an output 2D texture to store the raytracing result to.
    CreateRaytracingOutputResource();

    m_camera.Initialize(m_aspectRatio);
}

void SelectiveSDF::ReleaseDeviceDependentResources()
{
    m_dxrDevice.Reset();
	m_dxrCommandList.Reset();

    m_raytracingGlobalRootSignature.Reset();

    m_sceneCB.Release();
	m_sdfObjectsSB.Release();
    m_sdfInstancesSB.Release();
    m_brickTableSB.Release();

    /*m_hashTableSB.Release();
    m_instanceIndicesSB.Release();*/

    //ResetComPtrArray(&m_bottomLevelAS);
    m_bottomLevelAS.clear();
    //m_topLevelAS.reset();

    m_brickTextureBuffers.clear();

    m_raytracingOutput.Reset();
    m_raytracingOutputResourceUAVDescriptorHeapIndex = UINT_MAX;
    m_rayGenShaderTable.Reset();
    m_missShaderTable.Reset();
    m_hitGroupShaderTable.Reset();

    m_computeOutput.Reset();
	m_computeOutputResourceUAVDescriptorHeapIndex = UINT_MAX;

    m_resourceManager.reset();
    m_objects.clear();
}

void SelectiveSDF::InitializeScene()
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    //TODO: set initial camera position and orientation
    UpdateCameraMatrices();

    BuildSDFObjectsData();

    //BuildHashGridData();

    BuildBVHData();
}

void SelectiveSDF::Raytrace()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    //UpdateDynamicBLAS();
    UpdateTopLevelAS();

    auto DispatchRays = [&](auto* raytracingCommandList, auto* stateObject, auto* dispatchDesc)
    {
        dispatchDesc->HitGroupTable.StartAddress = m_hitGroupShaderTable->GetGPUVirtualAddress();
        dispatchDesc->HitGroupTable.SizeInBytes = m_hitGroupShaderTable->GetDesc().Width;
        dispatchDesc->HitGroupTable.StrideInBytes = m_hitGroupShaderTableStrideInBytes;
        dispatchDesc->MissShaderTable.StartAddress = m_missShaderTable->GetGPUVirtualAddress();
        dispatchDesc->MissShaderTable.SizeInBytes = m_missShaderTable->GetDesc().Width;
        dispatchDesc->MissShaderTable.StrideInBytes = m_missShaderTableStrideInBytes;
        dispatchDesc->RayGenerationShaderRecord.StartAddress = m_rayGenShaderTable->GetGPUVirtualAddress();
        dispatchDesc->RayGenerationShaderRecord.SizeInBytes = m_rayGenShaderTable->GetDesc().Width;
        dispatchDesc->Width = m_width;
        dispatchDesc->Height = m_height;
        dispatchDesc->Depth = 1;
        raytracingCommandList->SetPipelineState1(stateObject);

        raytracingCommandList->DispatchRays(dispatchDesc);
    };
	auto descriptorHeap = m_resourceManager->GetSRVDescriptorHeap();
    auto SetCommonPipelineState = [&](auto* descriptorSetCommandList)
    {
        descriptorSetCommandList->SetDescriptorHeaps(1, &descriptorHeap);
        // Set index and successive vertex buffer decriptor tables.
        commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::OutputView, m_raytracingOutputResourceUAVGpuDescriptor);
        commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::VertexBuffers, m_objects[0]->GetIndexBuffer().srvGpuDescriptorHandle);
        commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::SDFVoxels, m_sdfVoxelsGpuDescriptor);
        //commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::SDFLeafAtlas, m_sdfLeafAtlasGpuDescriptor);
        //commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::SDFLeafNodes, m_sdfLeafNodeGpuDescriptor);
        //commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::SDFInternal1Nodes, m_sdfInternal1NodeGpuDescriptor);
        //commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::SDFInternal2Nodes, m_sdfInternal2NodeGpuDescriptor);
        //commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::SDFRootNodes, m_sdfRootNodeGpuDescriptor);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::SDFBricks, m_brickMetaSB.GpuVirtualAddress(0));
        commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::SDFTextures, m_sdfTexturesGpuDescriptor);
        commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::SDFBrickTextures, m_sdfBricksSRVGpuDescriptor);
        //commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::BrickAtlas, m_brickAtlasBuffer.gpuDescriptorHandle);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::SDFObjectsData, m_sdfObjectsSB.GpuVirtualAddress(0));
        //commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::BrickTable, m_brickTableSB.GpuVirtualAddress(0));
    };

    commandList->SetComputeRootSignature(m_raytracingGlobalRootSignature.Get());

    // Copy dynamic buffers to GPU.
    {
		m_sceneCB.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootConstantBufferView(GlobalRootSignature::Slot::SceneConstant, m_sceneCB.GpuVirtualAddress(frameIndex));

        //m_sdfInstancesSB.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::SDFInstancesData, m_sdfInstancesSB.GpuVirtualAddress(frameIndex));


        m_bvhNodesSB.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::BVH, m_bvhNodesSB.GpuVirtualAddress(frameIndex));
        m_instanceIndicesSB.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::InstanceIndices, m_instanceIndicesSB.GpuVirtualAddress(frameIndex));

        /*m_hashTableSB.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::HashTable, m_hashTableSB.GpuVirtualAddress(frameIndex));
        m_instanceIndicesSB.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::InstanceIndices, m_instanceIndicesSB.GpuVirtualAddress(frameIndex));*/
    }

    // Bind the heaps, acceleration structure and dispatch rays.  
    D3D12_DISPATCH_RAYS_DESC dispatchDesc = {};
    SetCommonPipelineState(commandList);
    commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::AccelerationStructure, m_topLevelAS->accelerationStructure->GetGPUVirtualAddress());
    DispatchRays(m_dxrCommandList.Get(), m_dxrStateObject.Get(), &dispatchDesc);
}

void SelectiveSDF::Compute()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    auto descriptorHeap = m_resourceManager->GetSRVDescriptorHeap();
    auto SetCommonPipelineState = [&](auto* descriptorSetCommandList)
    {
        descriptorSetCommandList->SetDescriptorHeaps(1, &descriptorHeap);
        // Set index and successive vertex buffer decriptor tables.
        //commandList->SetComputeRootDescriptorTable(Compute::GlobalRootSignature::Slot::OutputAABBs, m_computeOutputResourceUAVGpuDescriptor);
        commandList->SetComputeRootDescriptorTable(Compute::GlobalRootSignature::Slot::OutputBrickTextures, m_sdfBricksUAVGpuDescriptor);
        //commandList->SetComputeRootShaderResourceView(Compute::GlobalRootSignature::Slot::CandidateVoxels, m_candidateVoxelsSB.GpuVirtualAddress(0));
        commandList->SetComputeRootShaderResourceView(Compute::GlobalRootSignature::Slot::CandidateBricks, m_brickMetaSB.GpuVirtualAddress(0));
        commandList->SetComputeRootShaderResourceView(Compute::GlobalRootSignature::Slot::SDFObjectsData, m_sdfObjectsSB.GpuVirtualAddress(0));
        commandList->SetComputeRootDescriptorTable(Compute::GlobalRootSignature::Slot::SDFTextures, m_sdfTexturesGpuDescriptor);
        commandList->SetComputeRootConstantBufferView(Compute::GlobalRootSignature::Slot::ComputeConstant, m_computeCB.GpuVirtualAddress(0));
    };

    commandList->SetComputeRootSignature(m_computeGlobalRootSignature.Get());

    {
        m_sdfInstancesSB.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootShaderResourceView(Compute::GlobalRootSignature::Slot::SDFInstancesData, m_sdfInstancesSB.GpuVirtualAddress(frameIndex));
    }

    SetCommonPipelineState(commandList);

    // Dispatch compute shader
    //uint32_t numCandidateVoxels = 100 * 6000;
    uint32_t threadGroupSize = 1000;
    //uint32_t numThreadGroups = (m_candidateVoxelCount + threadGroupSize - 1) / threadGroupSize;

    commandList->SetPipelineState(m_computeStateObject.Get());
    commandList->Dispatch(m_brickMetaSB.NumElementsPerInstance(), 1, 1);

    //D3D12_RESOURCE_BARRIER uavBarrier = CD3DX12_RESOURCE_BARRIER::UAV(m_computeOutput.Get());
    //commandList->ResourceBarrier(1, &uavBarrier);

    vector<D3D12_RESOURCE_BARRIER> barriers(SDFInstanceCount);
    for (UINT i = 0; i < SDFInstanceCount; i++)
    {
        barriers[i] = CD3DX12_RESOURCE_BARRIER::Transition(
            m_brickTextureBuffers[i].resource.Get(),
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
			D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE
        );
	}
    commandList->ResourceBarrier(SDFInstanceCount, barriers.data());
}

void SelectiveSDF::CopyRaytracingOutputToBackbuffer()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto renderTarget = m_deviceResources->GetRenderTarget();

    D3D12_RESOURCE_BARRIER preCopyBarriers[2];
    preCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_COPY_DEST);
    preCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(m_raytracingOutput.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE);
    commandList->ResourceBarrier(ARRAYSIZE(preCopyBarriers), preCopyBarriers);

    commandList->CopyResource(renderTarget, m_raytracingOutput.Get());

    D3D12_RESOURCE_BARRIER postCopyBarriers[2];
    postCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT);
    postCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(m_raytracingOutput.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    commandList->ResourceBarrier(ARRAYSIZE(postCopyBarriers), postCopyBarriers);
}

void SelectiveSDF::CreateAuxilaryDeviceResources()
{
    /*auto device = m_deviceResources->GetD3DDevice();
    auto commandQueue = m_deviceResources->GetCommandQueue();*/

	
}

void SelectiveSDF::CreateRaytracingInterfaces()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();

    ThrowIfFailed(device->QueryInterface(IID_PPV_ARGS(&m_dxrDevice)), L"Couldn't get DirectX Raytracing interface for the device.\n");
    ThrowIfFailed(commandList->QueryInterface(IID_PPV_ARGS(&m_dxrCommandList)), L"Couldn't get DirectX Raytracing interface for the command list.\n");
}

void SelectiveSDF::CreateRootSignatures()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Global Root Signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    {
        CD3DX12_DESCRIPTOR_RANGE ranges[8]; // Perfomance TIP: Order from most frequent to least frequent.
        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output texture
        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, m_objects.size() * 2, 7, 0);  // index and vertex buffers per object
        //ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, SDFInstanceCount, 7, 1);
        ranges[2].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, SDFModelObjectCount, 7, 1);
        ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, SDFModelObjectCount, 7, 2);
        ranges[4].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, SDFInstanceCount, 7, 3);
        //ranges[3].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, SDFModelObjectCount, 7, 2);
        //ranges[4].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, SDFModelObjectCount, 7, 3);
        //ranges[5].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, SDFModelObjectCount, 7, 4);
        //ranges[6].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, SDFModelObjectCount, 7, 5);
        ranges[7].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 6); // brick atlas texture

        CD3DX12_STATIC_SAMPLER_DESC staticSamplers[2];
        staticSamplers[0].Init(
            0,
            D3D12_FILTER_MIN_MAG_MIP_LINEAR,
            D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            D3D12_TEXTURE_ADDRESS_MODE_CLAMP
        );
        staticSamplers[1].Init(
            1,
            D3D12_FILTER_MIN_MAG_MIP_POINT,
            D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            D3D12_TEXTURE_ADDRESS_MODE_CLAMP
        );

        CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignature::Slot::Count];
        rootParameters[GlobalRootSignature::Slot::OutputView].InitAsDescriptorTable(1, &ranges[0]);
        rootParameters[GlobalRootSignature::Slot::AccelerationStructure].InitAsShaderResourceView(0);
        rootParameters[GlobalRootSignature::Slot::SceneConstant].InitAsConstantBufferView(0);
        //rootParameters[GlobalRootSignature::Slot::AABBattributeBuffer].InitAsShaderResourceView(3);
        rootParameters[GlobalRootSignature::Slot::VertexBuffers].InitAsDescriptorTable(1, &ranges[1]);
        rootParameters[GlobalRootSignature::Slot::SDFVoxels].InitAsDescriptorTable(1, &ranges[2]);
        //rootParameters[GlobalRootSignature::Slot::SDFLeafAtlas].InitAsDescriptorTable(1, &ranges[2]);
        rootParameters[GlobalRootSignature::Slot::SDFTextures].InitAsDescriptorTable(1, &ranges[3]);
        //rootParameters[GlobalRootSignature::Slot::SDFLeafNodes].InitAsDescriptorTable(1, &ranges[3]);
        rootParameters[GlobalRootSignature::Slot::SDFBrickTextures].InitAsDescriptorTable(1, &ranges[4]);
        //rootParameters[GlobalRootSignature::Slot::SDFInternal1Nodes].InitAsDescriptorTable(1, &ranges[4]);
        //rootParameters[GlobalRootSignature::Slot::SDFInternal2Nodes].InitAsDescriptorTable(1, &ranges[5]);
        //rootParameters[GlobalRootSignature::Slot::SDFRootNodes].InitAsDescriptorTable(1, &ranges[6]);
        //rootParameters[GlobalRootSignature::Slot::BrickAtlas].InitAsDescriptorTable(1, &ranges[7]);
        rootParameters[GlobalRootSignature::Slot::SDFObjectsData].InitAsShaderResourceView(1);
        rootParameters[GlobalRootSignature::Slot::SDFInstancesData].InitAsShaderResourceView(2);
        rootParameters[GlobalRootSignature::Slot::SDFBricks].InitAsShaderResourceView(3);
        //rootParameters[GlobalRootSignature::Slot::BrickTable].InitAsShaderResourceView(3);
        rootParameters[GlobalRootSignature::Slot::BVH].InitAsShaderResourceView(4);
        rootParameters[GlobalRootSignature::Slot::InstanceIndices].InitAsShaderResourceView(5);


        CD3DX12_ROOT_SIGNATURE_DESC globalRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        globalRootSignatureDesc.NumStaticSamplers = 2;
        globalRootSignatureDesc.pStaticSamplers = staticSamplers;

        SerializeAndCreateRaytracingRootSignature(globalRootSignatureDesc, &m_raytracingGlobalRootSignature);
    }

    // Local Root Signature
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.
    {
        // Triangle geometry
        {
            //CD3DX12_DESCRIPTOR_RANGE ranges[1]; // Perfomance TIP: Order from most frequent to least frequent.
            //ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 1);  // 2 static index and vertex buffers.

            //namespace RootSignatureSlots = LocalRootSignature::Triangle::Slot;
            //CD3DX12_ROOT_PARAMETER rootParameters[RootSignatureSlots::Count];
            //rootParameters[RootSignatureSlots::VertexBuffers].InitAsDescriptorTable(1, &ranges[0]);

            //CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
            //localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
            //SerializeAndCreateRaytracingRootSignature(localRootSignatureDesc, &m_raytracingLocalRootSignature[LocalRootSignature::Type::Triangle]);
        }

        // AABB geometry
        {
            namespace RootSignatureSlots = LocalRootSignature::AABB::Slot;
            CD3DX12_ROOT_PARAMETER rootParameters[RootSignatureSlots::Count];
            rootParameters[RootSignatureSlots::ObjectConstant].InitAsConstants(SizeOfInUint32(ObjectConstantBuffer), 1);

            CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
            localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
            SerializeAndCreateRaytracingRootSignature(localRootSignatureDesc, &m_raytracingLocalRootSignature[LocalRootSignature::Type::AABB]);
        }
    }
}

void SelectiveSDF::CreateComputeRootSignatures()
{
    auto device = m_deviceResources->GetD3DDevice();

    // Global Root Signature
    {
        CD3DX12_DESCRIPTOR_RANGE ranges[2]; // Perfomance TIP: Order from most frequent to least frequent.
        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);
        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, SDFModelObjectCount, 3, 0); // sdf textures



        CD3DX12_STATIC_SAMPLER_DESC staticSamplers[1];
        staticSamplers[0].Init(
            0,
            D3D12_FILTER_MIN_MAG_MIP_LINEAR,
            D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            D3D12_TEXTURE_ADDRESS_MODE_CLAMP
        );

        CD3DX12_ROOT_PARAMETER rootParameters[Compute::GlobalRootSignature::Slot::Count];
        //rootParameters[Compute::GlobalRootSignature::Slot::OutputAABBs].InitAsDescriptorTable(1, &ranges[0]);
        rootParameters[Compute::GlobalRootSignature::Slot::OutputBrickTextures].InitAsDescriptorTable(1, &ranges[0]);
        rootParameters[Compute::GlobalRootSignature::Slot::CandidateBricks].InitAsShaderResourceView(0);
        rootParameters[Compute::GlobalRootSignature::Slot::SDFObjectsData].InitAsShaderResourceView(1);
        rootParameters[Compute::GlobalRootSignature::Slot::SDFInstancesData].InitAsShaderResourceView(2);
		rootParameters[Compute::GlobalRootSignature::Slot::SDFTextures].InitAsDescriptorTable(1, &ranges[1]);
        rootParameters[Compute::GlobalRootSignature::Slot::ComputeConstant].InitAsConstantBufferView(0);


        CD3DX12_ROOT_SIGNATURE_DESC globalRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        globalRootSignatureDesc.NumStaticSamplers = 1;
        globalRootSignatureDesc.pStaticSamplers = staticSamplers;

        SerializeAndCreateRaytracingRootSignature(globalRootSignatureDesc, &m_computeGlobalRootSignature);
    }
}

void SelectiveSDF::SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig)
{
    auto device = m_deviceResources->GetD3DDevice();

    ComPtr<ID3DBlob> blob;
    ComPtr<ID3DBlob> error;
    ThrowIfFailed(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &error), error ? static_cast<wchar_t*>(error->GetBufferPointer()) : nullptr);
    ThrowIfFailed(device->CreateRootSignature(1, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(rootSig->GetAddressOf())));
}

void SelectiveSDF::CreateRaytracingPipelineStateObject()
{
    // Create subobjects that combine into a RTPSO
    CD3DX12_STATE_OBJECT_DESC raytracingPipeline{ D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE };

    // DXIL library
	CreateDxilLibrarySubobjects(&raytracingPipeline);

    // Hit groups
	CreateHitGroupSubobjects(&raytracingPipeline);

    // Shader config
    // Defines the maximum sizes in bytes for the ray rayPayload and attribute structure.
    auto shaderConfig = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_SHADER_CONFIG_SUBOBJECT>();
    UINT payloadSize = max(sizeof(RayPayload), sizeof(ShadowRayPayload));
    UINT attributeSize = sizeof(struct ProceduralPrimitiveAttributes);
    shaderConfig->Config(payloadSize, attributeSize);

    // Local root signature and shader association
    // TODO: Add this when i have local root signatures.

    // Local root signature and shader association
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.
    CreateLocalRootSignatureSubobjects(&raytracingPipeline);

    // Global root signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    auto globalRootSignature = raytracingPipeline.CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
    globalRootSignature->SetRootSignature(m_raytracingGlobalRootSignature.Get());

    // Pipeline config
    // Defines the maximum TraceRay() recursion depth.
    auto pipelineConfig = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_PIPELINE_CONFIG_SUBOBJECT>();
    UINT maxRecursionDepth = MAX_RAY_RECURSION_DEPTH;
    pipelineConfig->Config(maxRecursionDepth);

    // Create the state object.
    ThrowIfFailed(m_dxrDevice->CreateStateObject(raytracingPipeline, IID_PPV_ARGS(&m_dxrStateObject)), L"Couldn't create DirectX Raytracing state object.\n");
}

void SelectiveSDF::CreateComputePipelineStateObject()
{
    auto device = m_deviceResources->GetD3DDevice();

    auto csBlob = CompileShaderLibrary(L"res/shaders/SurfaceExtract.hlsl", L"CS", L"cs_6_5");

    D3D12_COMPUTE_PIPELINE_STATE_DESC computePipeline = {};

    computePipeline.pRootSignature = m_computeGlobalRootSignature.Get(); // reuse root sig
    computePipeline.CS = { csBlob->GetBufferPointer(), csBlob->GetBufferSize() };
    device->CreateComputePipelineState(&computePipeline, IID_PPV_ARGS(&m_computeStateObject));
}

void SelectiveSDF::CreateDxilLibrarySubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    //auto lib = raytracingPipeline->CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();

    //--- For when shaders are precompiled into bytecode to avoid runtime compilation overhead and dependencies.
    /*D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE((void*)g_pRaytracing, ARRAYSIZE(g_pRaytracing));
    lib->SetDXILLibrary(&libdxil);*/
    // Use default shader exports for a DXIL library/collection subobject ~ surface all shaders.


    // Library 1: RayGen
    {
        auto lib = raytracingPipeline->CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
        //m_rayGenShaderBlob = CompileShaderLibrary(L"res/shaders/RayGen.hlsl");
        auto pBlob = CompileShaderLibrary(L"res/shaders/RayGen.hlsl");
        D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE(pBlob->GetBufferPointer(), pBlob->GetBufferSize());
        lib->SetDXILLibrary(&libdxil);
    }

    // Library 2: Miss
    {
        auto lib = raytracingPipeline->CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
        //m_missShaderBlob = CompileShaderLibrary(L"res/shaders/Miss.hlsl");
        auto pBlob = CompileShaderLibrary(L"res/shaders/Miss.hlsl");
        D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE(pBlob->GetBufferPointer(), pBlob->GetBufferSize());
        lib->SetDXILLibrary(&libdxil);
    }

    // Library 3: Triangle object hit shaders
    {
        auto lib = raytracingPipeline->CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
        //m_triangleShaderBlob = CompileShaderLibrary(L"res/shaders/Triangle.hlsl");
        auto pBlob = CompileShaderLibrary(L"res/shaders/Triangle.hlsl");
        D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE(pBlob->GetBufferPointer(), pBlob->GetBufferSize());
        lib->SetDXILLibrary(&libdxil);
    }

    // Library 4: Procedural object shaders
    {
        auto lib = raytracingPipeline->CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
        //m_proceduralShaderBlob = CompileShaderLibrary(L"res/shaders/Procedural.hlsl");
        auto pBlob = CompileShaderLibrary(L"res/shaders/Procedural.hlsl");
        D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE(pBlob->GetBufferPointer(), pBlob->GetBufferSize());
        lib->SetDXILLibrary(&libdxil);
    }
}

void SelectiveSDF::CreateHitGroupSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    // Triangle geometry hit groups
    {
        for (UINT rayType = 0; rayType < RayType::Count; rayType++)
        {
            auto hitGroup = raytracingPipeline->CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
            if (rayType == RayType::Radiance)
            {
                hitGroup->SetClosestHitShaderImport(c_closestHitShaderNames[GeometryType::Triangle]);
            }
            hitGroup->SetHitGroupExport(c_hitGroupNames_TriangleGeometry[rayType]);
            hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_TRIANGLES);
        }
    }

    // AABB geometry hit groups
    {
        // Create hit groups for each intersection shader.
        for (UINT t = 0; t < IntersectionShaderType::Count; t++)
            for (UINT rayType = 0; rayType < RayType::Count; rayType++)
            {
                auto hitGroup = raytracingPipeline->CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
                hitGroup->SetIntersectionShaderImport(c_intersectionShaderNames[t]);
                if (rayType == RayType::Radiance)
                {
                    hitGroup->SetClosestHitShaderImport(c_closestHitShaderNames[GeometryType::AABB]);
                }
                hitGroup->SetHitGroupExport(c_hitGroupNames_AABBGeometry[t][rayType]);
                hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_PROCEDURAL_PRIMITIVE);
            }
    }
}

void SelectiveSDF::CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    // Ray gen and miss shaders in this sample are not using a local root signature and thus one is not associated with them.

    // Hit groups
    // Triangle geometry
    //{
    //    auto localRootSignature = raytracingPipeline->CreateSubobject<CD3DX12_LOCAL_ROOT_SIGNATURE_SUBOBJECT>();
    //    localRootSignature->SetRootSignature(m_raytracingLocalRootSignature[LocalRootSignature::Type::Triangle].Get());
    //    // Shader association
    //    auto rootSignatureAssociation = raytracingPipeline->CreateSubobject<CD3DX12_SUBOBJECT_TO_EXPORTS_ASSOCIATION_SUBOBJECT>();
    //    rootSignatureAssociation->SetSubobjectToAssociate(*localRootSignature);
    //    rootSignatureAssociation->AddExports(c_hitGroupNames_TriangleGeometry);
    //}

    // AABB geometry
    {
        auto localRootSignature = raytracingPipeline->CreateSubobject<CD3DX12_LOCAL_ROOT_SIGNATURE_SUBOBJECT>();
        localRootSignature->SetRootSignature(m_raytracingLocalRootSignature[LocalRootSignature::Type::AABB].Get());
        // Shader association
        auto rootSignatureAssociation = raytracingPipeline->CreateSubobject<CD3DX12_SUBOBJECT_TO_EXPORTS_ASSOCIATION_SUBOBJECT>();
        rootSignatureAssociation->SetSubobjectToAssociate(*localRootSignature);
        for (auto& hitGroupsForIntersectionShaderType : c_hitGroupNames_AABBGeometry)
        {
            rootSignatureAssociation->AddExports(hitGroupsForIntersectionShaderType);
        }
    }
}

void SelectiveSDF::BuildGeometry()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();
    auto commandAllocator = m_deviceResources->GetCommandAllocator();

    // Reset the command list for the acceleration structure construction.
    commandList->Reset(commandAllocator, nullptr);

    // triangle geometry
    for(auto& object : m_objects)
    {
        object->BuildGeometry(device, commandList);

        auto& indexBuffer = object->GetIndexBuffer();
        auto& vertexBuffer = object->GetVertexBuffer();

        
        // Vertex buffer is passed to the shader along with index buffer as a descriptor range.
        UINT descriptorIndexIB = m_resourceManager->CreateBufferSRV(device, &indexBuffer, object->GetIndexCount() * sizeof(Index) / 4, 0);
        UINT descriptorIndexVB = m_resourceManager->CreateBufferSRV(device, &vertexBuffer, object->GetVertexCount() * sizeof(Vertex) / 4, 0);
        ThrowIfFalse(descriptorIndexVB == descriptorIndexIB + 1, L"Vertex Buffer descriptor index must follow that of Index Buffer descriptor index");
	}


  //  const int LEAF_SIZE = 8;
  //  for (int i = 0; i < SDFModelObjectCount; i++)
  //  {
  //      int objectIdx = m_objects.size() - SDFModelObjectCount + i;
  //      auto& object = m_objects[objectIdx];

  //      auto hybridModelObject = dynamic_cast<AModel*>(object.get());
  //      if (hybridModelObject == nullptr)
  //      {
  //          throw runtime_error("SDF Model Object insertion order is messed up!");
  //      }

  //      hybridModelObject->BuildLeafAtlas(device, commandList);

  //      auto& sdfBuffer = hybridModelObject->GetLeafAtlasBuffer();
  //      m_resourceManager->CreateTexture2DArraySRV(device, &sdfBuffer, hybridModelObject->GetLeafCount() * LEAF_SIZE);

  //      if (i == 0)
  //      {
  //          m_sdfLeafAtlasGpuDescriptor = sdfBuffer.gpuDescriptorHandle;
  //      }
  //  }
  //  for (int i = 0; i < SDFModelObjectCount; i++)
  //  {
  //      int objectIdx = m_objects.size() - SDFModelObjectCount + i;
  //      auto& object = m_objects[objectIdx];

  //      auto hybridModelObject = dynamic_cast<AModel*>(object.get());
  //      if (hybridModelObject == nullptr)
  //      {
  //          throw runtime_error("SDF Model Object insertion order is messed up!");
  //      }

  //      hybridModelObject->BuildInternalLevel1(device, commandList);

  //      auto& sdfBuffer = hybridModelObject->GetInternal1NodeBuffer();
  //      m_resourceManager->CreateBufferSRV(device, &sdfBuffer, hybridModelObject->GetInternal1Count(), sizeof(InternalNode));

  //      if (i == 0)
  //      {
  //          m_sdfInternal1NodeGpuDescriptor = sdfBuffer.gpuDescriptorHandle;
  //      }
  //  }
  //  for (int i = 0; i < SDFModelObjectCount; i++)
  //  {
  //      int objectIdx = m_objects.size() - SDFModelObjectCount + i;
  //      auto& object = m_objects[objectIdx];

  //      auto hybridModelObject = dynamic_cast<AModel*>(object.get());
  //      if (hybridModelObject == nullptr)
  //      {
  //          throw runtime_error("SDF Model Object insertion order is messed up!");
  //      }

  //      hybridModelObject->BuildInternalLevel2(device, commandList);

  //      auto& sdfBuffer = hybridModelObject->GetInternal1NodeBuffer();
  //      m_resourceManager->CreateBufferSRV(device, &sdfBuffer, hybridModelObject->GetInternal2Count(), sizeof(InternalNode));

  //      if (i == 0)
  //      {
  //          m_sdfInternal2NodeGpuDescriptor = sdfBuffer.gpuDescriptorHandle;
  //      }
  //  }
  //  for (int i = 0; i < SDFModelObjectCount; i++)
  //  {
  //      int objectIdx = m_objects.size() - SDFModelObjectCount + i;
  //      auto& object = m_objects[objectIdx];

  //      auto hybridModelObject = dynamic_cast<AModel*>(object.get());
  //      if (hybridModelObject == nullptr)
  //      {
  //          throw runtime_error("SDF Model Object insertion order is messed up!");
  //      }

  //      hybridModelObject->BuildRoot(device, commandList);

  //      auto& sdfBuffer = hybridModelObject->GetRootNodeBuffer();
		//m_resourceManager->CreateBufferSRV(device, &sdfBuffer, 1, sizeof(InternalNode)); // Only one root node

  //      if (i == 0)
  //      {
  //          m_sdfRootNodeGpuDescriptor = sdfBuffer.gpuDescriptorHandle;
  //      }
  //  }


    // import sdf textures here
    //bool gpuHandleSet = false;
    //for (auto& object : m_objects)
    //{
    //    auto hybridModelObject = dynamic_cast<AModel*>(object.get());
    //    if (hybridModelObject == nullptr) continue;

    //    //hybridModelObject->BuildSDF(device, commandList);
    //    hybridModelObject->BuildSVS(device, commandList);


    //    auto& sdfBuffer = hybridModelObject->GetSDFBuffer();
    //    //m_resourceManager->CreateTexture3DSRV(device, &sdfBuffer);
    //    m_resourceManager->CreateBufferSRV(device, &sdfBuffer, hybridModelObject->GetCandidateVoxelCount(), sizeof(SurfaceVoxel));

    //    if (!gpuHandleSet)
    //    {
    //        m_sdfVoxelsGpuDescriptor = sdfBuffer.gpuDescriptorHandle;
    //        gpuHandleSet = true;
    //    }
    //}
    for (int i = 0; i < SDFModelObjectCount; i++)
    {
        int objectIdx = m_objects.size() - SDFModelObjectCount + i;
        auto& object = m_objects[objectIdx];

        auto hybridModelObject = dynamic_cast<AModel*>(object.get());
        if (hybridModelObject == nullptr)
        {
            throw runtime_error("SDF Model Object insertion order is messed up!");
        }

        hybridModelObject->BuildSVS(device, commandList);

        auto& sdfBuffer = hybridModelObject->GetSDFVoxelBuffer();
        m_resourceManager->CreateBufferSRV(device, &sdfBuffer, hybridModelObject->GetCandidateVoxelCount(), sizeof(SurfaceVoxel));

        if (i == 0)
        {
            m_sdfVoxelsGpuDescriptor = sdfBuffer.srvGpuDescriptorHandle;
        }
    }

    for (int i = 0; i < SDFModelObjectCount; i++)
    {
        int objectIdx = m_objects.size() - SDFModelObjectCount + i;
        auto& object = m_objects[objectIdx];

        auto hybridModelObject = dynamic_cast<AModel*>(object.get());
        if (hybridModelObject == nullptr)
        {
            throw runtime_error("SDF Model Object insertion order is messed up!");
        }

        hybridModelObject->BuildSDF(device, commandList);

        auto& sdfBuffer = hybridModelObject->GetSDFTextureBuffer();
        m_resourceManager->CreateTexture3DSRV(device, &sdfBuffer);

        if (i == 0)
        {
            m_sdfTexturesGpuDescriptor = sdfBuffer.srvGpuDescriptorHandle;
        }
    }


	UINT brickCount = 0;
    for (int i = 0; i < SDFInstanceCount; i++)
    {
        int instanceIdx = m_instances.size() - SDFInstanceCount + i;
        auto& instance = m_instances[instanceIdx];
        auto& object = m_objects[instance.objectIndex];

        auto hybridObject = dynamic_cast<HybridObject*>(object.get());
        if (hybridObject == nullptr)
        {
            throw runtime_error("SDF Model Object insertion order is messed up!");
        }

		brickCount += hybridObject->GetBrickCount();
    }

    m_brickTextureBuffers.resize(SDFInstanceCount);
    m_brickMetaSB.Create(device, brickCount, 1, L"SDF Instance Bricks Structured Buffer");
    UINT brickIndex = 0;
    for (int i = 0; i < SDFInstanceCount; i++)
    {
        int instanceIdx = m_instances.size() - SDFInstanceCount + i;
        auto& instance = m_instances[instanceIdx];
        auto& object = m_objects[instance.objectIndex];

        auto hybridObject = dynamic_cast<HybridObject*>(object.get());
        if (hybridObject == nullptr)
        {
            throw runtime_error("SDF Model Object insertion order is messed up!");
        }

        m_sdfInstancesSB[i].brickStart = brickIndex;

		auto& brickMeta = hybridObject->GetBricks();
        for (int i = 0; i < brickMeta.size(); i++)
        {
            m_brickMetaSB[brickIndex].packedCoord = brickMeta[i].packedCoord;
			m_brickMetaSB[brickIndex].sliceStart = brickMeta[i].sliceStart;
			m_brickMetaSB[brickIndex].instanceIndex = instanceIdx;

            brickIndex++;
        }


        UINT sliceCount = brickMeta.size() * 10;
        AllocateTexture(
            device,
            { 10, 10, sliceCount },
            &(m_brickTextureBuffers[i].resource),
            DXGI_FORMAT_R32G32B32A32_FLOAT,
            D3D12_RESOURCE_DIMENSION_TEXTURE2D,
            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
            D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS
        );

        m_resourceManager->CreateTexture2DArraySRV(device, &m_brickTextureBuffers[i], sliceCount, DXGI_FORMAT_R32G32B32A32_FLOAT);

        if (i == 0)
        {
            m_sdfBricksSRVGpuDescriptor = m_brickTextureBuffers[i].srvGpuDescriptorHandle;
        }
    }
    m_brickMetaSB.CopyStagingToGpu(0);


  //  m_candidateVoxelCount = 0;
  //  for (auto& instance : m_instances)
  //  {
  //      auto& object = m_objects[instance.objectIndex];
  //      if (object->GetType() != ObjectType::Hybrid) continue;
  //      auto hybridObject = static_cast<HybridObject*>(object.get());
  //      
		//m_candidateVoxelCount += hybridObject->GetCandidateVoxelCount();
  //  }

    // Execute to complete the copies
    m_deviceResources->ExecuteCommandList();
    m_deviceResources->WaitForGpu();

    for (auto& object : m_objects)
    {
        auto hybridModelObject = dynamic_cast<AModel*>(object.get());
        if (hybridModelObject != nullptr)
        {
            hybridModelObject->ReleaseStagingBuffers();
        }
        else object->ReleaseStagingBuffers();
    }
}

void SelectiveSDF::BuildGeometryDescsForBottomLevelAS(vector<vector<D3D12_RAYTRACING_GEOMETRY_DESC>>& geometryDescs)
{
    // Mark the geometry as opaque.
    D3D12_RAYTRACING_GEOMETRY_FLAGS geometryFlags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;

    // First, add all triangle-only objects (shared BLAS)
    for (UINT i = 0; i < m_objects.size(); i++)
    {
        if (m_objects[i]->GetType() == ObjectType::Triangle)
        {
            vector<D3D12_RAYTRACING_GEOMETRY_DESC> geometryDescsForObject(1);
            auto& geometryDesc = geometryDescsForObject[0];

            auto indexBuffer = m_objects[i]->GetIndexBuffer();
            auto vertexBuffer = m_objects[i]->GetVertexBuffer();

            geometryDesc = {};
            geometryDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
            geometryDesc.Triangles.IndexBuffer = indexBuffer.resource->GetGPUVirtualAddress();
            geometryDesc.Triangles.IndexCount = static_cast<UINT>(indexBuffer.resource->GetDesc().Width) / sizeof(Index);
            geometryDesc.Triangles.IndexFormat = DXGI_FORMAT_R32_UINT;
            geometryDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
            geometryDesc.Triangles.VertexCount = static_cast<UINT>(vertexBuffer.resource->GetDesc().Width) / sizeof(Vertex);
            geometryDesc.Triangles.VertexBuffer.StartAddress = vertexBuffer.resource->GetGPUVirtualAddress();
            geometryDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(Vertex);
            geometryDesc.Flags = geometryFlags;

            geometryDescs.push_back(move(geometryDescsForObject));
        }
    }



    // Then, add hybrid objects - one triangle BLAS per object type, one AABB BLAS per instance
    set<UINT> processedHybridObjects;

    UINT processedVoxelCount = 0;
    for (UINT instIdx = 0; instIdx < m_instances.size(); instIdx++)
    {
        auto& instance = m_instances[instIdx];
        auto& object = m_objects[instance.objectIndex];

        if (object->GetType() != ObjectType::Hybrid) continue;


        auto hybridObject = static_cast<HybridObject*>(object.get());

        // Add triangle BLAS once per unique hybrid object
        if (processedHybridObjects.find(instance.objectIndex) == processedHybridObjects.end())
        {
            vector<D3D12_RAYTRACING_GEOMETRY_DESC> triangleGeomDesc(1);
            auto& triDesc = triangleGeomDesc[0];

            auto indexBuffer = object->GetIndexBuffer();
            auto vertexBuffer = object->GetVertexBuffer();

            triDesc = {};
            triDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
            triDesc.Triangles.IndexBuffer = indexBuffer.resource->GetGPUVirtualAddress();
            triDesc.Triangles.IndexCount = static_cast<UINT>(indexBuffer.resource->GetDesc().Width) / sizeof(Index);
            triDesc.Triangles.IndexFormat = DXGI_FORMAT_R32_UINT;
            triDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
            triDesc.Triangles.VertexCount = static_cast<UINT>(vertexBuffer.resource->GetDesc().Width) / sizeof(Vertex);
            triDesc.Triangles.VertexBuffer.StartAddress = vertexBuffer.resource->GetGPUVirtualAddress();
            triDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(Vertex);
            triDesc.Flags = geometryFlags;

            geometryDescs.push_back(move(triangleGeomDesc));
            processedHybridObjects.insert(instance.objectIndex);
        }

        // Add AABB BLAS for each instance
        vector<D3D12_RAYTRACING_GEOMETRY_DESC> aabbGeomDesc(1);
        auto& aabbDesc = aabbGeomDesc[0];

        aabbDesc = {};
        aabbDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
        //aabbDesc.AABBs.AABBCount = hybridObject->GetCandidateVoxelCount();
        aabbDesc.AABBs.AABBCount = hybridObject->GetAAbbCount();
        //aabbDesc.AABBs.AABBs.StrideInBytes = sizeof(VoxelAABB);
        aabbDesc.AABBs.AABBs.StrideInBytes = sizeof(D3D12_RAYTRACING_AABB);
        aabbDesc.Flags = geometryFlags;
        UINT64 offsetInBytes = processedVoxelCount * sizeof(VoxelAABB);
        //aabbDesc.AABBs.AABBs.StartAddress = m_computeOutput.Get()->GetGPUVirtualAddress() + offsetInBytes;
        aabbDesc.AABBs.AABBs.StartAddress = hybridObject->GetAABBBuffer().resource->GetGPUVirtualAddress();

		processedVoxelCount += hybridObject->GetCandidateVoxelCount();

        geometryDescs.push_back(move(aabbGeomDesc));
    }


    
 //   for(UINT i = 0; i < m_objects.size(); i++)
 //   {
 //       switch(m_objects[i]->GetType())
 //       {
 //           case ObjectType::Hybrid:
 //           {
	//			vector<D3D12_RAYTRACING_GEOMETRY_DESC> geometryDescsForHybridObject;

 //               auto hybridObject = static_cast<HybridObject*>(m_objects[i].get());
 //               auto aabbBuffer = hybridObject->GetAABBBuffer();
 //               UINT aabbCount = hybridObject->GetAAbbCount();

	//			bool isModel = hybridObject->GetSDFPrimitiveType() == SDFPrimitive::AModel;

 //               geometryDescsForHybridObject.resize(1);
 //               auto& templateDesc = geometryDescsForHybridObject[0];

 //               templateDesc = {};
 //               templateDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
 //               templateDesc.AABBs.AABBCount = aabbCount;
 //               templateDesc.AABBs.AABBs.StrideInBytes = sizeof(D3D12_RAYTRACING_AABB);
 //               templateDesc.Flags = geometryFlags;

 //               templateDesc.AABBs.AABBs.StartAddress = aabbBuffer.resource->GetGPUVirtualAddress();

 //               geometryDescs.push_back(move(geometryDescsForHybridObject));
 //               //break;
 //           }
 //           case ObjectType::Triangle:
 //           {
 //               vector<D3D12_RAYTRACING_GEOMETRY_DESC> geometryDescsForObject;

 //               auto indexBuffer = m_objects[i]->GetIndexBuffer();
 //               auto vertexBuffer = m_objects[i]->GetVertexBuffer();

 //               geometryDescsForObject.resize(1);

 //               auto& geometryDesc = geometryDescsForObject[0];

 //               // set vertex and index buffers, their counts and startaddesses for each triangle geometry
 //               geometryDesc = {};
 //               geometryDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
 //               geometryDesc.Triangles.IndexBuffer = indexBuffer.resource->GetGPUVirtualAddress();
 //               geometryDesc.Triangles.IndexCount = static_cast<UINT>(indexBuffer.resource->GetDesc().Width) / sizeof(Index);
 //               geometryDesc.Triangles.IndexFormat = DXGI_FORMAT_R32_UINT;
 //               geometryDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
 //               geometryDesc.Triangles.VertexCount = static_cast<UINT>(vertexBuffer.resource->GetDesc().Width) / sizeof(Vertex);
 //               geometryDesc.Triangles.VertexBuffer.StartAddress = vertexBuffer.resource->GetGPUVirtualAddress();
 //               geometryDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(Vertex);
 //               geometryDesc.Flags = geometryFlags;

	//			geometryDescs.push_back(move(geometryDescsForObject));
 //               break;
 //           }
 //           default:
 //               throw std::runtime_error("Unknown object type for geometry descs");
	//	}
	//}
}

template <class InstanceDescType, class BLASPtrType>
void SelectiveSDF::BuildBottomLevelASInstanceDescs(vector<BLASPtrType>& bottomLevelASbuffers, UINT& instanceDescsCount, bool updateOnly)
{
    auto device = m_deviceResources->GetD3DDevice();

    vector<InstanceDescType> instanceDescs;

    for (UINT instIdx = 0; instIdx < m_instances.size(); instIdx++)
    {
        auto& instance = m_instances[instIdx];
        bool isHybrid = m_objects[instance.objectIndex]->GetType() == ObjectType::Hybrid;

        if (isHybrid)
        {
            UINT aabbBLASIdx = m_instanceToAABBBLAS.at(instIdx);
            UINT triangleBLASIdx = m_objectToTriangleBLAS.at(instance.objectIndex);

            // AABB instance desc
            InstanceDescType aabbInstanceDesc = {};
            aabbInstanceDesc.InstanceID = instance.objectIndex;
            aabbInstanceDesc.InstanceMask = 0x02;
            aabbInstanceDesc.InstanceContributionToHitGroupIndex = 1;
            aabbInstanceDesc.AccelerationStructure = bottomLevelASbuffers[aabbBLASIdx].accelerationStructure->GetGPUVirtualAddress();
            XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(aabbInstanceDesc.Transform), instance.CalculateTransform());
            instanceDescs.push_back(aabbInstanceDesc);

            // Triangle instance desc
            InstanceDescType triInstanceDesc = {};
            triInstanceDesc.InstanceID = instance.objectIndex;
            triInstanceDesc.InstanceMask = 0x04;
            triInstanceDesc.InstanceContributionToHitGroupIndex = 0;
            triInstanceDesc.AccelerationStructure = bottomLevelASbuffers[triangleBLASIdx].accelerationStructure->GetGPUVirtualAddress();
            XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(triInstanceDesc.Transform), instance.CalculateTransform());
            instanceDescs.push_back(triInstanceDesc);
        }
        else
        {
            UINT blasIdx = m_objectToTriangleBLAS.at(instance.objectIndex);

            InstanceDescType instanceDesc = {};
            instanceDesc.InstanceID = instance.objectIndex;
            instanceDesc.InstanceMask = 0x01;
            instanceDesc.InstanceContributionToHitGroupIndex = 0;
            instanceDesc.AccelerationStructure = bottomLevelASbuffers[blasIdx].accelerationStructure->GetGPUVirtualAddress();
            XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), instance.CalculateTransform());
            instanceDescs.push_back(instanceDesc);
        }
    }


    //UINT blasIndex = 0;

    //for (UINT i = 0; i < m_instances.size(); i++)
    //{
    //    auto& instance = m_instances[i];
    //    bool isHybrid = m_objects[instance.objectIndex]->GetType() == ObjectType::Hybrid;

    //    InstanceDescType instanceDesc = {};
    //    instanceDesc.InstanceID = instance.objectIndex;
    //    instanceDesc.InstanceMask = 0x01;
    //    instanceDesc.InstanceContributionToHitGroupIndex = 0;
    //    instanceDesc.AccelerationStructure = bottomLevelASaddresses[blasIndex];
    //    
    //    if (isHybrid)
    //    {
    //        instanceDesc.AccelerationStructure = bottomLevelASaddresses[blasIndex + 1];
    //        instanceDesc.InstanceMask = 0x04;

    //        auto hybridObject = static_cast<HybridObject*>(m_objects[instance.objectIndex].get());

    //        InstanceDescType aabbInstanceDesc = {};
    //        aabbInstanceDesc.InstanceID = instance.objectIndex; // same instance id as its associated polygonal geometry
    //        aabbInstanceDesc.InstanceMask = 0x02;
    //        //aabbInstanceDesc.InstanceContributionToHitGroupIndex = hybridObject->GetHitGroupIndex();
    //        aabbInstanceDesc.InstanceContributionToHitGroupIndex = 1; // probably no need for separate intersection shaders per hybrid object type??
    //        aabbInstanceDesc.AccelerationStructure = bottomLevelASaddresses[blasIndex];



    //        XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(aabbInstanceDesc.Transform), instance.CalculateTransform());
    //        instanceDescs.push_back(aabbInstanceDesc);
    //    }

    //    XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), instance.CalculateTransform());
    //    instanceDescs.push_back(instanceDesc);

    //    if (i < m_instances.size() - 1 && m_instances[i + 1].objectIndex > instance.objectIndex)
    //    {
    //        blasIndex += isHybrid ? 2 : 1;
    //    }
    //}

    if(instanceDescs.empty())
    {
        throw std::runtime_error("No instance descriptions were created from bottom level acceleration structure.");
	}

    instanceDescsCount = instanceDescs.size();

    UINT64 bufferSize = static_cast<UINT64>(instanceDescs.size() * sizeof(instanceDescs[0]));

    if (!updateOnly)
    {
        AllocateBuffer(
            device,
            D3D12_HEAP_TYPE_UPLOAD,
            bufferSize,
            &m_topLevelAS->instanceDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            D3D12_RESOURCE_FLAG_NONE,
            instanceDescs.data(),
            L"InstanceDescs"
        );
    }
    else
    {
        void* pMappedData;
        m_topLevelAS->instanceDesc->Map(0, nullptr, &pMappedData);
        memcpy(pMappedData, instanceDescs.data(), bufferSize);
        m_topLevelAS->instanceDesc->Unmap(0, nullptr);
    }
};

AccelerationStructureBuffers SelectiveSDF::BuildBottomLevelAS(const vector<D3D12_RAYTRACING_GEOMETRY_DESC>& geometryDescs, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags)
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();
    ComPtr<ID3D12Resource> scratch;
    ComPtr<ID3D12Resource> bottomLevelAS;

    // Get the size requirements for the scratch and AS buffers.
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC bottomLevelBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& bottomLevelInputs = bottomLevelBuildDesc.Inputs;
    bottomLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
    bottomLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    bottomLevelInputs.Flags = buildFlags;
    bottomLevelInputs.NumDescs = static_cast<UINT>(geometryDescs.size());
    bottomLevelInputs.pGeometryDescs = geometryDescs.data();

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO bottomLevelPrebuildInfo = {};
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&bottomLevelInputs, &bottomLevelPrebuildInfo);
    ThrowIfFalse(bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    // Create a scratch buffer.
    AllocateBuffer(
        device, 
        D3D12_HEAP_TYPE_DEFAULT,
        bottomLevelPrebuildInfo.ScratchDataSizeInBytes, 
        &scratch, 
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, 
        D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, 
        nullptr, 
        L"ScratchResource"
    );

    // Allocate resources for acceleration structures.
    {
		D3D12_HEAP_TYPE heapType = D3D12_HEAP_TYPE_DEFAULT;
        D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;
		D3D12_RESOURCE_FLAGS resourceFlags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
        AllocateBuffer(
            device, 
            heapType, 
            bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes, 
            &bottomLevelAS, 
            initialResourceState,
            resourceFlags,
            nullptr,
            L"BottomLevelAccelerationStructure");
    }

    // bottom-level AS desc.
    {
        bottomLevelBuildDesc.ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress();
        bottomLevelBuildDesc.DestAccelerationStructureData = bottomLevelAS->GetGPUVirtualAddress();
    }

    // Build the acceleration structure.
    m_dxrCommandList->BuildRaytracingAccelerationStructure(&bottomLevelBuildDesc, 0, nullptr);

    AccelerationStructureBuffers bottomLevelASBuffers;
    bottomLevelASBuffers.accelerationStructure = bottomLevelAS;
    bottomLevelASBuffers.scratch = scratch;
    bottomLevelASBuffers.ResultDataMaxSizeInBytes = bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes;
    return bottomLevelASBuffers;
}

void SelectiveSDF::BuildTopLevelAS(AccelerationStructureBuffers* bottomLevelAS, UINT bottomLevelASCount, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags)
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();

    m_topLevelAS = make_unique<AccelerationStructureBuffers>();


    // Create instance descs for the bottom-level acceleration structures
    UINT topLevelASCount;
    BuildBottomLevelASInstanceDescs<D3D12_RAYTRACING_INSTANCE_DESC>(m_bottomLevelAS, topLevelASCount);

    // Get required sizes for an acceleration structure.
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC topLevelBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& topLevelInputs = topLevelBuildDesc.Inputs;
    topLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    topLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    topLevelInputs.Flags = buildFlags;
    topLevelInputs.NumDescs = topLevelASCount;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO topLevelPrebuildInfo = {};
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&topLevelInputs, &topLevelPrebuildInfo);
    ThrowIfFalse(topLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    AllocateBuffer(
        device,
        D3D12_HEAP_TYPE_DEFAULT,
        topLevelPrebuildInfo.ScratchDataSizeInBytes,
        &m_topLevelAS->scratch,
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
        D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
        nullptr,
        L"ScratchResource"
    );

    // Allocate resources for acceleration structures.
    {
        D3D12_HEAP_TYPE heapType = D3D12_HEAP_TYPE_DEFAULT;
        D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;
        D3D12_RESOURCE_FLAGS resourceFlags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
        AllocateBuffer(
            device,
            heapType,
            topLevelPrebuildInfo.ResultDataMaxSizeInBytes,
            &m_topLevelAS->accelerationStructure,
            initialResourceState,
            resourceFlags,
            nullptr,
            L"TopLevelAccelerationStructure"
        );
    }

    // Top-level AS desc
    {
        topLevelBuildDesc.DestAccelerationStructureData = m_topLevelAS->accelerationStructure->GetGPUVirtualAddress();
        topLevelInputs.InstanceDescs = m_topLevelAS->instanceDesc->GetGPUVirtualAddress();
        topLevelBuildDesc.ScratchAccelerationStructureData = m_topLevelAS->scratch->GetGPUVirtualAddress();
    }

    // Build acceleration structure.
    m_dxrCommandList->BuildRaytracingAccelerationStructure(&topLevelBuildDesc, 0, nullptr);

    m_topLevelAS->ResultDataMaxSizeInBytes = topLevelPrebuildInfo.ResultDataMaxSizeInBytes;
}

void SelectiveSDF::UpdateDynamicBLAS()
{
    auto commandList = m_deviceResources->GetCommandList();

    // Collect all BLAS that need rebuilding
    vector<UINT> blasIndicesToRebuild;

	UINT processedVoxelCount = 0;
    for (const auto& [instIdx, blasIdx] : m_instanceToAABBBLAS)
    {
        auto& instance = m_instances[instIdx];
        auto hybridObject = dynamic_cast<HybridObject*>(m_objects[instance.objectIndex].get());

        // Step 1: Update the AABB buffer with new surface voxels
        //hybridObject->UpdateAABBBuffer(commandList);

        // Step 2: Create NEW geometry descriptor with updated info
        D3D12_RAYTRACING_GEOMETRY_DESC geometryDesc = {};
        geometryDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
        geometryDesc.AABBs.AABBCount = hybridObject->GetCandidateVoxelCount();
        geometryDesc.AABBs.AABBs.StrideInBytes = sizeof(VoxelAABB);
        geometryDesc.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
        UINT64 offsetInBytes = processedVoxelCount * sizeof(VoxelAABB);
        geometryDesc.AABBs.AABBs.StartAddress = m_computeOutput.Get()->GetGPUVirtualAddress() + offsetInBytes;
        //geometryDesc.AABBs.AABBs.StartAddress = m_computeOutput.Get()->GetGPUVirtualAddress();

		processedVoxelCount += hybridObject->GetCandidateVoxelCount();

        // Step 3: Rebuild BLAS with new geometry
        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC buildDesc = {};
        buildDesc.Inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
        buildDesc.Inputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_BUILD;
        buildDesc.Inputs.NumDescs = 1;
        buildDesc.Inputs.pGeometryDescs = &geometryDesc;  // Points to our new descriptor
        buildDesc.Inputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;

        // Reuse existing BLAS and scratch buffers
        auto& blas = m_bottomLevelAS[blasIdx];
        buildDesc.DestAccelerationStructureData = blas.accelerationStructure->GetGPUVirtualAddress();
        buildDesc.ScratchAccelerationStructureData = blas.scratch->GetGPUVirtualAddress();

        // Build it!
        m_dxrCommandList->BuildRaytracingAccelerationStructure(&buildDesc, 0, nullptr);

        blasIndicesToRebuild.push_back(blasIdx);
    }

    // Batch barrier for all rebuilt BLAS
    if (!blasIndicesToRebuild.empty())
    {
        UINT blasCount = static_cast<UINT>(blasIndicesToRebuild.size());
        // Batch all resource barriers for bottom-level AS builds.
        vector<D3D12_RESOURCE_BARRIER> resourceBarriers;
        resourceBarriers.resize(blasCount);
        for (UINT i = 0; i < blasCount; i++)
        {
            resourceBarriers[i] = CD3DX12_RESOURCE_BARRIER::UAV(m_bottomLevelAS[blasIndicesToRebuild[i]].accelerationStructure.Get());
        }
        commandList->ResourceBarrier(blasCount, resourceBarriers.data());
    }
}

void SelectiveSDF::UpdateTopLevelAS()
{
    UINT topLevelASCount;
    BuildBottomLevelASInstanceDescs<D3D12_RAYTRACING_INSTANCE_DESC>(m_bottomLevelAS, topLevelASCount, true);

    // Get required sizes for an acceleration structure.
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC topLevelBuildDesc = {};
    auto& topLevelInputs = topLevelBuildDesc.Inputs;
    topLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    topLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    topLevelInputs.NumDescs = topLevelASCount;
    topLevelBuildDesc.DestAccelerationStructureData = m_topLevelAS->accelerationStructure->GetGPUVirtualAddress();
    topLevelInputs.InstanceDescs = m_topLevelAS->instanceDesc->GetGPUVirtualAddress();
    topLevelBuildDesc.ScratchAccelerationStructureData = m_topLevelAS->scratch->GetGPUVirtualAddress();
    topLevelInputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE |
        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PERFORM_UPDATE;
    topLevelBuildDesc.SourceAccelerationStructureData = m_topLevelAS->accelerationStructure->GetGPUVirtualAddress();

    m_dxrCommandList->BuildRaytracingAccelerationStructure(&topLevelBuildDesc, 0, nullptr);
}

void SelectiveSDF::BuildAccelerationStructures()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();
    auto commandQueue = m_deviceResources->GetCommandQueue();
    auto commandAllocator = m_deviceResources->GetCommandAllocator();

    // Reset the command list for the acceleration structure construction.
    commandList->Reset(commandAllocator, nullptr);

    // Build bottom-level AS.
    //vector<AccelerationStructureBuffers> bottomLevelAS;
    vector<vector<D3D12_RAYTRACING_GEOMETRY_DESC>> geometryDescs;
    
    {
        BuildGeometryDescsForBottomLevelAS(geometryDescs);
        m_bottomLevelAS.resize(geometryDescs.size());
    }

    {
        // Build BLAS mapping info
        m_blasInfos.clear();
        UINT blasIdx = 0;

        // Triangle-only objects
        for (UINT i = 0; i < m_objects.size(); i++)
        {
            if (m_objects[i]->GetType() == ObjectType::Triangle)
            {
                m_blasInfos.push_back({ BLASInfo::TRIANGLE, i, UINT_MAX });
                blasIdx++;
            }
        }

        // Hybrid objects
        std::set<UINT> processedHybridObjects;
        for (UINT instIdx = 0; instIdx < m_instances.size(); instIdx++)
        {
            auto& instance = m_instances[instIdx];
            if (m_objects[instance.objectIndex]->GetType() == ObjectType::Hybrid)
            {
                // Triangle BLAS (one per unique object)
                if (processedHybridObjects.find(instance.objectIndex) == processedHybridObjects.end())
                {
                    m_blasInfos.push_back({ BLASInfo::HYBRID_TRIANGLE, instance.objectIndex, UINT_MAX });
                    processedHybridObjects.insert(instance.objectIndex);
                    blasIdx++;
                }

                // AABB BLAS (one per instance)
                m_blasInfos.push_back({ BLASInfo::HYBRID_AABB, instance.objectIndex, instIdx });
                blasIdx++;
            }
        }


        m_objectToTriangleBLAS.clear();
        m_instanceToAABBBLAS.clear();

        for (UINT i = 0; i < m_blasInfos.size(); i++)
        {
            const auto& info = m_blasInfos[i];

            if (info.type == BLASInfo::Type::TRIANGLE ||
                info.type == BLASInfo::Type::HYBRID_TRIANGLE)
            {
                m_objectToTriangleBLAS[info.objectIndex] = i;
            }
            else if (info.type == BLASInfo::Type::HYBRID_AABB)
            {
                m_instanceToAABBBLAS[info.instanceIndex] = i;
            }
        }
    }


    // Build all bottom-level AS.
    for (UINT i = 0; i < geometryDescs.size(); i++)
    {
        auto buildFlags = (m_blasInfos[i].type == BLASInfo::HYBRID_AABB) ?
            D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_BUILD :
            D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;

        m_bottomLevelAS[i] = BuildBottomLevelAS(geometryDescs[i], buildFlags);
    }

	UINT blasCount = static_cast<UINT>(m_bottomLevelAS.size());

    // Batch all resource barriers for bottom-level AS builds.
    vector<D3D12_RESOURCE_BARRIER> resourceBarriers;
    resourceBarriers.resize(blasCount);
    for (UINT i = 0; i < blasCount; i++)
    {
        resourceBarriers[i] = CD3DX12_RESOURCE_BARRIER::UAV(m_bottomLevelAS[i].accelerationStructure.Get());
    }
    commandList->ResourceBarrier(blasCount, resourceBarriers.data());


    // Build top-level AS.
    BuildTopLevelAS(
        m_bottomLevelAS.data(),
        blasCount,
        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_ALLOW_UPDATE
    );

    // Kick off acceleration structure construction.
    m_deviceResources->ExecuteCommandList();

    // Wait for GPU to finish as the locally created temporary GPU resources will get released once we go out of scope.
    m_deviceResources->WaitForGpu();

    // Store the AS buffers. The rest of the buffers will be released once we exit the function.
    /*m_bottomLevelAS.resize(BlasCount);
    for (UINT i = 0; i < BlasCount; i++)
    {
        m_bottomLevelAS[i] = bottomLevelAS[i].accelerationStructure;
    }*/
}

void SelectiveSDF::CreateConstantBuffers()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto frameCount = m_deviceResources->GetBackBufferCount();

    m_sceneCB.Create(device, frameCount, L"Scene Constant Buffer");

    m_sceneCB->triangeObjectCount = m_objects.size() - SDFObjectCount;
    m_sceneCB->sdfObjectCount = SDFObjectCount;
    m_sceneCB->triangleInstanceCount = m_instances.size() - SDFInstanceCount;
    m_sceneCB->sdfInstanceCount = SDFInstanceCount;

	m_computeCB.Create(device, 1, L"Compute Constant Buffer");
    //m_computeCB->candidateVoxelCount = m_candidateVoxelCount;
    m_computeCB->triangeObjectCount = m_objects.size() - SDFObjectCount;
    m_computeCB->sdfObjectCount = SDFObjectCount;
    m_computeCB->triangleInstanceCount = m_instances.size() - SDFInstanceCount;
    m_computeCB->sdfInstanceCount = SDFInstanceCount;
    m_computeCB.CopyStagingToGpu(0);
}

void SelectiveSDF::CreateStructuredBuffers()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto frameCount = m_deviceResources->GetBackBufferCount();

    m_sdfInstancesSB.Create(device, SDFInstanceCount, frameCount, L"SDF Instances Structured Buffer");

    m_bvhNodesSB.Create(device, m_bvhBuilder.m_nodes.size(), frameCount, L"BVH Structured Buffer");
    m_instanceIndicesSB.Create(device, m_bvhBuilder.m_flatInstanceIndices.size(), frameCount, L"BVH Node Instances Structured Buffer");


    m_sdfObjectsSB.Create(device, SDFObjectCount, 1, L"SDF Objects Structured Buffer");
    UINT objectIdx, textureIdx = 0;
    for (UINT i = 0; i < SDFObjectCount; i++)
    {
        objectIdx = m_objects.size() - SDFObjectCount + i;
        auto& object = m_objects[objectIdx];

        if (object->GetType() != ObjectType::Hybrid)
        {
            throw runtime_error("Instance insertion order is messed up!");
        }

        auto hybridObject = static_cast<HybridObject*>(object.get());

		auto primitiveType = hybridObject->GetSDFPrimitiveType();
        m_sdfObjectsSB[i].primitiveType = primitiveType;

        if (primitiveType == SDFPrimitive::AModel)
        {
            m_sdfObjectsSB[i].textureIndex = textureIdx++; // Only process AModel type for brick atlas
        }
    }

    // uncomment this if it's not being modified later
    m_sdfObjectsSB.CopyStagingToGpu(0);

    /*auto size = m_grid.GetHashTableSize();
    m_hashTableSB.Create(device, m_grid.GetHashTableSize(), frameCount, L"Hash Table Structured Buffer");
    m_instanceIndicesSB.Create(device, m_grid.GetInstanceIndicesSize(), frameCount, L"Hash Table Instance Indices Structured Buffer");*/

    //m_candidateVoxelsSB.Create(device, m_candidateVoxelCount, 1, L"Candidate Voxels Structured Buffer");
    //{
    //    int instanceIdx = 0;
    //    int voxelIndex = 0;
    //    for (auto& instance : m_instances)
    //    {
    //        auto& object = m_objects[instance.objectIndex];
    //        if (object->GetType() != ObjectType::Hybrid) continue;
    //        auto hybridObject = static_cast<HybridObject*>(object.get());

    //        auto& candidateVoxels = hybridObject->GetCandidateVoxels();
    //        for (int i = 0; i < candidateVoxels.size(); i++)
    //        {
    //            /*m_candidateVoxelsSB[voxelIndex] = candidateVoxels[i];
    //            m_candidateVoxelsSB[voxelIndex].instanceIndex = instanceIdx;*/

    //            uint32_t packed = (candidateVoxels[i].x & 0x3F) |
    //                ((candidateVoxels[i].y & 0x3F) << 6) |
    //                ((candidateVoxels[i].z & 0x3F) << 12) |
    //                (instanceIdx << 18);

    //            m_candidateVoxelsSB[voxelIndex] = packed;

    //            voxelIndex++;
    //        }
    //        instanceIdx++;
    //    }
    //}
    //m_candidateVoxelsSB.CopyStagingToGpu(0);
}

void SelectiveSDF::BuildShaderTables()
{
    auto device = m_deviceResources->GetD3DDevice();

    void* rayGenShaderID;
    void* missShaderIDs[RayType::Count];
    void* hitGroupShaderIDs_TriangleGeometry[RayType::Count];
    void* hitGroupShaderIDs_AABBGeometry[IntersectionShaderType::Count][RayType::Count];


    // A shader name look-up table for shader table debug print out.
    unordered_map<void*, wstring> shaderIdToStringMap;

    auto GetShaderIDs = [&](auto* stateObjectProperties)
    {
        rayGenShaderID = stateObjectProperties->GetShaderIdentifier(c_raygenShaderName);
        shaderIdToStringMap[rayGenShaderID] = c_raygenShaderName;

        for (UINT i = 0; i < RayType::Count; i++)
        {
            missShaderIDs[i] = stateObjectProperties->GetShaderIdentifier(c_missShaderNames[i]);
            shaderIdToStringMap[missShaderIDs[i]] = c_missShaderNames[i];
        }
        for (UINT i = 0; i < RayType::Count; i++)
        {
            hitGroupShaderIDs_TriangleGeometry[i] = stateObjectProperties->GetShaderIdentifier(c_hitGroupNames_TriangleGeometry[i]);
            shaderIdToStringMap[hitGroupShaderIDs_TriangleGeometry[i]] = c_hitGroupNames_TriangleGeometry[i];
        }
        for (UINT r = 0; r < IntersectionShaderType::Count; r++)
        {
            for (UINT c = 0; c < RayType::Count; c++)
            {
                hitGroupShaderIDs_AABBGeometry[r][c] = stateObjectProperties->GetShaderIdentifier(c_hitGroupNames_AABBGeometry[r][c]);
                shaderIdToStringMap[hitGroupShaderIDs_AABBGeometry[r][c]] = c_hitGroupNames_AABBGeometry[r][c];
            }
        }
    };

    // Get shader identifiers.
    UINT shaderIDSize;
    {
        ComPtr<ID3D12StateObjectProperties> stateObjectProperties;
        ThrowIfFailed(m_dxrStateObject.As(&stateObjectProperties));
        GetShaderIDs(stateObjectProperties.Get());
        shaderIDSize = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;
    }

    // RayGen shader table.
    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = shaderIDSize; // No root arguments

        ShaderTable rayGenShaderTable(device, numShaderRecords, shaderRecordSize, L"RayGenShaderTable");
        rayGenShaderTable.push_back(ShaderRecord(rayGenShaderID, shaderRecordSize, nullptr, 0));
        rayGenShaderTable.DebugPrint(shaderIdToStringMap);
        m_rayGenShaderTable = rayGenShaderTable.GetResource();
    }

    // Miss shader table.
    {
        UINT numShaderRecords = RayType::Count;
        UINT shaderRecordSize = shaderIDSize; // No root arguments

        ShaderTable missShaderTable(device, numShaderRecords, shaderRecordSize, L"MissShaderTable");
        for (UINT i = 0; i < RayType::Count; i++)
        {
            missShaderTable.push_back(ShaderRecord(missShaderIDs[i], shaderIDSize, nullptr, 0));
        }
        missShaderTable.DebugPrint(shaderIdToStringMap);
        m_missShaderTableStrideInBytes = missShaderTable.GetShaderRecordSize();
        m_missShaderTable = missShaderTable.GetResource();
    }

    // Hit group shader table.
    {
        UINT numShaderRecords = RayType::Count + IntersectionShaderType::Count * RayType::Count;
        UINT shaderRecordSize = shaderIDSize + LocalRootSignature::MaxRootArgumentsSize();
        ShaderTable hitGroupShaderTable(device, numShaderRecords, shaderRecordSize, L"HitGroupShaderTable");

        // Triangle geometry hit groups.
        {
            /*LocalRootSignature::Triangle::RootArguments rootArgs;
            rootArgs.materialCb = m_planeMaterialCB;*/

            for (auto& hitGroupShaderID : hitGroupShaderIDs_TriangleGeometry)
            {
                hitGroupShaderTable.push_back(ShaderRecord(hitGroupShaderID, shaderIDSize, nullptr, 0));
            }
        }

        // AABB geometry hit groups.
        {
            LocalRootSignature::AABB::RootArguments rootArgs;
            UINT instanceIndex = 0;

            // Create a shader record for each primitive.
            for (UINT iShader = 0, instanceIndex = 0; iShader < IntersectionShaderType::Count; iShader++)
            {
                //rootArgs.objectCb = m_aabbMaterialCB[instanceIndex];

                    // Ray types.
                for (UINT r = 0; r < RayType::Count; r++)
                {
                    auto& hitGroupShaderID = hitGroupShaderIDs_AABBGeometry[iShader][r];
                    hitGroupShaderTable.push_back(ShaderRecord(hitGroupShaderID, shaderIDSize, nullptr, 0));
                }
            }
        }
        hitGroupShaderTable.DebugPrint(shaderIdToStringMap);
        m_hitGroupShaderTableStrideInBytes = hitGroupShaderTable.GetShaderRecordSize();
        m_hitGroupShaderTable = hitGroupShaderTable.GetResource();
    }
}

void SelectiveSDF::CreateRaytracingOutputResource()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto backbufferFormat = m_deviceResources->GetBackBufferFormat();

    // Create the output resource. The dimensions and format should match the swap-chain.
    auto uavDesc = CD3DX12_RESOURCE_DESC::Tex2D(backbufferFormat, m_width, m_height, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
    ThrowIfFailed(device->CreateCommittedResource(
        &defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr, IID_PPV_ARGS(&m_raytracingOutput)));
    NAME_D3D12_OBJECT(m_raytracingOutput);

    D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
    m_raytracingOutputResourceUAVDescriptorHeapIndex = m_resourceManager->AllocateDescriptor(&uavDescriptorHandle, m_raytracingOutputResourceUAVDescriptorHeapIndex);
    D3D12_UNORDERED_ACCESS_VIEW_DESC UAVDesc = {};
    UAVDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    device->CreateUnorderedAccessView(m_raytracingOutput.Get(), nullptr, &UAVDesc, uavDescriptorHandle);
    m_raytracingOutputResourceUAVGpuDescriptor = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_resourceManager->GetSRVDescriptorHeap()->GetGPUDescriptorHandleForHeapStart(), m_raytracingOutputResourceUAVDescriptorHeapIndex, m_resourceManager->GetSRVDescriptorSize());
}

void SelectiveSDF::CreateComputeOutputResource()
{
    auto device = m_deviceResources->GetD3DDevice();

    
    for (int i = 0; i < SDFInstanceCount; i++)
    {
        int instanceIdx = m_instances.size() - SDFInstanceCount + i;
        auto& instance = m_instances[instanceIdx];
        auto& object = m_objects[instance.objectIndex];

        auto hybridObject = dynamic_cast<HybridObject*>(object.get());
        if (hybridObject == nullptr)
        {
            throw runtime_error("SDF Model Object insertion order is messed up!");
        }

        UINT sliceCount = hybridObject->GetBrickCount() * 10;

        m_resourceManager->CreateTexture2DArrayUAV(device, &m_brickTextureBuffers[i], sliceCount, DXGI_FORMAT_R32G32B32A32_FLOAT);

        if (i == 0)
        {
            m_sdfBricksUAVGpuDescriptor = m_brickTextureBuffers[i].uavGpuDescriptorHandle;
        }
    }
}

//void SelectiveSDF::BuildSpatialHashGrid()
//{
//    m_grid.ClearCells();
//
//    for (int i = 0; i < m_instances.size(); i++) {
//        const auto& instance = m_instances[i];
//
//        if (m_objects[instance.objectIndex]->GetType() != ObjectType::Hybrid) continue;
//
//        // Calculate world-space AABB
//        float halfSize = 0.6f * instance.scale;  // Your padded size
//        XMFLOAT3 worldMin = {
//            instance.position.x - halfSize,
//            instance.position.y - halfSize,
//            instance.position.z - halfSize
//        };
//        XMFLOAT3 worldMax = {
//            instance.position.x + halfSize,
//            instance.position.y + halfSize,
//            instance.position.z + halfSize
//        };
//
//        // Convert to cell coordinates
//        auto minCell = m_grid.WorldToCell(worldMin);
//        auto maxCell = m_grid.WorldToCell(worldMax);
//
//        // Insert into all overlapped cells
//        for (int x = minCell.x; x <= maxCell.x; x++) 
//        {
//            for (int y = minCell.y; y <= maxCell.y; y++) 
//            {
//                for (int z = minCell.z; z <= maxCell.z; z++) 
//                {
//                    m_grid.AddCellData(x, y, z, i);
//                }
//            }
//        }
//    }
//
//    // build hash table and indices flat array
//    m_grid.Build();
//}

void SelectiveSDF::BuildSDfBVH()
{
    m_bvhBuilder.Build(m_instances, m_instances.size() - SDFInstanceCount);
}

void SelectiveSDF::CalculateFrameStats()
{
    static int frameCnt = 0;
    static double prevTime = 0.0f;
    double totalTime = m_timer.GetTotalSeconds();

    frameCnt++;

    // Compute averages over one second period.
    if ((totalTime - prevTime) >= 1.0f)
    {
        float diff = static_cast<float>(totalTime - prevTime);
        float fps = static_cast<float>(frameCnt) / diff; // Normalize to an exact second.

        frameCnt = 0;
        prevTime = totalTime;

        wstringstream windowText;
        windowText << setprecision(2) << fixed  << L"    fps: " << fps;
        SetCustomWindowText(windowText.str().c_str());
    }
}

void SelectiveSDF::BuildSDFObjectsData()
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    UINT instanceIndex, textureIndex = 0;
    for (UINT i = 0; i < SDFInstanceCount; i++)
    {
        instanceIndex = m_instances.size() - SDFInstanceCount + i;
        auto& instance = m_instances[instanceIndex];
        auto& object = m_objects[instance.objectIndex];
        if (object->GetType() != ObjectType::Hybrid)
        {
            throw runtime_error("Instance insertion order is messed up!");
        }

        auto hybridObject = static_cast<HybridObject*>(object.get());

        auto& sdfInstance = m_sdfInstancesSB[i];
        sdfInstance.world = instance.CalculateTransform();
        sdfInstance.worldI = XMMatrixInverse(nullptr, instance.CalculateTransform());
        sdfInstance.scale = instance.scale;
		sdfInstance.objectIndex = instance.objectIndex;
        //sdfInstance.sdfPrimitiveType = hybridObject->GetSDFPrimitiveType();

        /*if (sdfObject.sdfPrimitiveType == SDFPrimitive::AModel)
        {
            sdfObject.sdfTextureIndex = textureIndex;

            if (i < SDFInstanceCount - 1 && m_instances[instanceIndex + 1].objectIndex > instance.objectIndex)
            {
                textureIndex++;
            }
        }*/

        sdfInstance.instanceIndex = instanceIndex;
    }
}

void SelectiveSDF::BuildBVHData()
{
    // copy to staging structured buffers
    for (UINT i = 0; i < m_bvhBuilder.m_nodes.size(); i++) {
        const auto& node = m_bvhBuilder.m_nodes[i];
        BVHNodeGPU& n = m_bvhNodesSB[i];

        n.min = node.boundingBox.min;
        n.max = node.boundingBox.max;
        n.leftChild = node.leftChild >= 0 ? node.leftChild : UINT_MAX;
        n.rightChild = node.rightChild >= 0 ? node.rightChild : UINT_MAX;
        n.firstInstance = node.firstInstance;
        n.instanceCount = node.instanceCount;
        n.padding[0] = n.padding[1] = 0;
    }
    for (UINT i = 0; i < m_bvhBuilder.m_flatInstanceIndices.size(); i++)
    {
        m_instanceIndicesSB[i] = { m_bvhBuilder.m_flatInstanceIndices[i] };
    }
}

void SelectiveSDF::UpdateCameraMatrices()
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

	m_sceneCB->viewI = XMMatrixInverse(nullptr, m_camera.GetViewMatrix());
	m_sceneCB->projectionI = XMMatrixInverse(nullptr, m_camera.GetProjectionMatrix());
}

void SelectiveSDF::UpdateSDFInstancesData()
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    for (UINT i = 0; i < SDFInstanceCount; i++)
    {
        auto& sdfInstance = m_sdfInstancesSB[i];
        auto& instanceData = m_instances[sdfInstance.instanceIndex];
        sdfInstance.world = instanceData.CalculateTransform();
        sdfInstance.worldI = XMMatrixInverse(nullptr, instanceData.CalculateTransform());
        sdfInstance.scale = instanceData.scale;
    }
}
void SelectiveSDF::BuildBrickAtlas()
{
  //  auto device = m_deviceResources->GetD3DDevice();
  //  auto commandList = m_deviceResources->GetCommandList();
  //  auto commandAllocator = m_deviceResources->GetCommandAllocator();

  //  commandList->Reset(commandAllocator, nullptr);

  //  const int BRICK_SIZE = 8;  // 8x8x8 values per brick
  //  const int SLICES_PER_BRICK = 8;

  //  UINT totalSlices = 0;
  //  vector<float> textureData;

  //  // First pass: Count total bricks and slices needed
  //  UINT objectIndex;
  //  for (UINT i = 0; i < SDFObjectCount; i++)
  //  {
  //      objectIndex = m_objects.size() - SDFObjectCount + i;
  //      auto& object = m_objects[objectIndex];

  //      if (object->GetType() != ObjectType::Hybrid)
  //      {
  //          throw runtime_error("Instance insertion order is messed up!");
  //      }

  //      auto hybridObject = static_cast<HybridObject*>(object.get());

  //      if(hybridObject->GetSDFPrimitiveType() == SDFPrimitive::AModel)
  //      {
  //          totalSlices += hybridObject->GetBrickCount() * SLICES_PER_BRICK;
  //      }
  //  }

  //  //m_brickTableSB.Create(device, totalSlices / SLICES_PER_BRICK, 1, L"SDF Bricks Structured Buffer");

  //  // Allocate texture data
  //  textureData.resize(BRICK_SIZE * BRICK_SIZE * totalSlices);

  //  // Second pass: Fill texture data and metadata
  //  UINT currentSlice = 0;
  //  UINT currentBrickIdx = 0;
  //  UINT textureIdx = 0;
  //  for (UINT i = 0; i < SDFObjectCount; i++)
  //  {
  //      objectIndex = m_objects.size() - SDFObjectCount + i;
  //      auto& object = m_objects[objectIndex];

  //      auto hybridObject = static_cast<HybridObject*>(object.get());

  //      auto primitiveType = hybridObject->GetSDFPrimitiveType();
  //      //m_sdfObjectsSB[i].primitiveType = primitiveType;

  //      if(primitiveType != SDFPrimitive::AModel)
  //      {
  //          continue; // Only process AModel type for brick atlas
		//}

		////m_sdfObjectsSB[i].textureIndex = textureIdx++;
  //      m_sdfObjectsSB[i].firstBrickIndex = currentBrickIdx;
  //      //m_sdfObjectsSB[i].brickCount = 0;

  //      auto objectBricks = hybridObject->GetBricks();

  //      for (const auto& brick : objectBricks) {
  //          BrickInfo info;
  //          info.brickCoord = { brick.bx, brick.by, brick.bz };

  //          // Copy brick data to texture
  //          for (int z = 0; z < BRICK_SIZE; z++) {
  //              UINT sliceIdx = currentSlice + z;
  //              UINT sliceStart = sliceIdx * BRICK_SIZE * BRICK_SIZE;

  //              for (int y = 0; y < BRICK_SIZE; y++) {
  //                  for (int x = 0; x < BRICK_SIZE; x++) {
  //                      UINT idx = sliceStart + y * BRICK_SIZE + x;
  //                      textureData[idx] = brick.values[x][y][z];
  //                  }
  //              }
  //          }

  //          currentBrickIdx++;
  //          currentSlice += SLICES_PER_BRICK;
  //      }
  //  }

  //  m_sdfObjectsSB.CopyStagingToGpu(0);

    //m_brickTableSB.CopyStagingToGpu(0);

    // atlas texture resource
    //{
    //    // Now create the texture atlas if we have any bricks
    //    if (totalSlices > 0) {
    //        // Create texture atlas using your existing pattern
    //        const UINT sliceWidth = 8;
    //        const UINT sliceHeight = 8;
    //        const UINT rowSize = sliceWidth * sizeof(float);
    //        const UINT alignedRowPitch = (rowSize + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1) & ~(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1);
    //        const UINT slicePitch = alignedRowPitch * sliceHeight;
    //        const UINT totalSize = slicePitch * totalSlices;

    //        // Prepare padded data
    //        std::vector<BYTE> paddedUploadData(totalSize, 0);

    //        // Copy texture data with padding
    //        for (UINT sliceIdx = 0; sliceIdx < totalSlices; sliceIdx++) {
    //            for (UINT y = 0; y < sliceHeight; y++) {
    //                const float* srcRow = &textureData[(sliceIdx * sliceHeight + y) * sliceWidth];
    //                BYTE* dstRow = &paddedUploadData[sliceIdx * slicePitch + y * alignedRowPitch];
    //                memcpy(dstRow, srcRow, rowSize);
    //            }
    //        }

    //        // Using your AllocateTexture pattern but for 2D array
    //        AllocateTexture(device, {sliceWidth, sliceHeight, totalSlices}, &m_brickAtlasBuffer.resource, D3D12_RESOURCE_DIMENSION_TEXTURE2D);
    //        AllocateBuffer(
    //            device,
    //            D3D12_HEAP_TYPE_UPLOAD,
    //            totalSize,
    //            &m_stagingBrickAtlasBuffer,
    //            D3D12_RESOURCE_STATE_GENERIC_READ,
    //            D3D12_RESOURCE_FLAG_NONE,
    //            paddedUploadData.data());

    //        // Prepare subresources
    //        vector<D3D12_SUBRESOURCE_DATA> subresources(totalSlices);
    //        for (UINT i = 0; i < totalSlices; i++) {
    //            subresources[i].pData = &paddedUploadData[i * slicePitch];
    //            subresources[i].RowPitch = alignedRowPitch;
    //            subresources[i].SlicePitch = slicePitch;
    //        }

    //        UpdateSubresources(commandList, m_brickAtlasBuffer.resource.Get(), m_stagingBrickAtlasBuffer.Get(),
    //            0, 0, totalSlices, subresources.data());

    //        auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
    //            m_brickAtlasBuffer.resource.Get(),
    //            D3D12_RESOURCE_STATE_COPY_DEST,
    //            D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
    //        commandList->ResourceBarrier(1, &barrier);


    //        m_resourceManager->CreateTexture2DArraySRV(device, &m_brickAtlasBuffer, totalSlices);
    //    }
    //}


    // Execute to complete the copies
    m_deviceResources->ExecuteCommandList();
    m_deviceResources->WaitForGpu();


    m_stagingBrickAtlasBuffer.Reset();
}

void SelectiveSDF::_BuildBrickAtlas()
{
 //   auto device = m_deviceResources->GetD3DDevice();
 //   auto commandList = m_deviceResources->GetCommandList();
 //   auto commandAllocator = m_deviceResources->GetCommandAllocator();

 //   commandList->Reset(commandAllocator, nullptr);

 //   const int LEAF_SIZE = 8;

 //   UINT totalSlices = 0;
 //   vector<float> textureData;

 //   UINT objectIndex;
 //   for (UINT i = 0; i < SDFObjectCount; i++)
 //   {
 //       objectIndex = m_objects.size() - SDFObjectCount + i;
 //       auto& object = m_objects[objectIndex];

 //       if (object->GetType() != ObjectType::Hybrid)
 //       {
 //           throw runtime_error("Instance insertion order is messed up!");
 //       }

 //       auto hybridModelObject = dynamic_cast<AModel*>(object.get());
 //       if (!hybridModelObject)
 //       {
 //           continue; // Only process AModel type for brick atlas
	//	}

 //       totalSlices += hybridModelObject->GetLeafCount() * LEAF_SIZE;
 //   }

 //   m_leafNodesSB.Create(device, totalSlices / LEAF_SIZE, 1, L"SDF Leaf Nodes Structured Buffer");
 //   textureData.resize(LEAF_SIZE * LEAF_SIZE * totalSlices);

 //   // Fill texture data
	//UINT currentLeafIndex = 0;
 //   for (UINT i = 0; i < SDFObjectCount; i++) {
 //       objectIndex = m_objects.size() - SDFObjectCount + i;
 //       auto hybridModelObject = dynamic_cast<AModel*>(m_objects[objectIndex].get());
 //       if (!hybridModelObject) continue;

 //       // Get this object's leaves
 //       auto& leaves = hybridModelObject->GetLeafNodes();

 //       // Copy voxel data to texture array
 //       for (const auto& leaf : leaves) {
	//		UINT currentSliceOffset = currentLeafIndex * LEAF_SIZE;
 //           // Store metadata with texture slice reference
	//		auto& meta = m_leafNodesSB[currentLeafIndex];
 //           meta.coord = leaf.coord;
 //           meta.sliceIndex = currentSliceOffset;
 //           memcpy(meta.bitmask, leaf.bitmask, sizeof(leaf.bitmask));

 //           // Copy 8x8x8 voxels as 8 slices of 8x8
 //           for (int z = 0; z < LEAF_SIZE; z++) {
 //               for (int y = 0; y < LEAF_SIZE; y++) {
 //                   for (int x = 0; x < LEAF_SIZE; x++) {
 //                       int voxelIdx = z * 64 + y * 8 + x;
 //                       int texIdx = (currentSliceOffset + z) * 64 + y * 8 + x;
 //                       textureData[texIdx] = leaf.voxels[voxelIdx];
 //                   }
 //               }
 //           }
 //           currentLeafIndex++;
 //       }
 //   }


 //   // atlas texture resource
 //   {
 //       // Now create the texture atlas if we have any bricks
 //       if (totalSlices > 0) {
 //           // Create texture atlas using your existing pattern
 //           const UINT sliceWidth = LEAF_SIZE;
 //           const UINT sliceHeight = LEAF_SIZE;
 //           const UINT rowSize = sliceWidth * sizeof(float);
 //           const UINT alignedRowPitch = (rowSize + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1) & ~(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1);
 //           const UINT slicePitch = alignedRowPitch * sliceHeight;
 //           const UINT totalSize = slicePitch * totalSlices;

 //           // Prepare padded data
 //           std::vector<BYTE> paddedUploadData(totalSize, 0);

 //           // Copy texture data with padding
 //           for (UINT sliceIdx = 0; sliceIdx < totalSlices; sliceIdx++) {
 //               for (UINT y = 0; y < sliceHeight; y++) {
 //                   const float* srcRow = &textureData[(sliceIdx * sliceHeight + y) * sliceWidth];
 //                   BYTE* dstRow = &paddedUploadData[sliceIdx * slicePitch + y * alignedRowPitch];
 //                   memcpy(dstRow, srcRow, rowSize);
 //               }
 //           }

 //           // Using your AllocateTexture pattern but for 2D array
 //           AllocateTexture(device, {sliceWidth, sliceHeight, totalSlices}, &m_brickAtlasBuffer.resource, D3D12_RESOURCE_DIMENSION_TEXTURE2D);
 //           AllocateBuffer(
 //               device,
 //               D3D12_HEAP_TYPE_UPLOAD,
 //               totalSize,
 //               &m_stagingBrickAtlasBuffer,
 //               D3D12_RESOURCE_STATE_GENERIC_READ,
 //               D3D12_RESOURCE_FLAG_NONE,
 //               paddedUploadData.data());

 //           // Prepare subresources
 //           vector<D3D12_SUBRESOURCE_DATA> subresources(totalSlices);
 //           for (UINT i = 0; i < totalSlices; i++) {
 //               subresources[i].pData = &paddedUploadData[i * slicePitch];
 //               subresources[i].RowPitch = alignedRowPitch;
 //               subresources[i].SlicePitch = slicePitch;
 //           }

 //           UpdateSubresources(commandList, m_brickAtlasBuffer.resource.Get(), m_stagingBrickAtlasBuffer.Get(),
 //               0, 0, totalSlices, subresources.data());

 //           auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
 //               m_brickAtlasBuffer.resource.Get(),
 //               D3D12_RESOURCE_STATE_COPY_DEST,
 //               D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
 //           commandList->ResourceBarrier(1, &barrier);


 //           m_resourceManager->CreateTexture2DArraySRV(device, &m_brickAtlasBuffer, totalSlices);
 //       }
 //   }

 //   m_deviceResources->ExecuteCommandList();
 //   m_deviceResources->WaitForGpu();


 //   m_stagingBrickAtlasBuffer.Reset();
}

//void SelectiveSDF::BuildHashGridData()
//{
//    // copy to staging structured buffers
//    for (UINT i = 0; i < m_grid.GetHashTableSize(); i++)
//    {
//        auto& entry = m_grid.GetHashTable()[i];
//        m_hashTableSB[i] = { entry.cellPos, entry.indexOffset, entry.count, entry.occupied };
//    }
//    for (UINT i = 0; i < m_grid.GetInstanceIndicesSize(); i++)
//    {
//        m_instanceIndicesSB[i] = {m_grid.GetInstanceIndices()[i]};
//    }
//
//    m_sceneCB->cellSize = m_grid.CellSize;
//    m_sceneCB->hashTableSize = m_grid.GetHashTableSize();
//}
