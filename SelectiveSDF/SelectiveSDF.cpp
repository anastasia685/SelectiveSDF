#include "stdafx.h"

#include "SelectiveSDF.h"

using namespace std;
using namespace DX;

// Shader entry points.
const wchar_t* SelectiveSDF::c_raygenShaderName = L"RayGen";
const wchar_t* SelectiveSDF::c_intersectionShaderNames[] =
{
    L"Intersection_SDF_Box",
    L"Intersection_SDF_Texture",
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
    { L"HitGroup_SDF_Box" },
    { L"HitGroup_SDF_Texture" },
};

SelectiveSDF::SelectiveSDF(UINT width, UINT height, std::wstring name) : DXSample(width, height, name)
{
    m_resourceManager = make_unique<ResourceManager>();

	unique_ptr<Plane> plane = make_unique<Plane>();
    plane->AddInstanceData({ 0, -0.5, 0 }, { 0, 0, 0 }, { 5, 1, 5 });

    unique_ptr<Box> box = make_unique<Box>();
    box->AddInstanceData({ -0.7, 0, 0 });

	unique_ptr<AModel> aModel = make_unique<AModel>();
    aModel->AddInstanceData({ 0.7, 0, 0 });


    m_objects.push_back(move(plane));
    m_objects.push_back(move(box));
    m_objects.push_back(move(aModel));

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

    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();

    InitializeScene();
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

    // Create a raytracing pipeline state object which defines the binding of shaders, state and resources to be used during raytracing.
    CreateRaytracingPipelineStateObject();

    // Create a heap for descriptors.
    m_resourceManager->Initialize(m_deviceResources->GetD3DDevice(), 16);

    // Build geometry to be used in the sample
    BuildGeometry();

    // Build raytracing acceleration structures from the generated geometry.
    BuildAccelerationStructures();

    // Create constant buffers for the scene.
    CreateConstantBuffers();

    // Build shader tables, which define shaders and their local root arguments.
    BuildShaderTables();

    // Create an output 2D texture to store the raytracing result to.
    //CreateRaytracingOutputResource();
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
    /*m_indexBuffer.resource.Reset();
    m_vertexBuffer.resource.Reset();
    m_aabbBuffer.resource.Reset();*/

    //ResetComPtrArray(&m_bottomLevelAS);
    m_bottomLevelAS.clear();
    m_topLevelAS.Reset();

    m_raytracingOutput.Reset();
    m_raytracingOutputResourceUAVDescriptorHeapIndex = UINT_MAX;
    m_rayGenShaderTable.Reset();
    m_missShaderTable.Reset();
    m_hitGroupShaderTable.Reset();

    m_resourceManager.reset();
    m_objects.clear();
}

void SelectiveSDF::InitializeScene()
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

    // import models
    // set up cb values for camera, colors, etc.

    // Setup camera.
    {
        //TODO: set initial camera position and orientation
        UpdateCameraMatrices();
    }

	// Add objects to the scene.
    {

    }
}

void SelectiveSDF::Raytrace()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

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
        commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::VertexBuffers, m_objects[0]->GetIndexBuffer().gpuDescriptorHandle);
        commandList->SetComputeRootDescriptorTable(GlobalRootSignature::Slot::OutputView, m_raytracingOutputResourceUAVGpuDescriptor);
    };

    commandList->SetComputeRootSignature(m_raytracingGlobalRootSignature.Get());

    // Copy dynamic buffers to GPU.
    {
		m_sceneCB.CopyStagingToGpu(frameIndex);
        commandList->SetComputeRootConstantBufferView(GlobalRootSignature::Slot::SceneConstant, m_sceneCB.GpuVirtualAddress(frameIndex));
    }

    // Bind the heaps, acceleration structure and dispatch rays.  
    D3D12_DISPATCH_RAYS_DESC dispatchDesc = {};
    SetCommonPipelineState(commandList);
    commandList->SetComputeRootShaderResourceView(GlobalRootSignature::Slot::AccelerationStructure, m_topLevelAS->GetGPUVirtualAddress());
    DispatchRays(m_dxrCommandList.Get(), m_dxrStateObject.Get(), &dispatchDesc);
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
        CD3DX12_DESCRIPTOR_RANGE ranges[2]; // Perfomance TIP: Order from most frequent to least frequent.
        ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);  // 1 output texture
        ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, m_objects.size() * 2, 1);  // index and vertex buffers per object

        CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignature::Slot::Count];
        rootParameters[GlobalRootSignature::Slot::OutputView].InitAsDescriptorTable(1, &ranges[0]);
        rootParameters[GlobalRootSignature::Slot::AccelerationStructure].InitAsShaderResourceView(0);
        rootParameters[GlobalRootSignature::Slot::SceneConstant].InitAsConstantBufferView(0);
        //rootParameters[GlobalRootSignature::Slot::AABBattributeBuffer].InitAsShaderResourceView(3);
        rootParameters[GlobalRootSignature::Slot::VertexBuffers].InitAsDescriptorTable(1, &ranges[1]);
        CD3DX12_ROOT_SIGNATURE_DESC globalRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);

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
    // Create subobjects that combine into a RTPSO:
    // Subobjects need to be associated with DXIL exports (i.e. shaders) either by way of default or explicit associations.
    // Default association applies to every exported shader entrypoint that doesn't have any of the same type of subobject associated with it.
    // This simple sample utilizes default shader association except for local root signature subobject
    // which has an explicit association specified purely for demonstration purposes.
    // 4 - DXIL library
    // 8 - Hit group types - 4 geometries (1 triangle, 3 aabb) x 2 ray types (ray, shadowRay)
    // 1 - Shader config
    // 6 - 3 x Local root signature and association
    // 1 - Global root signature
    // 1 - Pipeline config
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
    // PERFOMANCE TIP: Set max recursion depth as low as needed
    // as drivers may apply optimization strategies for low recursion depths.
    UINT maxRecursionDepth = MAX_RAY_RECURSION_DEPTH;
    pipelineConfig->Config(maxRecursionDepth);

    // Create the state object.
    ThrowIfFailed(m_dxrDevice->CreateStateObject(raytracingPipeline, IID_PPV_ARGS(&m_dxrStateObject)), L"Couldn't create DirectX Raytracing state object.\n");
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
        object->BuildGeometry(m_deviceResources->GetD3DDevice(), m_deviceResources->GetCommandList());

        auto& indexBuffer = object->GetIndexBuffer();
        auto& vertexBuffer = object->GetVertexBuffer();

        
        // Vertex buffer is passed to the shader along with index buffer as a descriptor range.
        UINT descriptorIndexIB = m_resourceManager->CreateBufferSRV(device, &indexBuffer, object->GetIndexCount() * sizeof(Index) / 4, 0);
        UINT descriptorIndexVB = m_resourceManager->CreateBufferSRV(device, &vertexBuffer, object->GetVertexCount() * sizeof(Vertex) / 4, 0);
        ThrowIfFalse(descriptorIndexVB == descriptorIndexIB + 1, L"Vertex Buffer descriptor index must follow that of Index Buffer descriptor index");
	}

    // Execute to complete the copies
    m_deviceResources->ExecuteCommandList();
    m_deviceResources->WaitForGpu();

    for (auto& object : m_objects)
    {
        object->ReleaseStagingBuffers();
    }
}

void SelectiveSDF::BuildGeometryDescsForBottomLevelAS(vector<vector<D3D12_RAYTRACING_GEOMETRY_DESC>>& geometryDescs)
{
    // Mark the geometry as opaque.
    D3D12_RAYTRACING_GEOMETRY_FLAGS geometryFlags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;


	//geometryDescs.resize(m_objects.size());
    
    for(UINT i = 0; i < m_objects.size(); i++)
    {
        switch(m_objects[i]->GetType())
        {
            case ObjectType::Hybrid:
            {
				vector<D3D12_RAYTRACING_GEOMETRY_DESC> geometryDescsForHybridObject;

                auto hybridObject = static_cast<HybridObject*>(m_objects[i].get());
                auto aabbBuffer = hybridObject->GetAABBBuffer();
                UINT aabbCount = hybridObject->GetAAbbCount();


                D3D12_RAYTRACING_GEOMETRY_DESC templateDesc = {};
                templateDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS;
                templateDesc.AABBs.AABBCount = aabbCount;
                templateDesc.AABBs.AABBs.StrideInBytes = sizeof(D3D12_RAYTRACING_AABB);
                templateDesc.Flags = geometryFlags;

                geometryDescsForHybridObject.resize(aabbCount, templateDesc);

                for (UINT j = 0; j < aabbCount; j++)
                {
                    geometryDescsForHybridObject[j].AABBs.AABBs.StartAddress = aabbBuffer.resource->GetGPUVirtualAddress() + j * sizeof(D3D12_RAYTRACING_AABB);
                }

				geometryDescs.push_back(move(geometryDescsForHybridObject));
                //break;
            }
            case ObjectType::Triangle:
            {
                vector<D3D12_RAYTRACING_GEOMETRY_DESC> geometryDescsForObject;

                auto indexBuffer = m_objects[i]->GetIndexBuffer();
                auto vertexBuffer = m_objects[i]->GetVertexBuffer();

                geometryDescsForObject.resize(1);

                auto& geometryDesc = geometryDescsForObject[0];

                // TODO: set vertex and index buffers, their counts and startaddesses for each triangle geometry
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
                break;
            }
            default:
                throw std::runtime_error("Unknown object type for geometry descs");
		}
	}
}

template <class InstanceDescType, class BLASPtrType>
void SelectiveSDF::BuildBottomLevelASInstanceDescs(vector<BLASPtrType>& bottomLevelASaddresses, ComPtr<ID3D12Resource>* instanceDescsResource)
{
    //const UINT BlasCount = bottomLevelASaddresses.size();

    auto device = m_deviceResources->GetD3DDevice();

    vector<InstanceDescType> instanceDescs;
    UINT blasIndex = 0;
    for(UINT i = 0; i < m_objects.size(); i++)
    {
        InstanceDescType aabbInstanceDesc = {};
        if (m_objects[i]->GetType() == ObjectType::Hybrid)
        {
            auto hybridObject = static_cast<HybridObject*>(m_objects[i].get());

            aabbInstanceDesc.InstanceMask = 2;
            aabbInstanceDesc.InstanceContributionToHitGroupIndex = hybridObject->GetHitGroupIndex();
            aabbInstanceDesc.AccelerationStructure = bottomLevelASaddresses[blasIndex++];
        }

        InstanceDescType instanceDesc = {};
        instanceDesc.InstanceID = i;
        instanceDesc.InstanceMask = 1;
        instanceDesc.InstanceContributionToHitGroupIndex = 0;
        instanceDesc.AccelerationStructure = bottomLevelASaddresses[blasIndex++];
		
		auto& instanceData = m_objects[i]->GetInstanceData();
        for(auto& instance : instanceData)
        {
            if (m_objects[i]->GetType() == ObjectType::Hybrid)
            {
                XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(aabbInstanceDesc.Transform), instance.CalculateTransform());
                instanceDescs.push_back(aabbInstanceDesc);
            }

            XMStoreFloat3x4(reinterpret_cast<XMFLOAT3X4*>(instanceDesc.Transform), instance.CalculateTransform());
            instanceDescs.push_back(instanceDesc);
		}
	}

    if(instanceDescs.empty())
    {
        throw std::runtime_error("No instance descriptions were created from bottom level acceleration structure.");
	}

    UINT64 bufferSize = static_cast<UINT64>(instanceDescs.size() * sizeof(instanceDescs[0]));

    AllocateBuffer(
        device, 
        D3D12_HEAP_TYPE_UPLOAD,
        bufferSize, 
        &(*instanceDescsResource), 
        D3D12_RESOURCE_STATE_GENERIC_READ,
		D3D12_RESOURCE_FLAG_NONE,
        instanceDescs.data(),
        L"InstanceDescs"
    );
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
    // Acceleration structures can only be placed in resources that are created in the default heap (or custom heap equivalent). 
    // Default heap is OK since the application doesn’t need CPU read/write access to them. 
    // The resources that will contain acceleration structures must be created in the state D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, 
    // and must have resource flag D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS. The ALLOW_UNORDERED_ACCESS requirement simply acknowledges both: 
    //  - the system will be doing this type of access in its implementation of acceleration structure builds behind the scenes.
    //  - from the app point of view, synchronization of writes/reads to acceleration structures is accomplished using UAV barriers.
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

AccelerationStructureBuffers SelectiveSDF::BuildTopLevelAS(AccelerationStructureBuffers* bottomLevelAS, UINT bottomLevelASCount, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags)
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();
    ComPtr<ID3D12Resource> scratch;
    ComPtr<ID3D12Resource> topLevelAS;

    // Get required sizes for an acceleration structure.
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC topLevelBuildDesc = {};
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS& topLevelInputs = topLevelBuildDesc.Inputs;
    topLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    topLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    topLevelInputs.Flags = buildFlags;
    topLevelInputs.NumDescs = bottomLevelASCount;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO topLevelPrebuildInfo = {};
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&topLevelInputs, &topLevelPrebuildInfo);
    ThrowIfFalse(topLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    AllocateBuffer(
        device, 
        D3D12_HEAP_TYPE_DEFAULT, 
        topLevelPrebuildInfo.ScratchDataSizeInBytes, 
        &scratch, 
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS, 
        D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, 
        nullptr, 
        L"ScratchResource"
    );

    // Allocate resources for acceleration structures.
    // Acceleration structures can only be placed in resources that are created in the default heap (or custom heap equivalent). 
    // Default heap is OK since the application doesn’t need CPU read/write access to them. 
    // The resources that will contain acceleration structures must be created in the state D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, 
    // and must have resource flag D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS. The ALLOW_UNORDERED_ACCESS requirement simply acknowledges both: 
    //  - the system will be doing this type of access in its implementation of acceleration structure builds behind the scenes.
    //  - from the app point of view, synchronization of writes/reads to acceleration structures is accomplished using UAV barriers.
    {
        D3D12_HEAP_TYPE heapType = D3D12_HEAP_TYPE_DEFAULT;
        D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;
        D3D12_RESOURCE_FLAGS resourceFlags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS;
        AllocateBuffer(
            device,
            heapType,
            topLevelPrebuildInfo.ResultDataMaxSizeInBytes,
            &topLevelAS,
            initialResourceState,
            resourceFlags,
            nullptr,
            L"TopLevelAccelerationStructure"
        );
    }


    // Create instance descs for the bottom-level acceleration structures.
    ComPtr<ID3D12Resource> instanceDescsResource;
    {
		vector<D3D12_GPU_VIRTUAL_ADDRESS> bottomLevelASaddresses;
		bottomLevelASaddresses.reserve(bottomLevelASCount);

        for(UINT i = 0; i < bottomLevelASCount; i++)
        {
            bottomLevelASaddresses.push_back(bottomLevelAS[i].accelerationStructure->GetGPUVirtualAddress());
		}
        BuildBottomLevelASInstanceDescs<D3D12_RAYTRACING_INSTANCE_DESC>(bottomLevelASaddresses, &instanceDescsResource);
    }

    // Top-level AS desc
    {
        topLevelBuildDesc.DestAccelerationStructureData = topLevelAS->GetGPUVirtualAddress();
        topLevelInputs.InstanceDescs = instanceDescsResource->GetGPUVirtualAddress();
        topLevelBuildDesc.ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress();
    }

    // Build acceleration structure.
    m_dxrCommandList->BuildRaytracingAccelerationStructure(&topLevelBuildDesc, 0, nullptr);

    AccelerationStructureBuffers topLevelASBuffers;
    topLevelASBuffers.accelerationStructure = topLevelAS;
    topLevelASBuffers.instanceDesc = instanceDescsResource;
    topLevelASBuffers.scratch = scratch;
    topLevelASBuffers.ResultDataMaxSizeInBytes = topLevelPrebuildInfo.ResultDataMaxSizeInBytes;
    return topLevelASBuffers;
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
    //const UINT BlasCount = TrianglePrimitive::Count + IntersectionShaderType::TotalPrimitiveCount;
    vector<AccelerationStructureBuffers> bottomLevelAS;
    vector<vector<D3D12_RAYTRACING_GEOMETRY_DESC>> geometryDescs;
	//geometryDescs.resize(BlasCount);
    {
        BuildGeometryDescsForBottomLevelAS(geometryDescs);
		bottomLevelAS.resize(geometryDescs.size());

        // Build all bottom-level AS.
        for (UINT i = 0; i < geometryDescs.size(); i++)
        {
            bottomLevelAS[i] = BuildBottomLevelAS(geometryDescs[i]);
        }
    }

	UINT BlasCount = static_cast<UINT>(bottomLevelAS.size());

    // Batch all resource barriers for bottom-level AS builds.
    vector<D3D12_RESOURCE_BARRIER> resourceBarriers;
    resourceBarriers.resize(BlasCount);
    for (UINT i = 0; i < BlasCount; i++)
    {
        resourceBarriers[i] = CD3DX12_RESOURCE_BARRIER::UAV(bottomLevelAS[i].accelerationStructure.Get());
    }
    commandList->ResourceBarrier(BlasCount, resourceBarriers.data());


    // Build top-level AS.
    AccelerationStructureBuffers topLevelAS = BuildTopLevelAS(bottomLevelAS.data(), BlasCount);

    // Kick off acceleration structure construction.
    m_deviceResources->ExecuteCommandList();

    // Wait for GPU to finish as the locally created temporary GPU resources will get released once we go out of scope.
    m_deviceResources->WaitForGpu();

    // Store the AS buffers. The rest of the buffers will be released once we exit the function.
    m_bottomLevelAS.resize(BlasCount);
    for (UINT i = 0; i < BlasCount; i++)
    {
        m_bottomLevelAS[i] = bottomLevelAS[i].accelerationStructure;
    }
    m_topLevelAS = topLevelAS.accelerationStructure;
}

void SelectiveSDF::CreateConstantBuffers()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto frameCount = m_deviceResources->GetBackBufferCount();

    m_sceneCB.Create(device, frameCount, L"Scene Constant Buffer");
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

void SelectiveSDF::UpdateCameraMatrices()
{
    auto frameIndex = m_deviceResources->GetCurrentFrameIndex();

	m_sceneCB->viewI = XMMatrixInverse(nullptr, m_camera.GetViewMatrix());
	m_sceneCB->projectionI = XMMatrixInverse(nullptr, m_camera.GetProjectionMatrix());
}
