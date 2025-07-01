#pragma once

#include "DXSample.h"
#include "DXRaytracingHelper.h"
#include "SceneDefines.h"
#include "StepTimer.h"
#include "ConstantBuffer.hpp"
#include "StructuredBuffer.hpp"
#include "Camera.h"
#include "Plane.h"
#include "Box.h"
#include "Sphere.h"
#include "AModel.h"
#include "SpatialHashGrid.hpp"
#include "BVHBuilder.hpp"

using namespace std;
using namespace ConstantBufferTypes;

class SelectiveSDF : public DXSample
{
public:
	SelectiveSDF(UINT width, UINT height, std::wstring name);

    // IDeviceNotify
    virtual void OnDeviceLost() override;
    virtual void OnDeviceRestored() override;

    // DXSample
    virtual void OnInit();
    virtual void OnUpdate();
    virtual void OnRender();
    virtual void OnSizeChanged(UINT width, UINT height, bool minimized) {};
    virtual void OnDestroy();
    virtual void OnKeyDown(UINT8 key) {};
    virtual IDXGISwapChain* GetSwapchain() { return m_deviceResources->GetSwapChain(); }


private:
    static const UINT FrameCount = 2;
    UINT SDFInstanceCount = 0;

	unique_ptr<ResourceManager> m_resourceManager;

    // DirectX Raytracing (DXR) attributes
    ComPtr<ID3D12Device5> m_dxrDevice;
    ComPtr<ID3D12GraphicsCommandList5> m_dxrCommandList;
    ComPtr<ID3D12StateObject> m_dxrStateObject;

    // Root signatures
    ComPtr<ID3D12RootSignature> m_raytracingGlobalRootSignature;
    ComPtr<ID3D12RootSignature> m_raytracingLocalRootSignature[LocalRootSignature::Type::Count];

    // Geometry
	std::vector<unique_ptr<Object>> m_objects;
    std::vector<ObjectInstance> m_instances;

    // Raytracing scene
    ConstantBuffer<SceneConstantBuffer> m_sceneCB;
    std::vector<D3D12_RAYTRACING_AABB> m_aabbs;

    StructuredBuffer<SDFObjectData> m_sdfObjectsSB;

    /*StructuredBuffer<HashTableEntry> m_hashTableSB;
    StructuredBuffer<InstanceIndex> m_instanceIndicesSB;*/

    // Acceleration structure
    std::vector<ComPtr<ID3D12Resource>> m_bottomLevelAS;
    unique_ptr<AccelerationStructureBuffers> m_topLevelAS;
    //UINT m_tlasInstanceCount = 0;

    // Raytracing output
    ComPtr<ID3D12Resource> m_raytracingOutput;
    D3D12_GPU_DESCRIPTOR_HANDLE m_raytracingOutputResourceUAVGpuDescriptor;
    UINT m_raytracingOutputResourceUAVDescriptorHeapIndex = UINT_MAX;

    D3D12_GPU_DESCRIPTOR_HANDLE m_sdfTexturesGpuDescriptor;

    // Shader tables
    static const wchar_t* c_hitGroupNames_TriangleGeometry[RayType::Count];
    static const wchar_t* c_hitGroupNames_AABBGeometry[IntersectionShaderType::Count][RayType::Count];
    static const wchar_t* c_raygenShaderName;
    static const wchar_t* c_intersectionShaderNames[IntersectionShaderType::Count];
    static const wchar_t* c_closestHitShaderNames[GeometryType::Count];
    static const wchar_t* c_missShaderNames[RayType::Count];

    ComPtr<IDxcBlob> m_rayGenShaderBlob, m_missShaderBlob, m_proceduralShaderBlob, m_triangleShaderBlob;

    ComPtr<ID3D12Resource> m_missShaderTable;
    UINT m_missShaderTableStrideInBytes;
    ComPtr<ID3D12Resource> m_hitGroupShaderTable;
    UINT m_hitGroupShaderTableStrideInBytes;
    ComPtr<ID3D12Resource> m_rayGenShaderTable;

    // Scene
    Camera m_camera;
    StepTimer m_timer;
    BVHBuilder m_bvhBuilder;

    void CreateDeviceDependentResources();
    void CreateWindowSizeDependentResources();
    void ReleaseDeviceDependentResources();
    void ReleaseWindowSizeDependentResources() {};

    void InitializeScene();
	void Raytrace();
    void CopyRaytracingOutputToBackbuffer();


	void CreateAuxilaryDeviceResources();
    void CreateRaytracingInterfaces();
    void CreateRootSignatures();
    void SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc, ComPtr<ID3D12RootSignature>* rootSig);
	void CreateRaytracingPipelineStateObject();
    void CreateDxilLibrarySubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
    void CreateHitGroupSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
    void CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
    void BuildGeometry();
    void BuildGeometryDescsForBottomLevelAS(vector<vector<D3D12_RAYTRACING_GEOMETRY_DESC>>& geometryDescs);
    template <class InstanceDescType, class BLASPtrType>
    void BuildBottomLevelASInstanceDescs(vector<BLASPtrType>& bottomLevelASaddresses, UINT& instanceDescsCount, bool updateOnly = false);
    AccelerationStructureBuffers BuildBottomLevelAS(const vector<D3D12_RAYTRACING_GEOMETRY_DESC>& geometryDescs, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE);
    void BuildTopLevelAS(AccelerationStructureBuffers* bottomLevelAS, UINT bottomLevelASCount, D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE);
    void UpdateTopLevelAS();
    void BuildAccelerationStructures();
    void CreateConstantBuffers();
    void CreateStructuredBuffers();
    void BuildShaderTables();
	void CreateRaytracingOutputResource();

    //void BuildSpatialHashGrid();
    void BuildSDfBVH();
    void CalculateFrameStats();

    // Application state
    void BuildSDFObjectsData();
    void UpdateCameraMatrices();
    void UpdateSDFObjectsData();
    //void BuildHashGridData();
};

inline bool CheckRaytracingSupport(IDXGIAdapter1* adapter)
{
    ComPtr<ID3D12Device> testDevice;
    D3D12_FEATURE_DATA_D3D12_OPTIONS5 featureSupportData = {};

    return SUCCEEDED(D3D12CreateDevice(adapter, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&testDevice)))
        && SUCCEEDED(testDevice->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &featureSupportData, sizeof(featureSupportData)))
        && featureSupportData.RaytracingTier != D3D12_RAYTRACING_TIER_NOT_SUPPORTED;
}
inline bool CheckRayQuerySupport(ID3D12Device* device)
{
    D3D12_FEATURE_DATA_D3D12_OPTIONS5 options5 = {};
    device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &options5, sizeof(options5));
    return options5.RaytracingTier >= D3D12_RAYTRACING_TIER_1_1;
}