#include "stdafx.h"
#include "AModel.h"
#include <algorithm>
#include <map>

using namespace std;

inline void ProcessNode(const aiNode* node, const aiScene* scene, vector<Vertex>& vertices, vector<Index>& indices);
inline void ProcessMesh(const aiMesh* mesh, const aiScene* scene, vector<Vertex>& vertices, vector<UINT>& indices);
inline vector<float> LoadSDFTexture(const string& filename, const XMUINT3& resolution);


// Single contiguous buffer - everything linearized
struct NanoVDBStyle {
	uint8_t* buffer;           // One big allocation
	uint32_t bufferSize;
	uint32_t rootOffset;       // Where to start traversal
	XMUINT3 resolution;
	float voxelSize;
	float backgroundValue;     // Default value for empty space
};

// Fixed-size node (exactly like NanoVDB)
struct CompactNode {
	uint32_t childOffsets[8];  // Byte offsets to children (0 = no child)
	float minSDF, maxSDF;      // Bounds 
	uint32_t nodeType;         // 0=background_tile, 1=interior_tile, 2=leaf
	float tileValue;           // Constant value for tiles
	uint32_t voxelDataOffset;  // Offset to 8³ voxel data for leaves
	// Pad to 64 bytes for GPU alignment
};
struct BlockInfo {
	XMUINT3 blockCoord;
	uint32_t nodeType;      // 0=background, 1=interior, 2=leaf
	float tileValue;        // For tiles
	float minSDF, maxSDF;   // Bounds
	vector<float> voxelData;  // Only filled for leaves
};

inline void BuildTree(const vector<float>& sdfData, XMUINT3 resolution);
inline BlockInfo AnalyzeBlock(const vector<float>& sdfData, XMUINT3 resolution, XMUINT3 blockCoord, float narrowBand);

void AModel::BuildSDF(ID3D12Device* device, ID3D12GraphicsCommandList* commandList)
{
	const XMUINT3 resolution = { m_sdfResolution, m_sdfResolution, m_sdfResolution };
	vector<float> sdfData = LoadSDFTexture("res/sdfs/" + m_fileName + ".raw", resolution);


	const UINT rowSize = resolution.x * sizeof(float);
	const UINT alignedRowPitch = (rowSize + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1) & ~(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1); // 256
	const UINT slicePitch = alignedRowPitch * resolution.y;
	const UINT totalSize = slicePitch * resolution.z;
	std::vector<BYTE> paddedUploadData(totalSize, 0);

	for (UINT z = 0; z < resolution.z; ++z)
	{
		for (UINT y = 0; y < resolution.y; ++y)
		{
			const float* srcRow = &sdfData[(z * resolution.y + y) * resolution.x];
			BYTE* dstRow = &paddedUploadData[z * slicePitch + y * alignedRowPitch];
			memcpy(dstRow, srcRow, rowSize);
		}
	}

	AllocateTexture(
		device, 
		resolution, 
		&m_sdfTextureBuffer.resource,
		DXGI_FORMAT_R32_FLOAT,
		D3D12_RESOURCE_DIMENSION_TEXTURE3D,
		D3D12_RESOURCE_STATE_COPY_DEST, 
		D3D12_RESOURCE_FLAG_NONE,
		L"Dense SDF Texture Buffer"
	);
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_UPLOAD,
		totalSize,
		&m_stagingSdfTextureBuffer,
		D3D12_RESOURCE_STATE_GENERIC_READ,
		D3D12_RESOURCE_FLAG_NONE,
		paddedUploadData.data(),
		L"Dense SDF Texture Staging Buffer"
	);

	D3D12_SUBRESOURCE_DATA textureSubresource = {};
	textureSubresource.pData = paddedUploadData.data();
	textureSubresource.RowPitch = alignedRowPitch;
	textureSubresource.SlicePitch = slicePitch;

	UpdateSubresources(commandList, m_sdfTextureBuffer.resource.Get(), m_stagingSdfTextureBuffer.Get(), 0, 0, 1, &textureSubresource);


	auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
		m_sdfTextureBuffer.resource.Get(),
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE
	);
	commandList->ResourceBarrier(1, &barrier);
}

void AModel::BuildSVS(ID3D12Device* device, ID3D12GraphicsCommandList* commandList)
{
	const UINT bufferDataSize = static_cast<UINT>(m_surfaceVoxels.size() * sizeof(SurfaceVoxel));

	// Allocate default heap buffer for the surface voxels
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_DEFAULT,
		bufferDataSize,
		&m_sdfVoxelBuffer.resource,
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_FLAG_NONE,
		nullptr);

	// Allocate upload buffer
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_UPLOAD,
		bufferDataSize,
		&m_stagingSdfVoxelBuffer,
		D3D12_RESOURCE_STATE_GENERIC_READ,
		D3D12_RESOURCE_FLAG_NONE,
		m_surfaceVoxels.data());

	commandList->CopyResource(m_sdfVoxelBuffer.resource.Get(), m_stagingSdfVoxelBuffer.Get());

	// Transition to shader resource
	auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
		m_sdfVoxelBuffer.resource.Get(),
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE
	);
	commandList->ResourceBarrier(1, &barrier);
}

void AModel::BuildLeafAtlas(ID3D12Device* device, ID3D12GraphicsCommandList* commandList)
{
	const int LEAF_SIZE = 8;
	UINT sliceCount = m_leafLevel.size() * LEAF_SIZE;

	vector<float> textureData(sliceCount * LEAF_SIZE * LEAF_SIZE, 0.0f);


	UINT currentSlice = 0;
	for (const auto& leaf : m_leafLevel) {
		for (int z = 0; z < LEAF_SIZE; z++) {
			UINT sliceIdx = currentSlice + z;
			UINT sliceStart = sliceIdx * LEAF_SIZE * LEAF_SIZE;

			for (int y = 0; y < LEAF_SIZE; y++) {
				for (int x = 0; x < LEAF_SIZE; x++) {
					UINT idx = sliceStart + y * LEAF_SIZE + x;
					textureData[idx] = leaf.voxels[x][y][z];
				}
			}
		}

		currentSlice += LEAF_SIZE;
	}


	// Create texture atlas using your existing pattern
	const UINT sliceWidth = LEAF_SIZE;
	const UINT sliceHeight = LEAF_SIZE;
	const UINT rowSize = sliceWidth * sizeof(float);
	const UINT alignedRowPitch = (rowSize + D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1) & ~(D3D12_TEXTURE_DATA_PITCH_ALIGNMENT - 1);
	const UINT slicePitch = alignedRowPitch * sliceHeight;
	const UINT totalSize = slicePitch * sliceCount;

	// Prepare padded data
	std::vector<BYTE> paddedUploadData(totalSize, 0);

	// Copy texture data with padding
	for (UINT sliceIdx = 0; sliceIdx < sliceCount; sliceIdx++) {
		for (UINT y = 0; y < sliceHeight; y++) {
			const float* srcRow = &textureData[(sliceIdx * sliceHeight + y) * sliceWidth];
			BYTE* dstRow = &paddedUploadData[sliceIdx * slicePitch + y * alignedRowPitch];
			memcpy(dstRow, srcRow, rowSize);
		}
	}

	// Using your AllocateTexture pattern but for 2D array
	AllocateTexture(device, {sliceWidth, sliceHeight, sliceCount}, &m_leafAtlasBuffer.resource, DXGI_FORMAT_R32_FLOAT, D3D12_RESOURCE_DIMENSION_TEXTURE2D);
	AllocateBuffer(
	    device,
	    D3D12_HEAP_TYPE_UPLOAD,
	    totalSize,
	    &m_stagingLeafAtlasBuffer,
	    D3D12_RESOURCE_STATE_GENERIC_READ,
	    D3D12_RESOURCE_FLAG_NONE,
	    paddedUploadData.data());

	// Prepare subresources
	vector<D3D12_SUBRESOURCE_DATA> subresources(sliceCount);
	for (UINT i = 0; i < sliceCount; i++) {
	    subresources[i].pData = &paddedUploadData[i * slicePitch];
	    subresources[i].RowPitch = alignedRowPitch;
	    subresources[i].SlicePitch = slicePitch;
	}

	UpdateSubresources(commandList, m_leafAtlasBuffer.resource.Get(), m_stagingLeafAtlasBuffer.Get(),
	    0, 0, sliceCount, subresources.data());

	auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
		m_leafAtlasBuffer.resource.Get(),
	    D3D12_RESOURCE_STATE_COPY_DEST,
	    D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
	commandList->ResourceBarrier(1, &barrier);
}

void AModel::BuildLeafLevel(ID3D12Device* device, ID3D12GraphicsCommandList* commandList)
{
	vector<LeafNodeData> leafNodes;
	leafNodes.resize(m_leafLevel.size());

	for (int i = 0; i < m_leafLevel.size(); i++) 
	{
		leafNodes[i].coord = m_leafLevel[i].coord;
		leafNodes[i].sliceIndex = i * 8; // Each leaf node corresponds to 8 slices in the atlas
		memcpy(leafNodes[i].bitmask, m_leafLevel[i].bitmask, sizeof(m_leafLevel[i].bitmask));
	}

	const UINT bufferDataSize = static_cast<UINT>(leafNodes.size() * sizeof(LeafNodeData));

	// Allocate default heap buffer for the surface voxels
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_DEFAULT,
		bufferDataSize,
		&m_leafNodeBuffer.resource,
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_FLAG_NONE,
		nullptr);

	// Allocate upload buffer
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_UPLOAD,
		bufferDataSize,
		&m_stagingLeafNodeBuffer,
		D3D12_RESOURCE_STATE_GENERIC_READ,
		D3D12_RESOURCE_FLAG_NONE,
		leafNodes.data());

	commandList->CopyResource(m_leafNodeBuffer.resource.Get(), m_stagingLeafNodeBuffer.Get());

	// Transition to shader resource
	auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
		m_leafNodeBuffer.resource.Get(),
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE
	);
	commandList->ResourceBarrier(1, &barrier);
}

void AModel::BuildInternalLevel1(ID3D12Device* device, ID3D12GraphicsCommandList* commandList)
{
	const UINT bufferDataSize = static_cast<UINT>(m_internalLevel1.size() * sizeof(InternalNode));

	// Allocate default heap buffer for the surface voxels
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_DEFAULT,
		bufferDataSize,
		&m_internal1NodeBuffer.resource,
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_FLAG_NONE,
		nullptr);

	// Allocate upload buffer
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_UPLOAD,
		bufferDataSize,
		&m_stagingInternal1NodeBuffer,
		D3D12_RESOURCE_STATE_GENERIC_READ,
		D3D12_RESOURCE_FLAG_NONE,
		m_internalLevel1.data());

	commandList->CopyResource(m_internal1NodeBuffer.resource.Get(), m_stagingInternal1NodeBuffer.Get());

	// Transition to shader resource
	auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
		m_internal1NodeBuffer.resource.Get(),
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE
	);
	commandList->ResourceBarrier(1, &barrier);
}

void AModel::BuildInternalLevel2(ID3D12Device* device, ID3D12GraphicsCommandList* commandList)
{
	const UINT bufferDataSize = static_cast<UINT>(m_internalLevel2.size() * sizeof(InternalNode));

	// Allocate default heap buffer for the surface voxels
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_DEFAULT,
		bufferDataSize,
		&m_internal2NodeBuffer.resource,
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_FLAG_NONE,
		nullptr);

	// Allocate upload buffer
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_UPLOAD,
		bufferDataSize,
		&m_stagingInternal2NodeBuffer,
		D3D12_RESOURCE_STATE_GENERIC_READ,
		D3D12_RESOURCE_FLAG_NONE,
		m_internalLevel2.data());

	commandList->CopyResource(m_internal2NodeBuffer.resource.Get(), m_stagingInternal2NodeBuffer.Get());

	// Transition to shader resource
	auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
		m_internal2NodeBuffer.resource.Get(),
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE
	);
	commandList->ResourceBarrier(1, &barrier);
}

void AModel::BuildRoot(ID3D12Device* device, ID3D12GraphicsCommandList* commandList)
{
	const UINT bufferDataSize = static_cast<UINT>(1 * sizeof(InternalNode)); // Only one root node

	// Allocate default heap buffer for the surface voxels
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_DEFAULT,
		bufferDataSize,
		&m_rootNodeBuffer.resource,
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_FLAG_NONE,
		nullptr);

	// Allocate upload buffer
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_UPLOAD,
		bufferDataSize,
		&m_stagingRootNodeBuffer,
		D3D12_RESOURCE_STATE_GENERIC_READ,
		D3D12_RESOURCE_FLAG_NONE,
		&m_rootNode);

	commandList->CopyResource(m_rootNodeBuffer.resource.Get(), m_stagingRootNodeBuffer.Get());

	// Transition to shader resource
	auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
		m_rootNodeBuffer.resource.Get(),
		D3D12_RESOURCE_STATE_COPY_DEST,
		D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE
	);
	commandList->ResourceBarrier(1, &barrier);
}

void AModel::BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices)
{
	// Create importer
	Assimp::Importer importer;

	// Load file with post-processing flags
	const aiScene* scene = importer.ReadFile("res/models/" + m_fileName + ".obj",
		aiProcess_Triangulate |              // Convert to triangles
		aiProcess_GenNormals |               // Generate normals if missing
		aiProcess_CalcTangentSpace |         // Calculate tangents/bitangents
		aiProcess_JoinIdenticalVertices |    // Remove duplicate vertices
		aiProcess_OptimizeMeshes            // Optimize for rendering
	);

	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
	{
		// Error loading
		throw logic_error(importer.GetErrorString());
	}

	ProcessNode(scene->mRootNode, scene, vertices, indices);

	m_vertexCount = vertices.size();
    m_indexCount = indices.size();
}

inline void UnpackBrickCoord(uint32_t packed, uint32_t& x, uint32_t& y, uint32_t& z)
{
	x = packed & 0xFF;
	y = (packed >> 8) & 0xFF;
	z = (packed >> 16) & 0xFF;
}
void AModel::BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs)
{
	// only one hardcoded AABB for now
	/*m_aabbCount = 1;

	aabbs.push_back(
		{ -0.5f, -0.5f, -0.5f, 0.5f,  0.5f,  0.5f }
	);
	return;*/

	const int GRID_SIZE = m_sdfResolution;
	const float WORLD_SIZE = 1.0f;  // -0.5 to +0.5
	const float VOXEL_SIZE = WORLD_SIZE / GRID_SIZE;
	const float HALF_WORLD = WORLD_SIZE * 0.5f;

	const int BRICK_SIZE = 9;
	m_aabbCount = static_cast<UINT>(m_brickMeta.size());
	for (const auto& brick : m_brickMeta) // m_surfaceBricks 
	{
		UINT brickX, brickY, brickZ;
		UnpackBrickCoord(brick.packedCoord, brickX, brickY, brickZ);

		float worldX = (brickX * BRICK_SIZE * VOXEL_SIZE) - HALF_WORLD;
		float worldY = (brickY * BRICK_SIZE * VOXEL_SIZE) - HALF_WORLD;
		float worldZ = (brickZ * BRICK_SIZE * VOXEL_SIZE) - HALF_WORLD;

		aabbs.push_back({
			worldX, worldY, worldZ,
			worldX + BRICK_SIZE * VOXEL_SIZE,
			worldY + BRICK_SIZE * VOXEL_SIZE,
			worldZ + BRICK_SIZE * VOXEL_SIZE
		});
	}
}

void AModel::BuildVDBSBS()
{
	struct LeafBrick {
		// Position in voxel coordinates (0-56 in steps of 8)
		XMINT3 origin;

		// Padding to ensure 16-byte alignment for what follows
		uint32_t padding1;

		// SDF values for 8x8x8 voxels (2048 bytes)
		float sdf[512];

		// Active voxel mask - 512 bits = 16 * 32 bits (64 bytes)
		uint32_t activeMask[16];

		// Min/max values in this brick for optimization (8 bytes)
		float minValue;
		float maxValue;

		// Optional: additional metadata (8 bytes)
		uint32_t voxelCount;    // Number of active voxels
		uint32_t padding2;      // Align to 16 bytes

		// Total size: 2048 + 64 + 16 + 16 = 2144 bytes
	};

	vector<LeafBrick> leafBricks;
	const int GRID_SIZE = 64;
	const int BRICK_SIZE = 8;
	const float VOXEL_SIZE = 1.0f / GRID_SIZE;
	const float NARROW_BAND_WIDTH = 3.0f * VOXEL_SIZE;

	vector<float> sdfData = LoadSDFTexture("res/sdfs/" + m_fileName + ".raw", { GRID_SIZE, GRID_SIZE, GRID_SIZE });

	auto sampleSDF = [&](int x, int y, int z) -> float {
		x = clamp(x, 0, GRID_SIZE - 1);
		y = clamp(y, 0, GRID_SIZE - 1);
		z = clamp(z, 0, GRID_SIZE - 1);
		return sdfData[z * GRID_SIZE * GRID_SIZE + y * GRID_SIZE + x];
	};

	for (int bz = 0; bz < GRID_SIZE; bz += BRICK_SIZE) {
		for (int by = 0; by < GRID_SIZE; by += BRICK_SIZE) {
			for (int bx = 0; bx < GRID_SIZE; bx += BRICK_SIZE) {
				LeafBrick brick;
				brick.origin = XMINT3(bx, by, bz);
				brick.padding1 = 0;
				brick.padding2 = 0;

				// Initialize all mask bits to 0
				memset(brick.activeMask, 0, sizeof(brick.activeMask));

				// Initialize min/max and voxel count
				brick.minValue = FLT_MAX;
				brick.maxValue = -FLT_MAX;
				brick.voxelCount = 0;

				bool hasActiveVoxel = false;

				for (int z = 0; z < BRICK_SIZE; ++z) {
					for (int y = 0; y < BRICK_SIZE; ++y) {
						for (int x = 0; x < BRICK_SIZE; ++x) {
							float val = sampleSDF(bx + x, by + y, bz + z);
							int idx = z * BRICK_SIZE * BRICK_SIZE + y * BRICK_SIZE + x;
							brick.sdf[idx] = val;

							// Update min/max for ALL voxels in brick
							brick.minValue = min(brick.minValue, val);
							brick.maxValue = max(brick.maxValue, val);

							// Check if active (in narrow band)
							if (abs(val) <= NARROW_BAND_WIDTH) {
								hasActiveVoxel = true;
								brick.voxelCount++;

								// Set bit in mask
								int maskIndex = idx / 32;
								int bitIndex = idx % 32;
								brick.activeMask[maskIndex] |= (1U << bitIndex);
							}
						}
					}
				}

				if (hasActiveVoxel) {
					leafBricks.push_back(brick);
				}
			}
		}
	}
}

inline void BuildTree(const vector<float>& sdfData, XMUINT3 resolution)
{
	float narrowBand = 1.5f / 64.0f;  // 1.5 voxels worth
	float backgroundValue = 3.0f;     // Far outside
	float interiorValue = -3.0f;      // Far inside

	// Step 1: Analyze all 8x8x8 blocks

	vector<BlockInfo> blocks;

	// 32³ texture = 4x4x4 blocks of 8³ each
	for (UINT bz = 0; bz < 8; bz++) {
		for (UINT by = 0; by < 8; by++) {
			for (UINT bx = 0; bx < 8; bx++) {
				BlockInfo info = AnalyzeBlock(sdfData, resolution, XMUINT3{ bx, by, bz }, narrowBand);
				blocks.push_back(info);
			}
		}
	}

}
inline BlockInfo AnalyzeBlock(const vector<float>& sdfData, XMUINT3 resolution, XMUINT3 blockCoord, float narrowBand) 
{
	BlockInfo info;
	info.blockCoord = blockCoord;

	float minSDF = FLT_MAX;
	float maxSDF = -FLT_MAX;
	vector<float> voxels;
	voxels.reserve(8 * 8 * 8);

	// Sample all 512 voxels in this 8³ block
	for (int z = 0; z < 8; z++) {
		for (int y = 0; y < 8; y++) {
			for (int x = 0; x < 8; x++) {
				// Convert block+local coords to global texture coords
				uint32_t gx = blockCoord.x * 8 + x;
				uint32_t gy = blockCoord.y * 8 + y;
				uint32_t gz = blockCoord.z * 8 + z;

				// Index into flat array (Z-major order)
				uint32_t index = gz * resolution.x * resolution.y + gy * resolution.x + gx;

				float sdf = sdfData[index];
				voxels.push_back(sdf);

				minSDF = min(minSDF, sdf);
				maxSDF = max(maxSDF, sdf);
			}
		}
	}

	info.minSDF = minSDF;
	info.maxSDF = maxSDF;

	// Decide: tile or leaf?
	if (minSDF > narrowBand) {
		// All far outside - background tile
		info.nodeType = 0;
		info.tileValue = 3.0f;  // Background value
	}
	else if (maxSDF < -narrowBand) {
		// All far inside - interior tile
		info.nodeType = 1;
		info.tileValue = -3.0f; // Interior value
	}
	else {
		// Contains or near surface - leaf node
		info.nodeType = 2;
		info.voxelData = move(voxels);
	}

	return info;
}

void AModel::ExtractNarrowBand()
{
	const int GRID_SIZE = 64;
	const float WORLD_SIZE = 1.0f;  // -0.5 to +0.5
	const float VOXEL_SIZE = WORLD_SIZE / GRID_SIZE;

	vector<float> sdfData = LoadSDFTexture("res/sdfs/" + m_fileName + ".raw", { GRID_SIZE, GRID_SIZE, GRID_SIZE });

	auto sampleSDF = [&](int x, int y, int z) -> float {
		x = clamp(x, 0, GRID_SIZE - 1);
		y = clamp(y, 0, GRID_SIZE - 1);
		z = clamp(z, 0, GRID_SIZE - 1);
		return sdfData[z * GRID_SIZE * GRID_SIZE + y * GRID_SIZE + x];
	};

	// Define narrow band width
	float bandOffset = 10.0f * VOXEL_SIZE;  // Adjust this as needed
	float bandWidth = 1.0f * VOXEL_SIZE;  // Adjust this as needed

	for (int z = 0; z < GRID_SIZE - 1; z++) 
	{
		for (int y = 0; y < GRID_SIZE - 1; y++) 
		{
			for (int x = 0; x < GRID_SIZE - 1; x++) 
			{
				/*float centerSDF = sampleSDF(x, y, z);

				// Include voxel if it's within narrow band distance of surface
				if (abs(sdfValue) <= bandWidth) 
				{
					SurfaceVoxel voxel;
					voxel.x = static_cast<UINT>(x);
					voxel.y = static_cast<UINT>(y);
					voxel.z = static_cast<UINT>(z);
					voxel.sdfValue = sdfValue;
					m_surfaceVoxels.push_back(voxel);
				}*/


				// Sample 8 corners first
				float s000 = sampleSDF(x + 0, y + 0, z + 0);
				float s100 = sampleSDF(x + 1, y + 0, z + 0);
				float s010 = sampleSDF(x + 0, y + 1, z + 0);
				float s110 = sampleSDF(x + 1, y + 1, z + 0);
				float s001 = sampleSDF(x + 0, y + 0, z + 1);
				float s101 = sampleSDF(x + 1, y + 0, z + 1);
				float s011 = sampleSDF(x + 0, y + 1, z + 1);
				float s111 = sampleSDF(x + 1, y + 1, z + 1);

				// Check if surface crosses this voxel
				float minSDF = min({ s000, s100, s010, s110, s001, s101, s011, s111 });
				float maxSDF = max({ s000, s100, s010, s110, s001, s101, s011, s111 });

				if ((minSDF <= 0.0f && maxSDF >= 0.0f)) {
					SurfaceVoxel voxel;
					voxel.x = x; voxel.y = y; voxel.z = z;
					voxel.distance = s000;
					//voxel.instanceIndex = 0;
					//voxel.s000 = s000; voxel.s100 = s100; voxel.s010 = s010; voxel.s110 = s110;
					//voxel.s001 = s001; voxel.s101 = s101; voxel.s011 = s011; voxel.s111 = s111;

					m_surfaceVoxels.push_back(voxel);
				}

				//float sdfVals[8] = { s000, s100, s010, s110, s001, s101, s011, s111 };
				//bool anyInBand = false;

				//for (int i = 0; i < 8; ++i) 
				//{
				//	float v = sdfVals[i];
				//	if (v >= bandOffset - bandWidth * 0.5 && v <= bandOffset + bandWidth * 0.5) 
				//	{
				//		anyInBand = true;
				//		break;
				//	}
				//}

				//if (anyInBand) 
				//{
				//	SurfaceVoxel voxel;
				//	voxel.x = x; voxel.y = y; voxel.z = z;
				//	voxel.distance = 0;
				//	//voxel.s000 = s000; voxel.s100 = s100; voxel.s010 = s010; voxel.s110 = s110;
				//	//voxel.s001 = s001; voxel.s101 = s101; voxel.s011 = s011; voxel.s111 = s111;

				//	m_surfaceVoxels.push_back(voxel);
				//}
			}
		}
	}
}
inline uint32_t PackBrickCoord(uint32_t x, uint32_t y, uint32_t z)
{
	return (x & 0xFF) | ((y & 0xFF) << 8) | ((z & 0xFF) << 16);
}
void AModel::ExtractBricks()
{
	const int GRID_SIZE = m_sdfResolution; // Data points (63x63x63 voxels)
	const int BRICK_VOXEL_SIZE = 9;  // Spatial voxel coverage per brick
	const int BRICK_COUNT = (GRID_SIZE - 1 + BRICK_VOXEL_SIZE - 1) / BRICK_VOXEL_SIZE; // (voxelCount + (voxelPerBrickCount - 1)) / voxelPerBrickCount
	//const float PADDING_THRESHOLD = 8.0f / static_cast<float>(GRID_SIZE);
	const float PADDING_THRESHOLD = 0.1f;
	const float FRACTION_THRESHOLD = 0.2f;

	vector<float> sdfData = LoadSDFTexture("res/sdfs/" + m_fileName + ".raw", { m_sdfResolution, m_sdfResolution, m_sdfResolution });

	auto sampleSDF = [&](int x, int y, int z) -> float {
		x = clamp(x, 0, GRID_SIZE - 1);
		y = clamp(y, 0, GRID_SIZE - 1);
		z = clamp(z, 0, GRID_SIZE - 1);
		return sdfData[z * GRID_SIZE * GRID_SIZE + y * GRID_SIZE + x];
		};

	// Iterate through potential brick positions
	for (UINT bz = 0; bz < BRICK_COUNT; bz++) {
		for (UINT by = 0; by < BRICK_COUNT; by++) {
			for (UINT bx = 0; bx < BRICK_COUNT; bx++) {
				bool hasSurface = false;
				int nearSurfaceCount = 0;

				// Check if any voxel in this brick contains surface
				int voxelStartX = bx * BRICK_VOXEL_SIZE;
				int voxelStartY = by * BRICK_VOXEL_SIZE;
				int voxelStartZ = bz * BRICK_VOXEL_SIZE;

				for (int z = 0; z < BRICK_VOXEL_SIZE; z++) {
					for (int y = 0; y < BRICK_VOXEL_SIZE; y++) {
						for (int x = 0; x < BRICK_VOXEL_SIZE; x++) {
							int gx = voxelStartX + x;
							int gy = voxelStartY + y;
							int gz = voxelStartZ + z;

							if (gx >= GRID_SIZE - 1 || gy >= GRID_SIZE - 1 || gz >= GRID_SIZE - 1)
								continue;

							float minVal = FLT_MAX, maxVal = -FLT_MAX;
							for (int cz = 0; cz <= 1; cz++) {
								for (int cy = 0; cy <= 1; cy++) {
									for (int cx = 0; cx <= 1; cx++) {
										float val = sampleSDF(gx + cx, gy + cy, gz + cz);
										minVal = min(minVal, val);
										maxVal = max(maxVal, val);
									}
								}
							}

							if (minVal <= 0.0f && maxVal >= 0.0f) 
							{
								hasSurface = true;
							}
							if (minVal >= 0.0f && minVal < PADDING_THRESHOLD)
							{
								nearSurfaceCount++;
							}
						}
					}
				}

				bool keepBrick = hasSurface;
				if(!hasSurface && nearSurfaceCount > 0)
				{
					// Check if enough surface voxels are near the surface
					float fraction = static_cast<float>(nearSurfaceCount) / (BRICK_VOXEL_SIZE * BRICK_VOXEL_SIZE * BRICK_VOXEL_SIZE);
					if (fraction >= FRACTION_THRESHOLD) 
					{
						keepBrick = true;
					}
				}
				if (keepBrick) 
				{
					BrickMeta brickMeta;
					brickMeta.packedCoord = PackBrickCoord(bx, by, bz);
					brickMeta.sliceStart = static_cast<UINT>(m_brickMeta.size()) * (BRICK_VOXEL_SIZE + 1);

					m_brickMeta.push_back(brickMeta);
				}
			}
		}
	}
}

inline uint32_t morton3(int x, int y, int z)
{
	auto part = [](uint32_t v)
		{
			v = (v | (v << 16)) & 0x030000FF;
			v = (v | (v << 8)) & 0x0300F00F;
			v = (v | (v << 4)) & 0x030C30C3;
			v = (v | (v << 2)) & 0x09249249;
			return v;
		};
	return (part(z) << 2) | (part(y) << 1) | part(x);
}
inline XMINT3 decodeMorton3(uint32_t morton) {
	auto unpart = [](uint32_t v) {
		v &= 0x09249249;
		v = (v ^ (v >> 2)) & 0x030C30C3;
		v = (v ^ (v >> 4)) & 0x0300F00F;
		v = (v ^ (v >> 8)) & 0x030000FF;
		v = (v ^ (v >> 16)) & 0x000003FF;
		return v;
		};

	return {
		static_cast<int>(unpart(morton)),
		static_cast<int>(unpart(morton >> 1)),
		static_cast<int>(unpart(morton >> 2))
	};
}
template<typename ChildNodeT>
inline void BuildInternalLevel(
	std::vector<ChildNodeT>& srcNodes,  // Will be reordered!
	std::vector<InternalNode>& dstNodes)
{
	dstNodes.clear();

	// Step 1: Group children by parent coordinate
	std::map<uint32_t, std::vector<uint32_t>> parentGroups;

	for (uint32_t childIdx = 0; childIdx < srcNodes.size(); ++childIdx) {
		const XMINT3 c = srcNodes[childIdx].coord;
		XMINT3 p = { c.x >> 1, c.y >> 1, c.z >> 1 };  // Parent coord
		uint32_t parentKey = morton3(p.x, p.y, p.z);

		parentGroups[parentKey].push_back(childIdx);
	}

	// Step 2: Build new node array with contiguous children
	std::vector<ChildNodeT> reorderedNodes;
	reorderedNodes.reserve(srcNodes.size());

	for (auto& [parentKey, childIndices] : parentGroups) {
		// Sort children within this parent by spatial bit position
		sort(childIndices.begin(), childIndices.end(), [&](uint32_t a, uint32_t b) {
			const XMINT3 coordA = srcNodes[a].coord;
			const XMINT3 coordB = srcNodes[b].coord;
			int bitA = ((coordA.z & 1) << 2) | ((coordA.y & 1) << 1) | (coordA.x & 1);
			int bitB = ((coordB.z & 1) << 2) | ((coordB.y & 1) << 1) | (coordB.x & 1);
			return bitA < bitB;
			});

		// Create parent node
		InternalNode parent{};

		// Decode parent coordinates (you'll need this helper function)
		parent.coord = decodeMorton3(parentKey);
		parent.firstChild = static_cast<uint32_t>(reorderedNodes.size());
		parent.childMask = 0;

		// Add children contiguously and build mask
		for (uint32_t childIdx : childIndices) {
			reorderedNodes.push_back(srcNodes[childIdx]);

			// Set corresponding bit in mask
			const XMINT3 c = srcNodes[childIdx].coord;
			int bit = ((c.z & 1) << 2) | ((c.y & 1) << 1) | (c.x & 1);
			parent.childMask |= (1u << bit);
		}

		dstNodes.push_back(parent);
	}

	// Replace original with reordered
	srcNodes = std::move(reorderedNodes);
}
void AModel::ExtractBricksVDB()
{
	const int GRID_SIZE = 64;           // your volume size
	const int LEAF_SIZE = 8;            // NanoVDB leaf tile size (fixed)

	// your input dense sdfData[GRID_SIZE³]
	vector<float> sdfData = LoadSDFTexture("res/sdfs/" + m_fileName + ".raw", { GRID_SIZE, GRID_SIZE, GRID_SIZE });

	// Utility to sample dense grid safely
	auto sampleVoxel = [&](int x, int y, int z) -> float {
		x = std::clamp(x, 0, GRID_SIZE - 1);
		y = std::clamp(y, 0, GRID_SIZE - 1);
		z = std::clamp(z, 0, GRID_SIZE - 1);
		return sdfData[z * GRID_SIZE * GRID_SIZE + y * GRID_SIZE + x];
		};

	//std::vector<LeafTile> leafTiles;

	const int NUM_TILES = GRID_SIZE / LEAF_SIZE;

	const float BAND_WIDTH = 2.0 / GRID_SIZE;

	for (int tz = 0; tz < NUM_TILES; tz++) 
	{
		for (int ty = 0; ty < NUM_TILES; ty++) 
		{
			for (int tx = 0; tx < NUM_TILES; tx++) 
			{
				LeafNode leaf;
				leaf.coord = { tx, ty, tz };
				bool hasActiveVoxel = false;
				memset(leaf.bitmask, 0, sizeof(leaf.bitmask));

				// Check all voxels in this tile
				for (int z = 0; z < LEAF_SIZE; z++) 
				{
					for (int y = 0; y < LEAF_SIZE; y++) 
					{
						for (int x = 0; x < LEAF_SIZE; x++) 
						{
							int localX = tx * LEAF_SIZE + x;
							int localY = ty * LEAF_SIZE + y;
							int localZ = tz * LEAF_SIZE + z;

							float val = sampleVoxel(localX, localY, localZ);
							int linearIndex = z * LEAF_SIZE * LEAF_SIZE + y * LEAF_SIZE + x;
							//leaf.voxels[linearIndex] = val;
							leaf.voxels[x][y][z] = val;

							// Determine if voxel is active (e.g. narrow band)
							if (abs(val) < BAND_WIDTH)
							{
								hasActiveVoxel = true;

								// Find which uint in bitmask and bit offset
								int uintIndex = linearIndex / 32;
								int bitOffset = linearIndex % 32;

								leaf.bitmask[uintIndex] |= (1u << bitOffset);
							}
						}
					}
				}

				if (hasActiveVoxel) 
				{
					m_leafLevel.push_back(leaf);
				}
			}
		}
	}

	//vector<InternalNode> level1;
	BuildInternalLevel(m_leafLevel, m_internalLevel1);

	//vector<InternalNode> level2;
	BuildInternalLevel(m_internalLevel1, m_internalLevel2);

	vector<InternalNode> rootLevel;
	BuildInternalLevel(m_internalLevel2, rootLevel);

	assert(rootLevel.size() == 1 && "Expected exactly one root node");
	//InternalNode root = rootLevel[0]; // only one root
	m_rootNode = rootLevel[0];
}

inline void ProcessNode(const aiNode* node, const aiScene* scene, vector<Vertex>& vertices, vector<Index>& indices)
{
	for (UINT i = 0; i < node->mNumMeshes; i++)
	{
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		ProcessMesh(mesh, scene, vertices, indices);
	}

	for (UINT i = 0; i < node->mNumChildren; i++)
	{
		ProcessNode(node->mChildren[i], scene, vertices, indices);
	}
}
inline void ProcessMesh(const aiMesh* mesh, const aiScene* scene, vector<Vertex>& vertices, vector<UINT>& indices)
{
	// Extract vertices
	for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
		// Position
		float x = mesh->mVertices[i].x;
		float y = mesh->mVertices[i].y;
		float z = mesh->mVertices[i].z;

		float nx = 0, ny = 0, nz = 0;

		// Normal (if available)
		if (mesh->HasNormals()) {
			nx = mesh->mNormals[i].x;
			ny = mesh->mNormals[i].y;
			nz = mesh->mNormals[i].z;
		}

		////// Texture coordinates (if available)
		//if (mesh->HasTextureCoords(0)) {
		//	float u = mesh->mTextureCoords[0][i].x;
		//	float v = mesh->mTextureCoords[0][i].y;
		//}

		vertices.push_back({ { x, y, z }, { nx, ny, nz } });
	}

	// Extract indices
	for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
		aiFace face = mesh->mFaces[i];
		// Should be 3 indices after triangulation
		for (unsigned int j = 0; j < face.mNumIndices; j++) {
			unsigned int index = face.mIndices[j];
			indices.push_back(index);
		}
	}
}
inline vector<float> LoadSDFTexture(const string& filename, const XMUINT3& resolution)
{
	vector<float> data(resolution.x * resolution.y * resolution.z);

	ifstream file(filename, ios::binary);
	if (!file) {
		throw runtime_error("Cannot open SDF file: " + filename);
	}

	file.read(reinterpret_cast<char*>(data.data()), data.size() * sizeof(float));
	file.close();

	return data;
}
