#include "stdafx.h"
#include "AModel.h"

using namespace std;

inline void ProcessNode(const aiNode* node, const aiScene* scene, vector<Vertex>& vertices, vector<Index>& indices);
inline void ProcessMesh(const aiMesh* mesh, const aiScene* scene, vector<Vertex>& vertices, vector<UINT>& indices);
inline vector<float> LoadSDFTexture(const string& filename, const XMUINT3& resolution);

void AModel::BuildSDF(ID3D12Device* device, ID3D12GraphicsCommandList* commandList)
{
	const XMUINT3 resolution = { 64, 64, 64 }; //TODO: this is temporary
	std::vector<float> sdfData = LoadSDFTexture("res/sdfs/" + m_fileName + ".raw", resolution);

	const UINT textureDataSize = static_cast<UINT>(sdfData.size() * sizeof(float));

	AllocateTexture(device, resolution, &m_sdfBuffer.resource);
	AllocateBuffer(
		device,
		D3D12_HEAP_TYPE_UPLOAD,
		textureDataSize,
		&m_stagingSdfBuffer,
		D3D12_RESOURCE_STATE_GENERIC_READ,
		D3D12_RESOURCE_FLAG_NONE,
		sdfData.data());

	D3D12_SUBRESOURCE_DATA textureSubresource = {};
	textureSubresource.pData = sdfData.data();
	textureSubresource.RowPitch = resolution.z * sizeof(float);  // Bytes per row
	textureSubresource.SlicePitch = resolution.x * resolution.y * sizeof(float);  // Bytes per Z-slice

	UpdateSubresources(commandList, m_sdfBuffer.resource.Get(), m_stagingSdfBuffer.Get(), 0, 0, 1, &textureSubresource);


	auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
		m_sdfBuffer.resource.Get(),
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
		throw std::logic_error(importer.GetErrorString());
	}

	ProcessNode(scene->mRootNode, scene, vertices, indices);

	m_vertexCount = vertices.size();
    m_indexCount = indices.size();
}

void AModel::BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs)
{
	// only one hardcoded AABB for now
	m_aabbCount = 1;

	aabbs.push_back(
		{ -0.6f, -0.6f, -0.6f, 0.6f,  0.6f,  0.6f }
	);
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
