#include "stdafx.h"
#include "AModel.h"

using namespace std;

inline void ProcessNode(const aiNode* node, const aiScene* scene, vector<Vertex>& vertices, vector<Index>& indices);
inline void ProcessMesh(const aiMesh* mesh, const aiScene* scene, vector<Vertex>& vertices, vector<UINT>& indices);

void AModel::BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices)
{
	// Create importer
	Assimp::Importer importer;

	// Load file with post-processing flags
	const aiScene* scene = importer.ReadFile("res/models/monkey.obj",
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
		{ -0.5f, -0.5f, -0.5f, 0.5f,  0.5f,  0.5f }
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
