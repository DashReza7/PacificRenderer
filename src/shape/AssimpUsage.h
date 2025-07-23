#include <iostream>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


void processMesh(aiMesh* mesh, const aiScene* scene) {
	std::cout << "mesh name: " << mesh->mName.C_Str() << std::endl;
	std::cout << "num vertices: " << mesh->mNumVertices << std::endl;
	for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
		aiVector3D pos = mesh->mVertices[i];
		aiVector3D normal = mesh->HasNormals() ? mesh->mNormals[i] : aiVector3D(0, 0, 0);
		aiVector3D texCoord = mesh->HasTextureCoords(0) ? mesh->mTextureCoords[0][i] : aiVector3D(0, 0, 0);

		// Store or process the position, normal, and texCoord
	}

	// Process indices
	std::cout << "num faces: " << mesh->mNumFaces << std::endl;
	for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
		aiFace face = mesh->mFaces[i];
		for (unsigned int j = 0; j < face.mNumIndices; j++) {
			unsigned int index = face.mIndices[j];
			// Store or use the index
		}
	}
}

void processNode(aiNode* node, const aiScene* scene) {
	// Process all the node's meshes (if any)
	for (unsigned int i = 0; i < node->mNumMeshes; i++) {
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		processMesh(mesh, scene);
	}
	// Recursively process each child node
	for (unsigned int i = 0; i < node->mNumChildren; i++) {
		processNode(node->mChildren[i], scene);
	}
}

void usage_example()
{
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile("C:/Users/masou/Desktop/Codes/Graphics/MitsubaStuff/scenes/meshes/sphere.obj", aiProcess_Triangulate);

	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
		std::cerr << "Assimp error: " << importer.GetErrorString() << std::endl;
		return;
	}

	processNode(scene->mRootNode, scene);
}