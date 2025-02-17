#include "EnginePch.h"
#include "ResourceMgr.h"

#include "FileIO.h"
#include "Texture.h"
#include "Mesh.h"
#include "Model.h"
#include "Shader.h"
#include "AnimationClip.h"
#include "AnimatorController.h"
#include "Component/ParticleSystem.h"

void ResourceMgr::LoadResources()
{
	LoadTextures();
	LoadRectangleMesh();
	LoadPointMesh();
	LoadModels();
	LoadShaders();
	LoadAnimationClips();
	LoadAnimatorControllers();
}

void ResourceMgr::Clear()
{
	for (auto& resource : mResources) {
		resource.clear();
	}
}

sptr<Texture> ResourceMgr::CreateTexture(const std::string& name, UINT width, UINT height, DXGI_FORMAT dxgiFormat, D3D12_RESOURCE_FLAGS resourcecFlags, D3D12_RESOURCE_STATES resourceStates, Vec4 clearColor)
{
	sptr<Texture> texture = std::make_shared<Texture>();
	texture->Create(width, height, dxgiFormat, resourcecFlags, resourceStates, clearColor);
	Add<Texture>(name, texture);
	return texture;
}

sptr<Texture> ResourceMgr::CreateTexture(const std::string& name, ComPtr<ID3D12Resource> resource)
{
	sptr<Texture> texture = std::make_shared<Texture>();
	texture->Create(resource);
	Add<Texture>(name, texture);
	return texture;
}

sptr<ModelObjectMesh> ResourceMgr::LoadRectangleMesh()
{
	sptr<ModelObjectMesh> findMesh = Get<ModelObjectMesh>("Rect");
	if (findMesh)
		return findMesh;

	sptr<ModelObjectMesh> mesh = std::make_shared<ModelObjectMesh>();
	mesh->CreateRectangleMesh();
	Add<ModelObjectMesh>("Rect", mesh);
	return mesh;
}

sptr<ModelObjectMesh> ResourceMgr::LoadPointMesh()
{
	sptr<ModelObjectMesh> findMesh = Get<ModelObjectMesh>("Point");
	if (findMesh)
		return findMesh;

	sptr<ModelObjectMesh> mesh = std::make_shared<ModelObjectMesh>();
	mesh->CreatePointMesh();
	Add<ModelObjectMesh>("Point", mesh);
	return mesh;
}

void ResourceMgr::LoadTextures()
{
	std::cout << "Load textures...\n";

	FileIO::ModelIO::LoadTextures("Import/Textures/");
	FileIO::ModelIO::LoadTextures("Import/UI/");
	FileIO::ModelIO::LoadTextures("Import/Skybox/", D3DResource::TextureCube);
}

void ResourceMgr::LoadModels()
{
	std::cout << "Load models...\n";

	const std::string rootFolder = "Import/Meshes/";

	sptr<MasterModel> model;
	for (const auto& modelFile : std::filesystem::directory_iterator(rootFolder)) {
		const std::string fileName = modelFile.path().filename().string();
		const std::string modelName = FileIO::RemoveExtension(fileName);

		model = FileIO::ModelIO::LoadGeometryFromFile(rootFolder + fileName);
		if (fileName.substr(0, 6) == "sprite") {
			model->SetSprite();
		}
		ResourceMgr::I->Add(modelName, model);
	}
}

void ResourceMgr::LoadShaders()
{
	std::cout << "Load shaders...\n";

// DeferredShader
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Global
	{
		ShaderInfo info = {
			ShaderType::Deferred,
		};

		ShaderPath path = {
			 "VShader_Standard.cso",
			 "PShader_Deferred.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Global", shader);
	}
	{
		ShaderInfo info = {
			ShaderType::Shadow,
			RasterizerType::DepthBias,
		};

		ShaderPath path = {
			 "VShader_Standard.cso",
			 "PShader_Shadow.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Shadow_Global", shader);
	}
#pragma endregion
#pragma region ObjectInst
	{
		ShaderInfo info = {
			ShaderType::Deferred,
		};

		ShaderPath path = {
			 "VShader_StandardInstance.cso",
			 "PShader_Deferred.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("ObjectInst", shader);
	}
	{
		ShaderInfo info = {
			ShaderType::Shadow,
			RasterizerType::DepthBias,
		};

		ShaderPath path = {
			 "VShader_StandardInstance.cso",
			 "PShader_Shadow.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Shadow_ObjectInst", shader);
	}
	{
		ShaderInfo info = {
			ShaderType::LDR,
			RasterizerType::Cull_Back,
			DepthStencilType::Less,
			BlendType::Alpha_Blend,
			InputLayoutType::Default,
		};

		ShaderPath path = {
			 "VShader_Voxel.cso",
			 "PShader_Voxel.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Voxel", shader);
	}
#pragma endregion
#pragma region SkinMesh
	{
		ShaderInfo info = {
			ShaderType::Deferred,
		};

		ShaderPath path = {
			 "VShader_SkinnedMesh.cso",
			 "PShader_Deferred.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("SkinMesh", shader);
	}
	{
		ShaderInfo info = {
			ShaderType::Shadow,
			RasterizerType::DepthBias,
		};

		ShaderPath path = {
			 "VShader_SkinnedMesh.cso",
			 "PShader_SkinnedMesh_Shadow.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Shadow_SkinMesh", shader);
	}
	{
		ShaderInfo info = {
			ShaderType::Shadow,
			RasterizerType::DepthBias,
			DepthStencilType::Greater,
		};

		ShaderPath path = {
			 "VShader_SkinnedMesh.cso",
			 "PShader_CustomDepth.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("CustomDepth_SkinMesh", shader);
	}
	{
		ShaderInfo info = {
			ShaderType::HDR,
			RasterizerType::Cull_Back,
			DepthStencilType::Less_No_Write,
			BlendType::Alpha_Blend,
		};

		ShaderPath path = {
			 "VShader_SkinnedMesh.cso",
			 "PShader_Forward.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Dissolve", shader);
	}
#pragma endregion
#pragma region Terrain
	{
		ShaderInfo info = {
			ShaderType::Deferred,
		};

		ShaderPath path = {
			 "VShader_Terrain.cso",
			 "PShader_Terrain.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Terrain", shader);
	}
#pragma endregion

// ForwardShader
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Water
	{
		ShaderInfo info = {
			ShaderType::HDR,
			RasterizerType::Cull_Back,
			DepthStencilType::Less_No_Write,
			BlendType::Alpha_Blend,
		};

		ShaderPath path = {
			 "VShader_Water.cso",
			 "PShader_Water.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Water", shader);
	}
#pragma endregion
#pragma region Billboard
	{
		ShaderInfo info = {
			ShaderType::HDR,
			RasterizerType::Cull_Back,
			DepthStencilType::Less,
			BlendType::Alpha_Blend,
		};

		ShaderPath path = {
			 "VShader_Billboard.cso",
			 "PShader_Billboard.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Billboard", shader);
	}
#pragma endregion
#pragma region Sprite
	{
		ShaderInfo info = {
			ShaderType::HDR,
			RasterizerType::Cull_Back,
			DepthStencilType::Less,
			BlendType::Alpha_Blend,
		};

		ShaderPath path = {
			 "VShader_Sprite.cso",
			 "PShader_Billboard.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Sprite", shader);
	}
#pragma endregion
#pragma region Final
	{
		ShaderInfo info = {
			ShaderType::HDR,
			RasterizerType::Cull_Back,
			DepthStencilType::No_DepthTest_No_Write,
		};

		ShaderPath path = {
			 "VShader_Tex.cso",
			 "PShader_Final.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Final", shader);
	}
#pragma endregion
#pragma region Canvas
	{
		ShaderInfo info = {
			ShaderType::LDR,
			RasterizerType::Cull_Back,
			DepthStencilType::No_DepthTest_No_Write,
			BlendType::Alpha_Blend,
		};

		ShaderPath path = {
			 "VShader_Canvas.cso",
			 "PShader_Canvas.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Canvas", shader);
	}
#pragma endregion
#pragma region Wire
	{
		ShaderInfo info = {
			ShaderType::LDR,
			RasterizerType::WireFrame,
			DepthStencilType::No_DepthTest,
			BlendType::Default,
			InputLayoutType::Wire,
			D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_LINELIST
		};

		ShaderPath path = {
			 "VShader_Wired.cso",
			 "PShader_Wired.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Wire", shader);
	}
#pragma endregion
#pragma region OffScreen
	{
		ShaderInfo info = {
			ShaderType::LDR,
			RasterizerType::Cull_Back,
			DepthStencilType::No_DepthTest_No_Write,
		};

		ShaderPath path = {
			 "VShader_Tex.cso",
			 "PShader_OffScreen.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("OffScreen", shader);
	}
#pragma endregion
#pragma region Lighting
	{
		ShaderInfo info = {
			ShaderType::Lighting,
			RasterizerType::Cull_Back,
			DepthStencilType::No_DepthTest_No_Write,
		};

		ShaderPath path = {
			 "VShader_Tex.cso",
			 "PShader_DirLighting.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("DirLighting", shader);
	}

	{
		ShaderInfo info = {
			ShaderType::Lighting,
			RasterizerType::Cull_Back,
			DepthStencilType::No_DepthTest_No_Write,
		};

		ShaderPath path = {
			 "VShader_SpotPointLighting.cso",
			 "PShader_SpotPointLighting.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("SpotPointLighting", shader);
	}
#pragma endregion
#pragma region Transparent
	{
		ShaderInfo info = {
			ShaderType::HDR,
			RasterizerType::Cull_Back,
			DepthStencilType::Less,
			BlendType::Alpha_Blend,
		};

		ShaderPath path = {
			 "VShader_Standard.cso",
			 "PShader_Forward.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Transparent", shader);
	}
#pragma endregion
#pragma region ShieldAbility
	{
		ShaderInfo info = {
			ShaderType::HDR,
			RasterizerType::Cull_None,
			DepthStencilType::Less_No_Write,
			BlendType::Alpha_Blend,
		};

		ShaderPath path = {
			 "VShader_Standard.cso",
			 "PShader_ShieldAbility.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("ShieldAbility", shader);
	}
#pragma endregion
#pragma region IRDetectorAbility
	{
		ShaderInfo info = {
			ShaderType::HDR,
			RasterizerType::Cull_Back,
			DepthStencilType::No_DepthTest_No_Write,
		};

		ShaderPath path = {
			 "VShader_Tex.cso",
			 "PShader_IRDetectorAbility.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("IRDetectorAbility", shader);
	}
#pragma endregion
#pragma region MinimapAbility
	{
		ShaderInfo info = {
			ShaderType::LDR,
			RasterizerType::Cull_Back,
			DepthStencilType::No_DepthTest_No_Write,
			BlendType::Alpha_Blend,
		};

		ShaderPath path = {
			 "VShader_Canvas.cso",
			 "PShader_MinimapAbility.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("MinimapAbility", shader);
	}
#pragma endregion
#pragma region SkyBox
	{
		ShaderInfo info = {
			ShaderType::HDR,
			RasterizerType::Cull_None,
			DepthStencilType::Less_Equal,
		};

		ShaderPath path = {
			 "VShader_Skybox.cso",
			 "PShader_Skybox.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("SkyBox", shader);
	}
#pragma endregion
#pragma region SSAO
	{
		ShaderInfo info = {
			ShaderType::Ssao,
			RasterizerType::Cull_None,
			DepthStencilType::No_DepthTest_No_Write,
		};

		ShaderPath path = {
			 "VShader_Ssao.cso",
			 "PShader_Ssao.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("Ssao", shader);
	}
#pragma endregion
#pragma region SSAOBlur
	{
		ShaderInfo info = {
			ShaderType::Ssao,
			RasterizerType::Cull_None,
			DepthStencilType::No_DepthTest_No_Write,
		};

		ShaderPath path = {
			 "VShader_Ssao.cso",
			 "PShader_SsaoBlur.cso",
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("SsaoBlur", shader);
	}
#pragma endregion
#pragma region GraphicsParticle
	{
		ShaderInfo info{
			ShaderType::HDR,
			RasterizerType::Cull_None,
			DepthStencilType::Less_No_Write,
			BlendType::Alpha_Blend,
			InputLayoutType::Default,
			D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_POINTLIST,
		};

		ShaderPath path = {
			 "VShader_Particle.cso",
			 "PShader_Particle.cso",
		};

		{
			path.GS = "GShader_Particle.cso";
			info.BlendType = BlendType::Alpha_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("GraphicsParticle", shader);
		}
		{
			path.GS = "GShader_StretchedParticle.cso";
			info.BlendType = BlendType::Alpha_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("GraphicsStretchedParticle", shader);
		}
		{
			path.GS = "GShader_Particle.cso";
			info.BlendType = BlendType::One_To_One_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("OneToOneBlend_GraphicsParticle", shader);
		}
		{
			path.GS = "GShader_StretchedParticle.cso";
			info.BlendType = BlendType::One_To_One_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("OneToOneBlend_GraphicsStretchedParticle", shader);
		}
		{
			path.GS = "GShader_Particle.cso";
			info.BlendType = BlendType::Additive_Soft_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("AdditiveSoft_GraphicsParticle", shader);
		}
		{
			path.GS = "GShader_StretchedParticle.cso";
			info.BlendType = BlendType::Additive_Soft_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("AdditiveSoft_GraphicsStretchedParticle", shader);
		}
		{
			path.GS = "GShader_Particle.cso";
			info.BlendType = BlendType::Multiply_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("MultiplyBlend_GraphicsParticle", shader);
		}
		{
			path.GS = "GShader_StretchedParticle.cso";
			info.BlendType = BlendType::Multiply_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("MultiplyBlend_GraphicsStretchedParticle", shader);
		}

		{
			path.GS = "GShader_Particle.cso";
			path.PS = "PShader_ScrollAlphaMask_Particle.cso";
			info.BlendType = BlendType::One_To_One_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("OneToOneBlend_GraphicsScrollAlphaMaskParticle", shader);
		}

		{
			path.GS = "GShader_Particle.cso";
			path.PS = "PShader_ScrollAlphaMask_Particle.cso";
			info.BlendType = BlendType::Multiply_Inv_Blend;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("MultiplyBlend_GraphicsScrollAlphaMaskParticle", shader);
		}

		{
			path.GS = "GShader_Particle.cso";
			path.PS = "PShader_Scroll_Smoke.cso";
			info.BlendType = BlendType::Scroll_Smoke;
			sptr<Shader> shader = std::make_shared<Shader>();
			shader->Load(info, path);
			Add<Shader>("Scroll_Smoke", shader);
		}
	}
#pragma endregion
// ComputeShader
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region HorzBlur
	{
		ShaderInfo info{
			ShaderType::Compute,
		};

		ShaderPath path = {
			 "",
			 "",
			 "",
			 "CShader_HorzBlur.cso"
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("HorzBlur", shader);
	}
#pragma endregion
#pragma region VertBlur
	{
		ShaderInfo info{
			ShaderType::Compute,
		};

		ShaderPath path = {
			 "",
			 "",
			 "",
			 "CShader_VertBlur.cso"
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("VertBlur", shader);
	}
#pragma endregion
#pragma region VertBlur
	{
		ShaderInfo info{
			ShaderType::Compute,
		};

		ShaderPath path = {
			 "",
			 "",
			 "",
			 "CShader_LUT.cso"
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("LUT", shader);
	}
#pragma endregion
#pragma region ComputeParticle
	{
		ShaderInfo info{
			ShaderType::Particle,
		};

		ShaderPath path = {
			 "",
			 "",
			 "",
			 "CShader_Particle.cso"
		};

		sptr<Shader> shader = std::make_shared<Shader>();
		shader->Load(info, path);
		Add<Shader>("ComputeParticle", shader);
	}
#pragma endregion
}

void ResourceMgr::LoadAnimationClips()
{
	std::cout << "Load animation clips...\n";

	const std::string rootFolder = "Import/AnimationClips/";
	for (const auto& clipFolder : std::filesystem::directory_iterator(rootFolder)) {
		std::string clipFolderName = clipFolder.path().filename().string();

		for (const auto& file : std::filesystem::directory_iterator(rootFolder + clipFolderName + '/')) {
			std::string fileName = file.path().filename().string();
			sptr<AnimationClip> clip = FileIO::AnimationIO::LoadAnimationClip(clipFolder.path().string() + '/' + fileName);

			FileIO::RemoveExtension(fileName);
			const std::string clipName = clipFolderName + '/' + fileName;
			ResourceMgr::I->Add<AnimationClip>(clipName, clip);
		}
	}
}

void ResourceMgr::LoadAnimatorControllers()
{
	const std::string rootFolder = "Import/AnimatorControllers/";
	for (const auto& file : std::filesystem::directory_iterator(rootFolder)) {
		const std::string fileName = file.path().filename().string();
		ResourceMgr::I->Add<AnimatorController>(FileIO::RemoveExtension(fileName), FileIO::AnimationIO::LoadAnimatorController(rootFolder + fileName));
	}
}
