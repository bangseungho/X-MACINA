#pragma region Include
#include "EnginePch.h"
#include "Scene.h"
#include "DXGIMgr.h"
#include "MultipleRenderTarget.h"
#include "FrameResource.h"

#include "ResourceMgr.h"
#include "Object.h"
#include "Model.h"
#include "Terrain.h"
#include "Shader.h"
#include "MeshRenderer.h"
#include "Timer.h"
#include "FileIO.h"
#include "Light.h"
#include "SkyBox.h"
#include "Texture.h"
#include "ObjectPool.h"
#include "Component/UI.h"
#include "Component/Camera.h"
#include "Component/Collider.h"
#include "Component/Component.h"
#include "Component/ParticleSystem.h"
#include "Component/Agent.h"
#include "AbilityMgr.h"

#include "TestCube.h"
#include "Ssao.h"
#include "VoxelManager.h"
#pragma endregion




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region C/Dtor
namespace {
	constexpr int kGridWidthCount = 20;						// all grid count = n*n
	constexpr Vec3 kBorderPos = Vec3(256, 200, 256);		// center of border
	constexpr Vec3 kBorderExtents = Vec3(1500, 500, 1500);	// extents of border
	constexpr int kGridWidth = static_cast<int>(kBorderExtents.x / kGridWidthCount);		// length of x for one grid
	static int kGridCols{};			// number of columns in the grid
}

Scene::Scene()
	:
	mMapBorder(kBorderPos, kBorderExtents),
	mLight(std::make_shared<Light>())
{

}

void Scene::Release()
{
	FRAME_RESOURCE_MGR->WaitForGpuComplete();

	ReleaseObjects();

	ProcessAllObjects([](sptr<Object> object) {
		object->Destroy();
		});

	mGameManager->OnDestroy();
}
#pragma endregion





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Getter
float Scene::GetTerrainHeight(float x, float z) const
{
	assert(mTerrain);

	return mTerrain->GetHeight(x, z);
}

std::vector<sptr<GameObject>> Scene::GetAllObjects() const
{
	std::vector<sptr<GameObject>> result;
	result.reserve(mEnvironments.size() + mStaticObjects.size() + mDynamicObjects.size());
	result.insert(result.end(), mEnvironments.begin(), mEnvironments.end());
	result.insert(result.end(), mStaticObjects.begin(), mStaticObjects.end());
	result.insert(result.end(), mDynamicObjects.begin(), mDynamicObjects.end());

	return result;
}

#pragma endregion





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region DirectX
void Scene::ReleaseUploadBuffers()
{
	MeshRenderer::ReleaseUploadBuffers();
}

void Scene::UpdateShaderVars()
{
	UpdateMainPassCB();
	UpdateShadowPassCB();
	UpdateSsaoCB();
	UpdateMaterialBuffer();
}

void Scene::UpdateMainPassCB()
{
	Matrix proj = MAIN_CAMERA->GetProjMtx();
	PassConstants passCB;
	passCB.MtxView = MAIN_CAMERA->GetViewMtx().Transpose();
	passCB.MtxProj = MAIN_CAMERA->GetProjMtx().Transpose();
	auto determinantProj = XMMatrixDeterminant(proj);
	passCB.MtxInvProj = XMMatrixInverse(&determinantProj, proj);
	passCB.MtxShadow = mLight->GetShadowMtx().Transpose();
	passCB.MtxNoLagView = MAIN_CAMERA->GetNoLagViewtx().Transpose();
	passCB.CameraPos = MAIN_CAMERA->GetPosition();
	passCB.CameraRight = MAIN_CAMERA->GetRight();
	passCB.CameraUp = MAIN_CAMERA->GetUp();
	passCB.DeltaTime = DeltaTime();
	passCB.TotalTime = Timer::I->GetTotalTime();
	passCB.FrameBufferWidth = DXGIMgr::I->GetWindowWidth();
	passCB.FrameBufferHeight = DXGIMgr::I->GetWindowHeight();
	passCB.SkyBoxIndex = mSkyBox->GetTexture()->GetSrvIdx();
	passCB.DefaultDsIndex = RESOURCE<Texture>("DefaultDepthStencil")->GetSrvIdx();
	passCB.ShadowDsIndex = RESOURCE<Texture>("ShadowDepthStencil")->GetSrvIdx();
	passCB.CustomDsIndex = RESOURCE<Texture>("CustomDepthStencil")->GetSrvIdx();
	passCB.RT0G_PositionIndex = RESOURCE<Texture>("PositionTarget")->GetSrvIdx();
	passCB.RT1G_NormalIndex = RESOURCE<Texture>("NormalTarget")->GetSrvIdx();
	passCB.RT2G_DiffuseIndex = RESOURCE<Texture>("DiffuseTarget")->GetSrvIdx();
	passCB.RT3G_EmissiveIndex = RESOURCE<Texture>("EmissiveTarget")->GetSrvIdx();
	passCB.RT4G_MetallicSmoothnessIndex = RESOURCE<Texture>("MetallicSmoothnessTarget")->GetSrvIdx();
	passCB.RT5G_OcclusionIndex = RESOURCE<Texture>("OcclusionTarget")->GetSrvIdx();
	passCB.RT0L_DiffuseIndex = RESOURCE<Texture>("DiffuseAlbedoTarget")->GetSrvIdx();
	passCB.RT1L_SpecularIndex = RESOURCE<Texture>("SpecularAlbedoTarget")->GetSrvIdx();
	passCB.RT2L_AmbientIndex = RESOURCE<Texture>("AmbientTarget")->GetSrvIdx();
	passCB.RT0S_SsaoIndex = RESOURCE<Texture>("SSAOTarget_0")->GetSrvIdx();
	passCB.RT0O_OffScreenIndex = RESOURCE<Texture>("OffScreenTarget")->GetSrvIdx();
	passCB.LiveObjectDissolveIndex = RESOURCE<Texture>("LiveObjectDissolve")->GetSrvIdx();
	passCB.BuildingDissolveIndex = RESOURCE<Texture>("Dissolve_01_05")->GetSrvIdx();
	passCB.LightCount = mLight->GetLightCount();
	passCB.GlobalAmbient = Vec4(0.4f, 0.4f, 0.4f, 1.f);
	passCB.FilterOption = DXGIMgr::I->GetFilterOption();
	passCB.ShadowIntensity = 0.0f;
	passCB.FogColor = Colors::Gray;
	memcpy(&passCB.Lights, mLight->GetSceneLights().get()->Lights.data(), sizeof(passCB.Lights));

	FRAME_RESOURCE_MGR->CopyData(0, passCB);
}

void Scene::UpdateShadowPassCB()
{
	PassConstants passCB;
	passCB.MtxView = mLight->GetLightViewMtx().Transpose();
	passCB.MtxProj = mLight->GetLightProjMtx().Transpose();
	passCB.LiveObjectDissolveIndex = RESOURCE<Texture>("LiveObjectDissolve")->GetSrvIdx();

	FRAME_RESOURCE_MGR->CopyData(1, passCB);
}

void Scene::UpdateSsaoCB()
{
	SsaoConstants ssaoCB;

	Matrix mtxProj = MAIN_CAMERA->GetProjMtx();
	Matrix mtxTex = {
		0.5f, 0.0f, 0.0f, 0.0f,
		0.0f, -0.5f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.5f, 0.5f, 0.0f, 1.0f };

	ssaoCB.MtxInvProj = MAIN_CAMERA->GetProjMtx().Invert().Transpose();
	ssaoCB.MtxProjTex = (mtxProj * mtxTex).Transpose();
	DXGIMgr::I->GetSsao()->GetOffsetVectors(ssaoCB.OffsetVectors);

	// for Blur 
	auto blurWeights = Filter::CalcGaussWeights(2.5f);
	ssaoCB.BlurWeights[0] = Vec4(&blurWeights[0]);
	ssaoCB.BlurWeights[1] = Vec4(&blurWeights[4]);
	ssaoCB.BlurWeights[2] = Vec4(&blurWeights[8]);

	auto ssaoTarget = RESOURCE<Texture>("SSAOTarget_0");
	ssaoCB.InvRenderTargetSize = Vec2{ 1.f / ssaoTarget->GetWidth(), 1.f / ssaoTarget->GetHeight() };

	// coordinates given in view space.
	ssaoCB.OcclusionRadius = 0.5f;
	ssaoCB.OcclusionFadeStart = 0.2f;
	ssaoCB.OcclusionFadeEnd = 1.0f;
	ssaoCB.SurfaceEpsilon = 0.05f;
	ssaoCB.AccessContrast = 12;

	ssaoCB.RandomVectorIndex = RESOURCE<Texture>("RandomVector")->GetSrvIdx();

	FRAME_RESOURCE_MGR->CopyData(ssaoCB);
}

void Scene::UpdateAbilityCB(int& idx, const AbilityConstants& value)
{
	FRAME_RESOURCE_MGR->CopyData(idx, value);
}

void Scene::SetAbilityCB(int idx) const
{
	CMD_LIST->SetGraphicsRootConstantBufferView(DXGIMgr::I->GetGraphicsRootParamIndex(RootParam::Ability), FRAME_RESOURCE_MGR->GetAbilityCBGpuAddr(idx));
}

void Scene::SetPlayer(Object* player)
{
	mPlayer = player;
}

void Scene::UpdateMaterialBuffer()
{
	ResourceMgr::I->ProcessFunc<MasterModel>(
		[](sptr<MasterModel> model) {
			model->GetMesh()->UpdateMaterialBuffer();
		});
}
#pragma endregion





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Build
void Scene::BuildObjects()
{
	// load canvas (UI)
	Canvas::I->Init();

	// load models
	LoadSceneObjects("Import/Scene.bin");
	mGameManager   = std::make_shared<Object>();

	// build settings
	BuildTerrain();

	// build static meshes
	MeshRenderer::BuildMeshes();

	// skybox
	mSkyBox = std::make_shared<SkyBox>();

	VoxelManager::I->Init();
}

void Scene::ReleaseObjects()
{
	MeshRenderer::Release();
}

void Scene::BuildTerrain()
{
	mTerrain = std::make_shared<Terrain>("Import/Terrain.bin");

	BuildGrid();
}

void Scene::BuildGrid()
{
	constexpr float kMaxHeight = 300.f;	// for 3D grid

	// recalculate scene grid size
	const int adjusted = Math::GetNearestMultiple((int)mMapBorder.Extents.x, kGridWidth);
	mMapBorder.Extents = Vec3((float)adjusted, mMapBorder.Extents.y, (float)adjusted);

	// set grid start pos
	mGridStartPoint = mMapBorder.Center.x - mMapBorder.Extents.x / 2;

	// set grid count
	kGridCols = adjusted / kGridWidth;
	const int gridCount = kGridCols * kGridCols;
	mGrids.resize(gridCount);

	// set grid bounds
	const float gridExtent = (float)kGridWidth / 2.0f;
	for (int y = 0; y < kGridCols; ++y) {
		for (int x = 0; x < kGridCols; ++x) {
			float gridX = (kGridWidth * x) + ((float)kGridWidth / 2) + mGridStartPoint;
			float gridZ = (kGridWidth * y) + ((float)kGridWidth / 2) + mGridStartPoint;

			BoundingBox bb{};
			bb.Center = Vec3(gridX, kMaxHeight, gridZ);
			bb.Extents = Vec3(gridExtent, kMaxHeight, gridExtent);

			int index = (y * kGridCols) + x;
			mGrids[index] = std::make_shared<Grid>(index, kGridWidth, bb);
		}
	}
}

void Scene::UpdateGridInfo()
{
	ProcessActiveObjects([this](sptr<GridObject> object) {
		UpdateObjectGrid(object.get());
		});

	mTerrain->UpdateGrid();
}

void Scene::UpdateVoxelsOnTerrain()
{
	for (int i = 0; i < static_cast<int>(kBorderExtents.z / Grid::mkVoxelWidth); ++i) {
		for (int j = 0; j < static_cast<int>(kBorderExtents.x / Grid::mkVoxelWidth); ++j) {
			Vec3 pos = GetVoxelPos(Pos{i, j, 0});
			Pos index = Pos{ i, j, static_cast<int>(std::round(GetTerrainHeight(pos.x, pos.z))) };
			Pos upIndex = index.Up();

			// 위 복셀이 스태틱이면 해당 아래 복셀도 스태틱으로 설정
			VoxelState upState = GetVoxelState(upIndex);
			if (upState == VoxelState::Static || upState == VoxelState::CanWalk) {
				UpdateVoxelsProximityCost(index);
			}
			else {
				SetVoxelState(index, VoxelState::Terrain);
			}
		}
	}
}

void Scene::UpdateVoxelsProximityCost(const Pos& index, bool isReset)
{
	constexpr int radius = 5;
	int maxCost = static_cast<int>(std::sqrt(radius * radius));

	for (int dz = -radius; dz <= radius; ++dz) {
		for (int dx = -radius; dx <= radius; ++dx) {
			int nx = index.X + dx;
			int nz = index.Z + dz;
			double distance = std::sqrt((nx - index.X) * (nx - index.X) + (nz - index.Z) * (nz - index.Z));
			int cost = isReset ? 0 : static_cast<int>(maxCost - distance);
			Vec3 pos = GetVoxelPos(Pos{ nz, nx, 0 });
			int yIndex = static_cast<int>(std::round(GetTerrainHeight(pos.x, pos.z)));
			SetProximityCost(Pos{ nz, nx, yIndex }, cost, isReset);
		}
	}
}

void Scene::LoadSceneObjects(const std::string& fileName)
{
	std::ifstream file = FileIO::OpenBinFile(fileName);

	mLight->BuildLights(file);
	LoadGameObjects(file);
}

void Scene::LoadGameObjects(std::ifstream& file)
{
	std::string token{};
	std::string name{};

	int objectCount;
	FileIO::ReadString(file, token); // "<GameObjects>:"
	FileIO::ReadVal(file, objectCount);

	mStaticObjects.reserve(objectCount);
	mDynamicObjects.reserve(objectCount);

	int sameObjectCount{};			// get one unique model from same object
	sptr<MasterModel> model{};
	sptr<ObjectPool> objectPool{};

	bool isInstancing{};
	ObjectTag tag{};
	ObjectLayer layer{};
	for (int i = 0; i < objectCount; ++i) {
		sptr<GridObject> object{};

		if (sameObjectCount <= 0) {
			FileIO::ReadString(file, token); //"<Tag>:"
			FileIO::ReadString(file, token);
			tag = GetTagByString(token);
			
			int layerNum{};
			FileIO::ReadString(file, token); //"<Layer>:"
			FileIO::ReadVal(file, layerNum);
			layer = GetLayerByNum(layerNum);

			FileIO::ReadString(file, token); //"<FileName>:"

			std::string meshName{};
			FileIO::ReadString(file, meshName);
			model = RESOURCE<MasterModel>(meshName);

			FileIO::ReadString(file, token); //"<Transforms>:"
			FileIO::ReadVal(file, sameObjectCount);

			FileIO::ReadString(file, token); //"<IsInstancing>:"
			FileIO::ReadVal(file, isInstancing);

			if (isInstancing) {
				objectPool = CreateObjectPool(model, sameObjectCount, [&](rsptr<InstObject> object) {
					object->SetTag(tag);
					});
			}
		}

		if (isInstancing) {
			// 인스턴싱 객체는 생성된 객체를 받아온다.
			object = objectPool->Get(false);
		}
		else {
			object = std::make_shared<GridObject>();
			object->SetModel(model);
			InitObjectByTag(tag, object);
		}


		object->SetLayer(layer);

		Matrix transform;
		FileIO::ReadVal(file, transform);
		object->SetWorldTransform(transform);

		--sameObjectCount;
	}
}

void Scene::InitObjectByTag(ObjectTag tag, sptr<GridObject> object)
{
	object->SetTag(tag);
	ObjectType type = object->GetType();

	switch (type) {
	case ObjectType::Dynamic:
		mDynamicObjects.push_back(object);
		break;
	case ObjectType::Env:
		mEnvironments.push_back(object);
		break;
	default:
		mStaticObjects.push_back(object);
		break;
	}
}
#pragma endregion





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Render
namespace {
	bool IsBehind(const Vec3& point, const Vec4& plane)
	{
		return XMVectorGetX(XMPlaneDotCoord(XMLoadFloat4(&plane), _VECTOR(point))) < 0.f;
	}
}

void Scene::ClearRenderedObjects()
{
	mRenderedObjects.clear();
	mSkinMeshObjects.clear();
	mTransparentObjects.clear();
	mGridObjects.clear();
	mCullingGrids.clear();
}


void Scene::RenderShadow()
{
	if (!DXGIMgr::I->GetFilterOption(FilterOption::Shadow))
		return;

#pragma region PrepareRender
	CMD_LIST->SetGraphicsRootConstantBufferView(DXGIMgr::I->GetGraphicsRootParamIndex(RootParam::Pass), FRAME_RESOURCE_MGR->GetPassCBGpuAddr(1));
#pragma endregion

#pragma region Shadow_Global
	RenderGridObjects(RenderType::Shadow);
#pragma endregion
#pragma region Shadow_SkinMesh
	RenderSkinMeshObjects(RenderType::Shadow);
#pragma endregion
#pragma region Shadow_ObjectInst
	RenderInstanceObjects(RenderType::Shadow);
#pragma endregion
}


void Scene::RenderDeferred()
{
#pragma region PrepareRender
	CMD_LIST->SetGraphicsRootConstantBufferView(DXGIMgr::I->GetGraphicsRootParamIndex(RootParam::Pass), FRAME_RESOURCE_MGR->GetPassCBGpuAddr(0));
#pragma endregion

#pragma region Globald
	RenderGridObjects(RenderType::Deferred);
	RenderEnvironments();
#pragma endregion
#pragma region ObjectInst
	RenderInstanceObjects(RenderType::Deferred);
#pragma endregion
#pragma region SkinMesh
	RenderSkinMeshObjects(RenderType::Deferred);
#pragma endregion
#pragma region Terrain
	if (VoxelManager::I->GetRenderMode() != RenderMode::Voxel) RenderTerrain();
#pragma endregion
}


void Scene::RenderCustomDepth()
{
	if (!DXGIMgr::I->GetFilterOption(FilterOption::Custom))
		return;

#pragma region CustomDepth_SkinMesh
	RenderSkinMeshObjects(RenderType::CustomDepth);
#pragma endregion
}


void Scene::RenderLights()
{
	if (mLight) {
		mLight->Render();
	}
}

void Scene::RenderFinal()
{
	// 조명에서 출력한 diffuse와 specular를 결합하여 최종 색상을 렌더링한다.
	RESOURCE<Shader>("Final")->Set();
	RESOURCE<ModelObjectMesh>("Rect")->Render();
}

void Scene::RenderForward()
{
	RenderTransparentObjects(); 
	RenderDissolveObjects();
	RenderSkyBox();
	RenderAbilities();
}

void Scene::RenderPostProcessing(int offScreenIndex)
{
	// 포스트 프로세싱에 필요한 상수 버퍼 뷰 설정
	PostPassConstants passConstants;
	passConstants.RT0_OffScreenIndex = offScreenIndex;
	FRAME_RESOURCE_MGR->CopyData(passConstants);
	CMD_LIST->SetGraphicsRootConstantBufferView(DXGIMgr::I->GetGraphicsRootParamIndex(RootParam::PostPass), FRAME_RESOURCE_MGR->GetPostPassCBGpuAddr());

	RESOURCE<Shader>("OffScreen")->Set();
	RESOURCE<ModelObjectMesh>("Rect")->Render();
}

void Scene::RenderUI()
{
	Canvas::I->Render();
	RenderBounds(mRenderedObjects);
}

void Scene::RenderTerrain()
{
	CMD_LIST->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
	RESOURCE<Shader>("Terrain")->Set();

	if (mTerrain) {
		mTerrain->Render();
	}
}

void Scene::RenderTransparentObjects()
{
	RESOURCE<Shader>("Transparent")->Set();
	for (auto& object : mTransparentObjects) {
		object->Render();
	}
}

void Scene::RenderDissolveObjects()
{
	RESOURCE<Shader>("Dissolve")->Set();
	// [destroyTime]초 경과 후 객체 제거
	constexpr float destroyTime = 1.f;
	std::set<sptr<GridObject>> destroyedObjects{};
	for (auto it = mDissolveObjects.begin(); it != mDissolveObjects.end(); ++it) {
		auto object = *it;

		object->mObjectCB.DeathElapsed += DeltaTime() / 2.f;
		object->Render();

		if (object->mObjectCB.DeathElapsed > destroyTime) {
			destroyedObjects.insert(object);
		}
	}

	for (auto& object : destroyedObjects) {
		mDissolveObjects.erase(object);
	}
}

void Scene::RenderSkyBox()
{
	mSkyBox->Render();
}

void Scene::RenderAbilities()
{
	AbilityMgr::I->Render();
}

void Scene::RenderGridObjects(RenderType type)
{
	CMD_LIST->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	BoundingFrustum camFrustum;

	if (type == RenderType::Shadow) {
		RESOURCE<Shader>("Shadow_Global")->Set();
		camFrustum = MAIN_CAMERA->GetFrustumShadow();
	}
	else {
		RESOURCE<Shader>("Global")->Set();
		camFrustum = MAIN_CAMERA->GetFrustum();
	}

	if (mRenderedObjects.empty()) {
		for (auto& grid : mGrids) {
			if (grid->Empty()) {
				continue;
			}

			if (camFrustum.Intersects(grid->GetBB())) {
				mCullingGrids.insert(grid.get());
				auto& objects = grid->GetObjects();
				for (auto& object : objects) {
					if (camFrustum.Intersects(object->GetCollider()->GetBS())) {
						mRenderedObjects.insert(object);
					}
				}
			}
		}

		std::set<GridObject*> disabledObjects{};
		for (auto& object : mRenderedObjects) {
			if (!object->IsActive()) {
				disabledObjects.insert(object);
				continue;
			}
			if (object->IsTransparent()) {
				mTransparentObjects.insert(object);
				continue;
			}

			if (object->IsSkinMesh()) {
				mSkinMeshObjects.insert(object);
			}
			else {
				mGridObjects.insert(object);
			}
		}
		if (!disabledObjects.empty()) {
			std::set<GridObject*> diff;
			std::set_difference(mRenderedObjects.begin(), mRenderedObjects.end(), disabledObjects.begin(), disabledObjects.end(), std::inserter(diff, diff.begin()));
			mRenderedObjects = std::move(diff);
		}
	}

	for (auto& object : mGridObjects)
		object->Render();
}

void Scene::RenderSkinMeshObjects(RenderType type)
{
	switch (type)
	{
	case RenderType::Shadow:
		RESOURCE<Shader>("Shadow_SkinMesh")->Set();
		for (auto& object : mDissolveObjects)
			object->Render();
		break;
	case RenderType::Deferred:
		RESOURCE<Shader>("SkinMesh")->Set();
		break;
	case RenderType::CustomDepth:
		RESOURCE<Shader>("CustomDepth_SkinMesh")->Set();
		for (auto& object : mDissolveObjects)
			object->Render();
		break;
	}

	for (auto& object : mSkinMeshObjects)
		object->Render();
}

void Scene::RenderInstanceObjects(RenderType type)
{
	switch (type)
	{
	case RenderType::Shadow:
		RESOURCE<Shader>("Shadow_ObjectInst")->Set();
		break;
	case RenderType::Deferred:
		RESOURCE<Shader>("ObjectInst")->Set();
		break;
	}

	for (auto& buffer : mObjectPools) {
		buffer->Render();
	}
}

void Scene::RenderEnvironments()
{
	for (auto& env : mEnvironments) {
		env->Render();
	}
}

bool Scene::RenderBounds(const std::set<GridObject*>& renderedObjects)
{
	//CMD_LIST->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
	//RESOURCE<Shader>("Wire")->Set();
	//MeshRenderer::RenderBox(Vec3(100, 13.5f, 105), Vec3(.2f,.2f,.2f));
	//RenderObjectBounds(renderedObjects);
	//RenderGridBounds();
	CMD_LIST->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	RESOURCE<Shader>("Voxel")->Set();
	VoxelManager::I->Render();
	return true;
}

void Scene::RenderObjectBounds(const std::set<GridObject*>& renderedObjects)
{
	for (auto& object : renderedObjects) {
		object->RenderBounds();
	}
}

//#define DRAW_SCENE_GRID_3D
void Scene::RenderGridBounds()
{
	for (const auto& grid : mGrids) {
#ifdef DRAW_SCENE_GRID_3D
		MeshRenderer::Render(grid->GetBB());
#else
		constexpr float kGirdHeight = 5.f;
		Vec3 pos = grid->GetBB().Center;
		pos.y = kGirdHeight;
		MeshRenderer::RenderPlane(pos, (float)kGridWidth, (float)kGridWidth);
#endif
	}
}
#pragma endregion





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Update
void Scene::Start()
{
	/* Awake */
	mTerrain->Awake();
	MainCamera::I->Awake();
	ProcessAllObjects([](sptr<Object> object) {
		object->Awake();
		});

	mGameManager->Awake();

	/* Enable */
	mTerrain->SetActive(true);
	MainCamera::I->SetActive(true);
	ProcessAllObjects([](sptr<Object> object) {
		object->SetActive(true);
		});
	mGameManager->SetActive(true);

	UpdateGridInfo();
	UpdateVoxelsOnTerrain();
}

void Scene::Update()
{
	//CheckCollisions();

	mGameManager->Update();
	UpdateObjects();
	mGameManager->LateUpdate();

	MainCamera::I->Update();
	MAIN_CAMERA->UpdateViewMtx();
	mLight->Update();
	Canvas::I->Update();

	UpdateShaderVars();

	PopObjectBuffer();

	AgentManager::I->Update();
}

void Scene::CheckCollisions()
{
	for (const auto& grid : mGrids) {
		grid->CheckCollisions();
	}
}

void Scene::CheckCollisionCollider(rsptr<Collider> collider, std::vector<GridObject*>& out, CollisionType type) const
{
	int gridIndex = GetGridIndex(collider->GetCenter());
	for (const auto& grid : GetNeighborGrids(gridIndex, true)) {
		if (!collider->Intersects(grid->GetBB())) {
			continue;
		}

		grid->CheckCollisions(collider, out, type);
	}
}

float Scene::CheckCollisionsRay(int gridIndex, const Ray& ray) const
{
	// 상하좌우, 대각선 그리드도 체크 필요
	return mGrids[gridIndex]->CheckCollisionsRay(ray);
}

void Scene::UpdateObjects()
{
	ProcessActiveObjects([this](sptr<Object> object) {
		if (object->IsActive()) {
			object->Update();
		}
		});

	ProcessActiveObjects([this](sptr<Object> object) {
			object->Animate();
		});

	ProcessActiveObjects([this](sptr<Object> object) {
		if (object->IsActive()) {
			object->LateUpdate();
		}
		});
}
#pragma endregion





void Scene::PopObjectBuffer()
{
	if (!mDynamicObjectBuffer.empty()) {
		mDynamicObjects.insert(mDynamicObjects.end(), mDynamicObjectBuffer.begin(), mDynamicObjectBuffer.end());
		mDynamicObjectBuffer.clear();
	}
}

//////////////////* Others *//////////////////
int Scene::GetGridIndex(const Pos& index) const {
	const int gridX = static_cast<int>(index.X * Grid::mkVoxelWidth / kGridWidth);
	const int gridZ = static_cast<int>(index.Z * Grid::mkVoxelHeight / kGridWidth);
	return gridZ * kGridCols + gridX;
}

int Scene::GetGridIndex(Vec3 pos) const
{
	pos.x -= mGridStartPoint;
	pos.z -= mGridStartPoint;

	const int gridX = static_cast<int>(pos.x / kGridWidth);
	const int gridZ = static_cast<int>(pos.z / kGridWidth);

	return gridZ * kGridCols + gridX;
}

Pos Scene::GetVoxelIndex(const Vec3& pos) const
{
	// 월드 포지션으로부터 타일의 고유 인덱스를 계산
	const int voxelInGridIndexX = (pos.x - mGridStartPoint + 0.25f) / Grid::mkVoxelWidth;
	const int voxelInGridIndexZ = (pos.z - mGridStartPoint + 0.25f) / Grid::mkVoxelWidth;
	const int voxelInGridIndexY = static_cast<int>(std::round(pos.y / Grid::mkVoxelHeight));

	return Pos{ voxelInGridIndexZ, voxelInGridIndexX, voxelInGridIndexY };
}

Vec3 Scene::GetVoxelPos(const Pos& index) const
{
	// 타일의 고유 인덱스로부터 월드 포지션을 계산
	const float posX = index.X * Grid::mkVoxelWidth + mGridStartPoint;
	const float posZ = index.Z * Grid::mkVoxelWidth + mGridStartPoint;
	const float posY = index.Y * Grid::mkVoxelHeight;

	return Vec3{ posX, posY, posZ };
}

VoxelState Scene::GetVoxelState(const Pos& index) const
{
	return mGrids[GetGridIndex(index)]->GetVoxelState(index);
}

VoxelCondition Scene::GetVoxelCondition(const Pos& index) const
{
	return mGrids[GetGridIndex(index)]->GetVoxelCondition(index);
}

int Scene::GetProximityCost(const Pos& index) const
{
	return mGrids[GetGridIndex(index)]->GetProximityCost(index);
}

float Scene::GetEdgeCost(const Pos& index, bool isRowEdge) const
{
	return mGrids[GetGridIndex(index)]->GetEdgeCost(index, isRowEdge);
}

Voxel Scene::GetVoxel(const Pos& index) const
{
	return mGrids[GetGridIndex(index)]->GetVoxel(index);
}

PairMapRange Scene::GetCanWalkVoxels(const Pos& index) const
{
	return mGrids[GetGridIndex(index)]->GetCanWalkVoxels(index);
}

void Scene::RemoveCanWalkVoxel(const Pos& index, VoxelState state) const
{
	return mGrids[GetGridIndex(index)]->RemoveCanWalkVoxel(index, state);
}

void Scene::SetVoxelState(const Pos& index, VoxelState state) const
{
	mGrids[GetGridIndex(index)]->SetVoxelState(index, state);
}

void Scene::SetVoxelCondition(const Pos& index, VoxelCondition condition) const
{
	mGrids[GetGridIndex(index)]->SetVoxelCondition(index, condition);
}

void Scene::SetProximityCost(const Pos& index, int cost, bool isReset) const
{
	mGrids[GetGridIndex(index)]->SetProximityCost(index, cost, isReset);
}

void Scene::ToggleDrawBoundings()
{
	mIsRenderBounds = !mIsRenderBounds;

	ProcessAllObjects([](sptr<GridObject> object) {
		object->ToggleDrawBoundings();
		});
}

void Scene::ToggleFilterOptions()
{
	static UINT8 filterIdx = 0;
	static std::array<DWORD, 5> values = { 0x004, 0x008, 0x010, 0x020, 0x002 };
	DXGIMgr::I->SetFilterOptions(values[filterIdx++]);
	filterIdx %= values.size();

	if (filterIdx == 0)
		std::reverse(values.begin(), values.end());
}

void Scene::SetFilterOptions(DWORD option)
{
	DXGIMgr::I->SetFilterOptions(option);
}

void Scene::UpdateObjectGrid(GridObject* object, bool isCheckAdj)
{
	const int gridIndex = GetGridIndex(object->GetPosition());

	if (IsGridOutOfRange(gridIndex)) {
		RemoveObjectFromGrid(object);

		object->SetGridIndex(-1);
		return;
	}

	// remove object from current grid if move to another grid
	if (gridIndex != object->GetGridIndex()) {
		RemoveObjectFromGrid(object);
	}

	// ObjectCollider가 활성화된 경우
	// 1칸 이내의 "인접 그리드(8개)와 충돌검사"
	const auto& collider = object->GetCollider();
	if (collider && collider->IsActive()) {
		std::unordered_set<int> gridIndices{ gridIndex };
		const auto& objectBS = collider->GetBS();

		// BoundingSphere가 Grid 내부에 완전히 포함되면 "인접 그리드 충돌검사" X
		if (isCheckAdj && mGrids[gridIndex]->GetBB().Contains(objectBS) != ContainmentType::CONTAINS) {

			for (const auto& neighborGrid : GetNeighborGrids(gridIndex)) {
				if (neighborGrid->GetBB().Intersects(objectBS)) {
					neighborGrid->AddObject(object);
					gridIndices.insert(neighborGrid->GetIndex());
				}
				else {
					neighborGrid->RemoveObject(object);
				}
			}

			object->SetGridIndices(gridIndices);
		}
	}

	object->SetGridIndex(gridIndex);
	mGrids[gridIndex]->AddObject(object);
}

void Scene::RemoveObjectFromGrid(GridObject* object)
{
	for (const int gridIndex : object->GetGridIndices()) {
		if (!IsGridOutOfRange(gridIndex)) {
			mGrids[gridIndex]->RemoveObject(object);
		}
	}

	object->ClearGridIndices();
}

sptr<GridObject> Scene::Instantiate(const std::string& modelName, ObjectTag tag, ObjectLayer layer, bool enable)
{
	const auto& model = RESOURCE<MasterModel>(modelName);
	if (!model) {
		return nullptr;
	}

	sptr<GridObject> instance = std::make_shared<GridObject>();
	instance->SetModel(model);
	instance->SetTag(tag);
	instance->SetLayer(layer);
	if (enable) {
		instance->SetActive(true);
	}
	mDynamicObjectBuffer.push_back(instance);

	return instance;
}


void Scene::RemoveDynamicObject(GridObject* target)
{
	for (size_t i = 0; i < mDynamicObjects.size();++i) {
		auto& object = mDynamicObjects[i];
		if (object.get() == target) {
			if (object->IsSkinMesh()) {
				mDissolveObjects.insert(object);
			}
			mDestroyObjects.insert(i);
			object = nullptr;
			return;
		}
	}
}


sptr<ObjectPool> Scene::CreateObjectPool(const std::string& modelName, int maxSize, const std::function<void(rsptr<InstObject>)>& objectInitFunc)
{
	return CreateObjectPool(RESOURCE<MasterModel>(modelName), maxSize, objectInitFunc);
}

sptr<ObjectPool> Scene::CreateObjectPool(rsptr<const MasterModel> model, int maxSize, const std::function<void(rsptr<InstObject>)>& objectInitFunc)
{
	sptr<ObjectPool> pool = mObjectPools.emplace_back(std::make_shared<ObjectPool>(model, maxSize));
	pool->CreateObjects<InstObject>(objectInitFunc);

	return pool;
}

std::vector<sptr<Grid>> Scene::GetNeighborGrids(int gridIndex, bool includeSelf) const
{
	std::vector<sptr<Grid>> result;
	result.reserve(9);

	const int gridX = gridIndex % kGridCols;
	const int gridZ = gridIndex / kGridCols;

	for (int offsetZ = -1; offsetZ <= 1; ++offsetZ) {
		for (int offsetX = -1; offsetX <= 1; ++offsetX) {
			const int neighborX = gridX + offsetX;
			const int neighborZ = gridZ + offsetZ;

			// 인덱스가 전체 그리드 범위 내에 있는지 확인
			if (neighborX >= 0 && neighborX < kGridCols && neighborZ >= 0 && neighborZ < kGridCols) {
				const int neighborIndex = neighborZ * kGridCols + neighborX;

				if (neighborIndex == gridIndex && !includeSelf) {
					continue;
				}

				result.push_back(mGrids[neighborIndex]);
			}
		}
	}

	return result;
}

void Scene::ToggleFullScreen()
{
	DXGIMgr::I->ToggleFullScreen();
}

std::vector<sptr<GridObject>> Scene::FindObjectsByName(const std::string& name)
{
	std::vector<sptr<GridObject>> result{};
	auto FindObjects = [&](sptr<GridObject> object) {
		if (object->GetName() == name) {
			result.push_back(object);
		}
		};

	ProcessAllObjects(FindObjects);

	return result;
}

void Scene::ProcessActiveObjects(std::function<void(sptr<GridObject>)> processFunc)
{
	for (auto& object : mDynamicObjects) {
		if (object && object->IsActive()) {
			processFunc(object);
		}
	}

	RemoveDesrtoyedObjects();

	for (auto& objectPool : mObjectPools) {
		objectPool->DoActiveObjects(processFunc);
	}
}

void Scene::ProcessAllObjects(std::function<void(sptr<GridObject>)> processFunc)
{
	for (auto& object : mDynamicObjects) {
		if (object) {
			processFunc(object);
		}
	}

	RemoveDesrtoyedObjects();

	for (auto& object : mObjectPools) {
		object->DoAllObjects(processFunc);
	}
}

void Scene::RemoveDesrtoyedObjects()
{
	for (auto& index : mDestroyObjects | std::ranges::views::reverse) {
		mDynamicObjects[index] = mDynamicObjects.back();
		mDynamicObjects.pop_back();
	}

	mDestroyObjects.clear();
}




ObjectTag Scene::GetTagByString(const std::string& tag)
{
	switch (Hash(tag)) {
	case Hash("Building"):
		return ObjectTag::Building;

	case Hash("Dissolve_Building"):
		return ObjectTag::DissolveBuilding;

	case Hash("Background"):
		return ObjectTag::Environment;

	case Hash("Enemy"):
		return ObjectTag::Enemy;

	case Hash("Prop"):
		return ObjectTag::Prop;

	case Hash("Dynamic"):
		return ObjectTag::Dynamic;

	default:
		//assert(0);
		break;
	}

	return ObjectTag::Unspecified;
}

ObjectLayer Scene::GetLayerByNum(int num)
{
	switch (num) {
	case 0:
		return ObjectLayer::Default;

	case 3:
		return ObjectLayer::Transparent;

	case 4:
		return ObjectLayer::Water;

	default:
		break;
	}

	return ObjectLayer::Default;
}