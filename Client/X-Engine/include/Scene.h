#pragma once


#pragma region Include
#include "Grid.h"
#pragma endregion


#pragma region Define
#define GAME_MGR Scene::I->GetGameManager()
#pragma endregion


#pragma region Using
#pragma endregion


#pragma region ClassForwardDecl
class Camera;
class Object;
class GameObject;
class GridObject;
class InstObject;
class Terrain;
class Light;
class SkyBox;
class ObjectPool;
class TestCube;
class MasterModel;
class ObjectTag;
#pragma endregion


#pragma region EnumClass
enum class RenderType : UINT8 {
	Shadow = 0,
	Deferred,
	CustomDepth,
};
#pragma endregion


#pragma region Class
class Scene : public Singleton<Scene> {
	friend Singleton;

public:
	enum class FXType {
		SmallExplosion = 0,
		BigExplosion
	};


private:
	/* Light */
	sptr<Light> mLight{};

	/* SkyBox */
	sptr<SkyBox> mSkyBox{};

	/* Object */
	sptr<Object>					mGameManager{};
	std::vector<sptr<GameObject>>	mEnvironments{};
	std::vector<sptr<GridObject>>	mStaticObjects{};
	std::vector<sptr<GridObject>>	mDynamicObjects{};
	std::vector<sptr<ObjectPool>>	mObjectPools{};
	std::vector<sptr<GridObject>>	mDynamicObjectBuffer{};		// 추가(Instantiate) 대기 버퍼
	std::set<size_t>				mDestroyObjects{};

	std::set<sptr<GridObject>>	mDissolveObjects{};
	std::set<GridObject*>	    mRenderedObjects{};
	std::set<GridObject*>	    mTransparentObjects{};
	std::set<GridObject*>	    mSkinMeshObjects{};
	std::set<GridObject*>	    mGridObjects{};

	/* Map */
	sptr<Terrain>		mTerrain{};
	BoundingBox			mMapBorder{};			// max scene range	(grid will be generated within this border)

	/* Grid */
	std::vector<sptr<Grid>>	mGrids{};				// all scene grids
	std::set<Grid*>			mCullingGrids{};		// all scene grids
	float					mGridStartPoint{};		// leftmost coord of the entire grid

	/* Others */
	bool mIsRenderBounds = false;
	Object* mPlayer{};

private:
#pragma region C/Dtor
	Scene();
	virtual ~Scene() = default;

public:
	void Release();

private:
#pragma endregion

public:
#pragma region Getter
	float GetTerrainHeight(float x, float z) const;
	std::vector<sptr<GameObject>> GetAllObjects() const;
	rsptr<Object>GetGameManager() const { return mGameManager; }

	int				GetGridIndex(const Pos& index) const;
	int				GetGridIndex(Vec3 pos) const;
	Pos				GetVoxelIndex(const Vec3& pos) const;
	Vec3			GetVoxelPos(const Pos& index) const;
	Voxel			GetVoxel(const Pos& index) const;
	PairMapRange	GetCanWalkVoxels(const Pos& index) const;
	void			RemoveCanWalkVoxel(const Pos& index, VoxelState state = VoxelState::Static) const;
	VoxelState		GetVoxelState(const Pos& index) const;
	VoxelCondition	GetVoxelCondition(const Pos& index) const;
	int				GetProximityCost(const Pos& index) const;
	float			GetEdgeCost(const Pos& index, bool isRowEdge) const;
	void			SetVoxelState(const Pos& index, VoxelState state) const;
	void			SetVoxelCondition(const Pos& index, VoxelCondition condition) const;
	void			SetProximityCost(const Pos& index, int cost, bool isReset) const;
#pragma endregion

#pragma region DirectX
public:
	void ReleaseUploadBuffers();
	void UpdateAbilityCB(int& idx, const AbilityConstants& value);
	void SetAbilityCB(int idx) const;
	void SetPlayer(Object* player);

private:
	void UpdateShaderVars();
	void UpdateMainPassCB();
	void UpdateShadowPassCB();
	void UpdateSsaoCB();
	void UpdateMaterialBuffer();
#pragma endregion

#pragma region Build
public:
	void BuildObjects();
	void ReleaseObjects();
	void UpdateVoxelsOnTerrain();
	void UpdateVoxelsProximityCost(const Pos& index, bool isReset = false);

private:
	/* Object */
	void BuildTerrain();

	/* Grid */
	// generate grids
	void BuildGrid();
	// update grid indices for all objects
	void UpdateGridInfo();

	/* Load */
	// 씬 파일에서 모든 객체와 조명의 정보를 불러온다.
	void LoadSceneObjects(const std::string& fileName);
	// 씬 파일에서 모든 객체의 정보를 불러온다. - call from Scene::LoadSceneObjects()
	void LoadGameObjects(std::ifstream& file);

	/* Other */
	// 태그별에 따라 객체를 초기화하고 씬 컨테이너에 객체를 삽입한다.(static, explosive, environments, ...)
	void InitObjectByTag(ObjectTag tag, sptr<GridObject> object);

#pragma endregion

#pragma region Render
public:
	// render scene
	void ClearRenderedObjects();
	void RenderShadow();
	void RenderCustomDepth();
	void RenderDeferred();
	void RenderLights();
	void RenderFinal();
	void RenderForward();
	void RenderPostProcessing(int offScreenIndex);
	void RenderUI();

private:
	void RenderGridObjects(RenderType type);
	void RenderSkinMeshObjects(RenderType type);
	void RenderEnvironments();
	void RenderInstanceObjects(RenderType type);
	void RenderTerrain();
	void RenderTransparentObjects();
	void RenderDissolveObjects();
	void RenderSkyBox();
	void RenderAbilities();
	bool RenderBounds(const std::set<GridObject*>& renderedObjects);
	void RenderObjectBounds(const std::set<GridObject*>& renderedObjects);
	void RenderGridBounds();
#pragma endregion


#pragma region Update
public:
	void Start();
	void Update();

private:
	void CheckCollisions();

	// update all objects
	void UpdateObjects();

#pragma endregion

public:
	// get objects[out] that collide with [collider] (expensive call cost)
	void CheckCollisionCollider(rsptr<Collider> collider, std::vector<GridObject*>& out, CollisionType type = CollisionType::All) const;
	float CheckCollisionsRay(int gridIndex, const Ray& ray) const;
	void ToggleDrawBoundings();
	void ToggleFilterOptions();
	void SetFilterOptions(DWORD option);


	// update objects' grid indices
	void UpdateObjectGrid(GridObject* object, bool isCheckAdj = true);
	void RemoveObjectFromGrid(GridObject* object);

	// create new game object from model
	sptr<GridObject> Instantiate(const std::string& modelName, ObjectTag tag = ObjectTag::Unspecified, ObjectLayer layer = ObjectLayer::Default, bool enable = true);

	void AddDynamicObject(rsptr<GridObject> object) { mDynamicObjects.push_back(object); }
	void RemoveDynamicObject(GridObject* object);

	sptr<ObjectPool> CreateObjectPool(const std::string& modelName, int maxSize, const std::function<void(rsptr<InstObject>)>& objectInitFunc = nullptr);
	sptr<ObjectPool> CreateObjectPool(rsptr<const MasterModel> model, int maxSize, const std::function<void(rsptr<InstObject>)>& objectInitFunc = nullptr);

	std::vector<sptr<Grid>> GetNeighborGrids(int gridIndex, bool includeSelf = false) const;

	void ToggleFullScreen();

	std::vector<sptr<GridObject>> FindObjectsByName(const std::string& name);

private:
	// do [processFunc] for activated objects
	void ProcessActiveObjects(std::function<void(sptr<GridObject>)> processFunc);
	// do [processFunc] for all objects
	void ProcessAllObjects(std::function<void(sptr<GridObject>)> processFunc);

	void RemoveDesrtoyedObjects();

	// move mObjectBuffer's objects to mDynamicObjects
	void PopObjectBuffer();

	bool IsGridOutOfRange(int index) { return index < 0 || index >= mGrids.size(); }

	// 유니티의 tag 문자열을 ObjectTag로 변환한다.
	static ObjectTag GetTagByString(const std::string& tag);

	// 유니티의 Layer 번호[num]를 ObjectLayer로 변환한다.
	static ObjectLayer GetLayerByNum(int num);
};
#pragma endregion
