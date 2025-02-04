#include "EnginePch.h"

#include "ImguiCode//imgui.h"
#include "ImguiCode/imgui_impl_win32.h"
#include "ImguiCode/imgui_impl_dx12.h"
#include "ImguiCode/imgui_internal.h"
#include "ImguiCode/ImGuizmo.h"

#include "ImGuiManager.h"
#include "DXGIMgr.h"
#include "Scene.h"
#include "Object.h"
#include "Component/ParticleSystem.h"
#include "Component/Agent.h"
#include "Component/Camera.h"
#include "VoxelManager.h"
#include <iostream>
#include "Grid.h"

ImGuiManager::ImGuiManager()
{
}

ImGuiManager::~ImGuiManager()
{
    DestroyImGui();
}

bool ImGuiManager::Init()
{
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls    
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
	//io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows

    //ImGui::StyleColorsLight();
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 8.f;
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.2f, 0.2f, 0.2f, 0.8f);
        style.FramePadding = ImVec2(2.f, 2.f);
        style.FrameRounding = 1.f;
    }

    // create descriptor heap
    D3D12_DESCRIPTOR_HEAP_DESC desc = {};
    desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    desc.NumDescriptors = 1;
    desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    if (FAILED(DEVICE->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&mSrvDescHeap))))
        assert(nullptr);

    bool bSuccess = ImGui_ImplWin32_Init(DXGIMgr::I->GetHwnd());
    bSuccess = ImGui_ImplDX12_Init(DEVICE.Get(), 3, DXGI_FORMAT_R8G8B8A8_UNORM
        , mSrvDescHeap.Get(), mSrvDescHeap->GetCPUDescriptorHandleForHeapStart()
        , mSrvDescHeap.Get()->GetGPUDescriptorHandleForHeapStart());

    uptr<ImGuiFunc> voxelFunc = std::make_unique<ImGuiVoxelFunc>(Vec2{ 0, 0 }, Vec2{ 450, 300 });
    uptr<ImGuiFunc> pathFunc = std::make_unique<ImGuiPathFunc>(Vec2{ 0, 300 }, Vec2{ 450, 200 });
    uptr<ImGuiFunc> agentFunc = std::make_unique<ImGuiAgentFunc>(Vec2{ 0, 500 }, Vec2{ 450, 200 });
    mFuncs.emplace_back(std::move(voxelFunc));
    mFuncs.emplace_back(std::move(pathFunc));
    mFuncs.emplace_back(std::move(agentFunc));

    return true;
}

void ImGuiManager::Render_Prepare()
{
    if (!mOnImGui)
        return;

    // Start the Dear ImGui frame
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
}

void ImGuiManager::Update()
{
    if (!mOnImGui)
        return;

	if (mShowDemo)
		ImGui::ShowDemoWindow(&mShowDemo);

    for (const auto& func : mFuncs) {
        func->ExecuteBegin();
		func->Execute(nullptr);
        func->ExecuteEnd();
	}
}

void ImGuiManager::Render()
{
    if (!mOnImGui)
        return;

    // Rendering
    CMD_LIST->SetDescriptorHeaps(1, mSrvDescHeap.GetAddressOf());

    ImGui::Render();
    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), CMD_LIST.Get());

    // Update and Render additional Platform Windows
    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
		ImGui::UpdatePlatformWindows();
		ImGui::RenderPlatformWindowsDefault();
    }
}

void ImGuiManager::DestroyImGui()
{
    // Cleanup
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

void ImGuiManager::SetAgent(Agent* pickedAgent)
{
	for (const auto& func : mFuncs) {
		func->SetAgent(pickedAgent);
	}
}

void ImGuiManager::FocusOff()
{
	ImGui::FocusWindow(NULL);
	mIsFocused = false;
}

ImGuiFunc::ImGuiFunc(const Vec2& pos, const Vec2& size, std::string label)
	:
	mPosition(pos),
	mSize(size),
	mName(label)
{
}

void ImGuiFunc::ExecuteBegin()
{
	ImGuiStyle& style = ImGui::GetStyle();
	style.Colors[ImGuiCol_WindowBg].w = 0.5f;

    // 창 크기 및 위치 조정
	ImGui::SetNextWindowPos(ImVec2{ mPosition.x, mPosition.y });
	ImGui::SetNextWindowSize(ImVec2{ mSize.x, mSize.y });
	ImGui::Begin(mName.c_str(), nullptr, ImGuiManager::I->GetMoveWindow() ? ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove : ImGuiWindowFlags_None);
}

void ImGuiFunc::ExecuteEnd()
{
	ImGui::End();
}

void ImGuiFunc::SetAgent(Agent* pickedAgent)
{
	mCrntAgent = pickedAgent;
}

void ImGuiVoxelFunc::Execute(GameObject* selectedObject)
{
    // index
    const Pos& selectedVoxelIndex = VoxelManager::I->GetSelectedVoxelPos();
    ImGui::Text("Idx : x = %d, y = %d, z = %d", selectedVoxelIndex.X, selectedVoxelIndex.Y, selectedVoxelIndex.Z);

    // position
    const Vec3& selectedVoxelPosW = Scene::I->GetVoxelPos(selectedVoxelIndex);
	ImGui::Text("Pos : x = %.1f, y = %.1f, z = %.1f", selectedVoxelPosW.x, selectedVoxelPosW.y, selectedVoxelPosW.z);
    
	// costs
	ImGui::Text("Proximity Cost : %d", VoxelManager::I->GetSelectedVoxelProximityCost());
	ImGui::Text("RowEdge Cost : %.1f", VoxelManager::I->GetSelectedVoxelEdgeCost().first);
	ImGui::Text("ColEdge Cost : %.1f", VoxelManager::I->GetSelectedVoxelEdgeCost().second);

    // voxel
    const VoxelState selectedVoxelState = Scene::I->GetVoxelState(selectedVoxelIndex);
    std::string voxelName{};
    switch (selectedVoxelState)
    {
    case VoxelState::None:
        voxelName = "None";
        break;
    case VoxelState::Static:
		voxelName = "Static";
		break;
    case VoxelState::Dynamic:
		voxelName = "Dynamic";
        break;
    case VoxelState::Terrain:
		voxelName = "Terrain";
		break;
    default:
        break;
    }
	ImGui::Text("Voxel : %s", voxelName);

	// render mode
	// TODO : 라디오 버튼에서 체크 박스로 변경 예정 -> 해당 옵션 뿐만 아니라 오픈 클로즈 리스트 그리는 것도 결정
	ImGui::Text("Render Mode : ");
	ImGui::SameLine(mTextSpacing);
	RenderMode renderMode = VoxelManager::I->GetRenderMode();
	if (ImGui::RadioButton("Voxel", renderMode == RenderMode::Voxel)) renderMode = RenderMode::Voxel; ImGui::SameLine();
	if (ImGui::RadioButton("World", renderMode == RenderMode::World)) renderMode = RenderMode::World; ImGui::SameLine();
	if (ImGui::RadioButton("Both", renderMode == RenderMode::Both)) renderMode = RenderMode::Both;
	VoxelManager::I->SetRenderMode(renderMode);

	// create mode
	ImGui::Text("Create Mode : ");
	ImGui::SameLine(mTextSpacing);
	CreateMode createMode = VoxelManager::I->GetCreateMode();
	if (ImGui::RadioButton("None", createMode == CreateMode::None)) createMode = CreateMode::None; ImGui::SameLine();
	if (ImGui::RadioButton("Create", createMode == CreateMode::Create)) createMode = CreateMode::Create; ImGui::SameLine();
	if (ImGui::RadioButton("Remove", createMode == CreateMode::Remove)) createMode = CreateMode::Remove;
	VoxelManager::I->SetCreateMode(createMode);

    // render voxel rows
    {
		int value = VoxelManager::I->GetRenderVoxelRows();
		ImGui::Text("VoxelRows :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::SliderInt("##int_VoxelRows", &value, 10, 200);

		if (mVoxelRowsValue != value) {
			mVoxelRowsValue = value;
			VoxelManager::I->SetRenderVoxelRows(value);
		}
    }

	// render voxel height
	{
		int value = VoxelManager::I->GetRenderVoxelHeight();
		ImGui::Text("VoxelHeight :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::SliderInt("##int_VoxelHeight", &value, 1, 100);

		if (mVoxelHeightValue != value) {
			mVoxelHeightValue = value;
			VoxelManager::I->SetRenderVoxelHeight(value);
		}
	}
}

void ImGuiPathFunc::Execute(GameObject* selectedObject)
{

	// on voxel cost
	{
		int value = PathOption::I->GetOnVoxelCost();
		ImGui::Text("OnVoxelCost :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::InputInt("##int_OnVoxelCostt", &value, 10, 30);
		PathOption::I->SetOnVoxelCost(max(0, value));
	}

	// heuristic weight
	{
		int value = PathOption::I->GetHeuristicWeight();
		ImGui::Text("HeuristicWeight :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::InputInt("##int_HeuristicWeight", &value, 5, 10);
		PathOption::I->SetHeuristicWeight(max(0, value));
	}

	// proximity weight
	{
		int value = PathOption::I->GetProximityWeight();
		ImGui::Text("ProximityWeight :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::InputInt("##int_ProximityWeight", &value, 5, 10);
		PathOption::I->SetProximityWeight(max(0, value));
	}

	// edge weight
	{
		int value = PathOption::I->GetEdgeWeight();
		ImGui::Text("EdgeWeight :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::InputInt("##int_EdgeWeight", &value, 5, 10);
		PathOption::I->SetEdgeWeight(max(0, value));
	}

	ImGui::Text("PostProcess : ");
	{
		ImGui::SameLine(mTextSpacing);
		bool value = PathOption::I->GetDirPathOptimize(); 
		ImGui::Checkbox("DirOptimize", &value);
		PathOption::I->SetDirPathOptimize(value);
		ImGui::SameLine();
	}

	{
		bool value = PathOption::I->GetRayPathOptimize();
		ImGui::Checkbox("RayOptimize", &value);
		PathOption::I->SetRayPathOptimize(value);
		ImGui::SameLine();
	}

	{
		bool value = PathOption::I->GetSplinePath();
		ImGui::Checkbox("SplinePath", &value);
		PathOption::I->SetSplinePath(value);
	}

	// clear path
	{
		ImVec2 windowSize = ImGui::GetContentRegionAvail();
		ImVec2 buttonSize = ImVec2(100, 30);
		float buttonX = (windowSize.x - buttonSize.x) * 0.5f;
		float buttonY = windowSize.y - buttonSize.y;
		ImGui::SetCursorPos(ImVec2(buttonX, buttonY));
		ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 50);
		if (ImGui::Button("Clear Path", ImVec2{ 100, 30 })) {
			AgentManager::I->ClearPathList();
		}
	}
}

void ImGuiAgentFunc::Execute(GameObject* selectedObject)
{
	if (!mCrntAgent) {
		return;
	}

	// heuristic
	{
		ImGui::Text("Heuristic : ");
		ImGui::SameLine(mTextSpacing);
		Heuristic heuri = mCrntAgent->mOption.Heuri;
		if (ImGui::RadioButton("Manhattan", heuri == Heuristic::Manhattan)) heuri = Heuristic::Manhattan; ImGui::SameLine();
		if (ImGui::RadioButton("Euclidean", heuri == Heuristic::Euclidean)) heuri = Heuristic::Euclidean;
		mCrntAgent->mOption.Heuri = heuri;
	}

	// agent speed
	{
		float value = mCrntAgent->mOption.AgentSpeed;
		ImGui::Text("AgentSpeed :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::InputFloat("##float_AgentSpeed", &value, 0.5f, 1.0f, "%.3f"); // float 입력 박스
		mCrntAgent->mOption.AgentSpeed = max(0.f, value);
	}

	// allowed height
	{
		int value = mCrntAgent->mOption.AllowedHeight;
		ImGui::Text("AllowedHeight :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::InputInt("##int_AllowedHeight", &value, 1, 10);
		mCrntAgent->mOption.AllowedHeight = max(0, value);
	}

	// add agent
	{
		ImVec2 windowSize = ImGui::GetContentRegionAvail();
		ImVec2 buttonSize = ImVec2(100, 30);
		float buttonX = (windowSize.x - buttonSize.x) * 0.5f;
		float buttonY = windowSize.y - buttonSize.y;
		ImGui::SetCursorPos(ImVec2(buttonX, buttonY));
		ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 50);
		if (ImGui::Button("Add Agent", ImVec2{ 100, 30 })) {
			Scene::I->Instantiate("EliteTrooper")->AddComponent<Agent>();
		}
	}

	UpdateGuizmo();
}

void ImGuiAgentFunc::UpdateGuizmo()
{
	Matrix mtxWorld = mCrntAgent->GetWorldMatrix();
	Vec3 posWorld = mCrntAgent->GetWorldPosition();

	IMGUIZMO_NAMESPACE::BeginFrame();
	static IMGUIZMO_NAMESPACE::OPERATION		currentGizmoOperation(IMGUIZMO_NAMESPACE::ROTATE);
	static IMGUIZMO_NAMESPACE::MODE			currentGizmoMode(IMGUIZMO_NAMESPACE::WORLD);

	if (ImGui::RadioButton("Scale", currentGizmoOperation == IMGUIZMO_NAMESPACE::SCALE)) currentGizmoOperation = IMGUIZMO_NAMESPACE::SCALE;
	ImGui::SameLine();
	if (ImGui::RadioButton("Rotation", currentGizmoOperation == IMGUIZMO_NAMESPACE::ROTATE)) currentGizmoOperation = IMGUIZMO_NAMESPACE::ROTATE;
	ImGui::SameLine();
	if (ImGui::RadioButton("Translation", currentGizmoOperation == IMGUIZMO_NAMESPACE::TRANSLATE)) currentGizmoOperation = IMGUIZMO_NAMESPACE::TRANSLATE;

	Vec3 translate{}, rotate{}, scale{};
	bool isChanged{};

	IMGUIZMO_NAMESPACE::DecomposeMatrixToComponents((float*)&mtxWorld, (float*)&translate, (float*)&rotate, (float*)&scale);
	if (ImGui::InputFloat3("Scale", (float*)&scale))
		isChanged = true;
	if (ImGui::InputFloat3("Rotation", (float*)&rotate))
		isChanged = true;
	if (ImGui::InputFloat3("Translation", (float*)&translate))
		isChanged = true;
	IMGUIZMO_NAMESPACE::RecomposeMatrixFromComponents((float*)&translate, (float*)&rotate, (float*)&scale, (float*)&mtxWorld);

	if (currentGizmoOperation != ImGuizmo::SCALE)
	{
		if (ImGui::RadioButton("LocalSpace", currentGizmoMode == ImGuizmo::LOCAL))
			currentGizmoMode = ImGuizmo::LOCAL;
		ImGui::SameLine();
		if (ImGui::RadioButton("WorldSpace", currentGizmoMode == ImGuizmo::WORLD))
			currentGizmoMode = ImGuizmo::WORLD;
	}

	static bool isUseSnap{};
	ImGui::Checkbox("Grab", &isUseSnap);
	ImGui::SameLine();
	Vec3 snap = Vec3{ 0.f, 0.f, 0.f };
	switch (currentGizmoOperation)
	{
	case IMGUIZMO_NAMESPACE::TRANSLATE:
		ImGui::InputFloat3("Translate Grab", &snap.x);
		break;
	case IMGUIZMO_NAMESPACE::ROTATE:
		ImGui::InputFloat("Rotate Grab", &snap.x);
		break;
	case IMGUIZMO_NAMESPACE::SCALE:
		ImGui::InputFloat("Scale Grab", &snap.x);
		break;
	}

	ImGuiIO& io = { ImGui::GetIO() };
	IMGUIZMO_NAMESPACE::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
	Matrix ProjMatrix, ViewMatrix;
	ProjMatrix = MAIN_CAMERA->GetProjMtx();
	ViewMatrix = MAIN_CAMERA->GetViewMtx();
	IMGUIZMO_NAMESPACE::Manipulate(&ViewMatrix.m[0][0], &ProjMatrix.m[0][0], currentGizmoOperation, currentGizmoMode, &mtxWorld.m[0][0], NULL, isUseSnap ? &snap.x : NULL);
	mCrntAgent->SetWorldMatrix(mtxWorld);
}





