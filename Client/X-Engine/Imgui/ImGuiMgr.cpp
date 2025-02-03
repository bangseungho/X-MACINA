#include "EnginePch.h"

#include "ImguiCode//imgui.h"
#include "ImguiCode/imgui_impl_win32.h"
#include "ImguiCode/imgui_impl_dx12.h"
#include "ImguiCode/imgui_internal.h"

#include "ImGuiMgr.h"
#include "DXGIMgr.h"
#include "Scene.h"
#include "Object.h"
#include "Component/ParticleSystem.h"
#include "Component/Agent.h"
#include "VoxelManager.h"
#include <iostream>
#include "Grid.h"

ImGuiMgr::ImGuiMgr()
{
}

ImGuiMgr::~ImGuiMgr()
{
    DestroyImGui();
}

bool ImGuiMgr::Init()
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
    uptr<ImGuiFunc> pathFunc = std::make_unique<ImGuiPathFunc>(Vec2{ 0, 300 }, Vec2{ 450, 300 });
    mFuncs.emplace_back(std::move(voxelFunc));
    mFuncs.emplace_back(std::move(pathFunc));

    return true;
}

void ImGuiMgr::Render_Prepare()
{
    if (!mOnImGui)
        return;

    // Start the Dear ImGui frame
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
}

void ImGuiMgr::Update()
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

void ImGuiMgr::Render()
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

void ImGuiMgr::DestroyImGui()
{
    // Cleanup
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

void ImGuiMgr::FocusOff()
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
	ImGui::Begin(mName.c_str(), nullptr, ImGuiMgr::I->GetMoveWindow() ? ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove : ImGuiWindowFlags_None);
}

void ImGuiFunc::ExecuteEnd()
{
	ImGui::End();
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
	case VoxelState::TerrainStatic:
		voxelName = "TerrainStatic";
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

	{
		ImGui::Text("Agent :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);

		if (ImGui::Button("Add Agent", ImVec2{ 100, 30 })) {
			Scene::I->Instantiate("EliteTrooper")->AddComponent<Agent>();
		}

		ImGui::SameLine();

		if (ImGui::Button("Clear Path", ImVec2{ 100, 30 })) {
			AgentManager::I->ClearPathList();
		}
	}
}

void ImGuiPathFunc::Execute(GameObject* selectedObject)
{
	// agent speed
	{
		float value = PathOption::I->GetAgentSpeed();
		ImGui::Text("AgentSpeed :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::InputFloat("##float_AgentSpeed", &value, 0.5f, 1.0f, "%.3f"); // float 입력 박스
		PathOption::I->SetAgentSpeed(max(0.f, value));
	}

	// allowed height
	{
		int value = PathOption::I->GetAllowedHeight();
		ImGui::Text("AllowedHeight :"); // 안내 텍스트
		ImGui::SameLine(mTextSpacing);
		ImGui::InputInt("##int_AllowedHeight", &value, 1, 10);
		PathOption::I->SetAllowedHeight(max(0, value));
	}

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

	{
		ImGui::Text("Heuristic : ");
		ImGui::SameLine(mTextSpacing);
		Heuristic heuri = PathOption::I->GetHeuristic();
		if (ImGui::RadioButton("Manhattan", heuri == Heuristic::Manhattan)) heuri = Heuristic::Manhattan; ImGui::SameLine();
		if (ImGui::RadioButton("Euclidean", heuri == Heuristic::Euclidean)) heuri = Heuristic::Euclidean;
		PathOption::I->SetHeuristic(heuri);
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
}
