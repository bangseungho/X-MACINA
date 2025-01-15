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
	io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows

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

    //uptr<ImGuiFunc> hierachyFunc = std::make_unique<ImGuiHierarchyFunc>(Vec2{ 0, 0 }, Vec2{ 300, 400 });
    //uptr<ImGuiFunc> inspectorFunc = std::make_unique<ImGuiInspectorFunc>(Vec2{ 0, 400 }, Vec2{ 300, 400 });
    uptr<ImGuiFunc> voxelFunc = std::make_unique<ImGuiVoxelFunc>(Vec2{ 0, 0 }, Vec2{ 300, 200 });
    //mFuncs.emplace_back(std::move(hierachyFunc));
    //mFuncs.emplace_back(std::move(inspectorFunc));
    mFuncs.emplace_back(std::move(voxelFunc));

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
	//ImGui::FocusWindow(NULL);
	//mIsFocused = false;
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
    // 창 크기 및 위치 조정
    RECT rect;
	GetWindowRect(DXGIMgr::I->GetHwnd(), &rect);
	ImGui::SetNextWindowPos(ImVec2{ rect.left + mPosition.x + 5, rect.top + mPosition.y + 32 });
	ImGui::SetNextWindowSize(ImVec2{ mSize.x, mSize.y });
	ImGui::Begin(mName.c_str(), nullptr, ImGuiMgr::I->GetMoveWindow() ? ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove : ImGuiWindowFlags_None);
}

void ImGuiFunc::ExecuteEnd()
{
	ImGui::End();
}

void ImGuiHierarchyFunc::Execute(GameObject* selectedObject)
{
}

void ImGuiInspectorFunc::Execute(GameObject* selectedObject)
{
}

void ImGuiVoxelFunc::Execute(GameObject* selectedObject)
{
    // index
    const Pos& selectedVoxelIndex = VoxelManager::I->GetSelectedVoxelPos();
    ImGui::Text("Idx : x = %d, y = %d, z = %d", selectedVoxelIndex.X, selectedVoxelIndex.Y, selectedVoxelIndex.Z);

    // position
    const Vec3& selectedVoxelPosW = Scene::I->GetTilePosFromUniqueIndex(selectedVoxelIndex);
	ImGui::Text("Pos : x = %.1f, y = %.1f, z = %.1f", selectedVoxelPosW.x, selectedVoxelPosW.y, selectedVoxelPosW.z);

    // agent speed
    {
		float value = PathOption::I->GetAgentSpeed();
		ImGui::Text("AgentSpeed :"); // 안내 텍스트
		ImGui::InputFloat("##float_input", &value, 0.5f, 1.0f, "%.3f"); // float 입력 박스
		PathOption::I->SetAgentSpeed(value);
    }

    // allowed height
    {
		int value = PathOption::I->GetAllowedHeight();
		ImGui::Text("AllowedHeight :"); // 안내 텍스트
        ImGui::InputInt("##int_input", &value);
		PathOption::I->SetAllowedHeight(value);
    }
}
