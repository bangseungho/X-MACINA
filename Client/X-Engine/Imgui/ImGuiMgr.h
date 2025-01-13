#pragma once
#include "Singleton.h"

class GameObject;

class ImGuiFunc {
protected:
	Vec2			mPosition{};
	Vec2			mSize{};
	std::string		mName{};

public:
	ImGuiFunc(const Vec2& pos, const Vec2& size, std::string label);
	virtual ~ImGuiFunc() = default;

public:
	std::string GetLabel() const { return mName; }

public:
	void ExecuteBegin();
	void ExecuteEnd();

	virtual void Execute(GameObject* selectedObject) abstract;
};


class ImGuiHierarchyFunc : public ImGuiFunc {
	using base = ImGuiFunc;

public:
	ImGuiHierarchyFunc(const Vec2& pos, const Vec2& size) : ImGuiFunc(pos, size, "Hierarchy") {}

public:
	virtual void Execute(GameObject* selectedObject) override;
};


class ImGuiInspectorFunc : public ImGuiFunc {
	using base = ImGuiFunc;

public:
	ImGuiInspectorFunc(const Vec2& pos, const Vec2& size) : ImGuiFunc(pos, size, "Inspector") {}

public:
	virtual void Execute(GameObject* selectedObject) override;
};


class ImGuiVoxelFunc : public ImGuiFunc {
	using base = ImGuiFunc;

public:
	ImGuiVoxelFunc(const Vec2& pos, const Vec2& size) : ImGuiFunc(pos, size, "Voxel") {}

public:
	virtual void Execute(GameObject* selectedObject) override;
};


class ImGuiMgr : public Singleton<ImGuiMgr>
{
	friend Singleton;

private:
	bool							mOnImGui = true;
	bool							mShowDemo = false;
	bool							mIsFocused = false;
	bool							mNoMoveWindow = true;

private:
	ComPtr<ID3D12DescriptorHeap>	mSrvDescHeap{};

private:
	std::vector<uptr<ImGuiFunc>>	mFuncs{};

public:
	ImGuiMgr();
	~ImGuiMgr();

public:
	void ToggleImGui() { mOnImGui = !mOnImGui; }
	bool Init();
	void Render_Prepare();
	void Update();
	void Render();
	void DestroyImGui();

public:
	bool GetMoveWindow() const { return mNoMoveWindow; }

public:
	bool IsFocused() const { return mIsFocused ; }
	void FocusOff();
};

