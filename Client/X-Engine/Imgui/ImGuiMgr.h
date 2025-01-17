#pragma once
#include "Singleton.h"

class GameObject;

class ImGuiFunc {
protected:
	Vec2			mPosition{};
	Vec2			mSize{};
	std::string		mName{};
	float			mTextSpacing = 130;

public:
	ImGuiFunc(const Vec2& pos, const Vec2& size, std::string label);
	virtual ~ImGuiFunc() = default;

public:
	std::string GetLabel() const { return mName; }

public:
	void ExecuteBegin();
	void ExecuteEnd();

public:
	virtual void Execute(GameObject* selectedObject) abstract;
};


class ImGuiVoxelFunc : public ImGuiFunc {
	using base = ImGuiFunc;

public:
	ImGuiVoxelFunc(const Vec2& pos, const Vec2& size) : ImGuiFunc(pos, size, "Voxel") {}

public:
	virtual void Execute(GameObject* selectedObject) override;
};


class ImGuiPathFunc : public ImGuiFunc {
	using base = ImGuiFunc;

public:
	ImGuiPathFunc(const Vec2& pos, const Vec2& size) : ImGuiFunc(pos, size, "Path") {}

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

