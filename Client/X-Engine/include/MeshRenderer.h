#pragma once

class Scene;
class ModelObjectMesh;

// bounds의 mesh를 렌더링하기 위한 클래스 (for debug)
// texture(UV) 정보를 가지지 않는다.
class MeshRenderer {
	friend Scene;		// 빌드는 Scene 클래스에서 수행한다.

private:
	static uptr<ModelObjectMesh> mWireBoxMesh;
	static uptr<ModelObjectMesh> mBoxMesh;
	static uptr<ModelObjectMesh> mSphereMesh;
	static uptr<ModelObjectMesh> mPlaneMesh;

public:
	MeshRenderer() = delete;

public:
	static void Render(const BoundingBox& box);
	static void Render(const BoundingOrientedBox& box);
	static void Render(const BoundingSphere& bs);

	static void RenderPlane(const Vec3& pos, float width, float length);
	static void RenderBox(const Vec3& pos, const Vec3& size, const Vec4& color = Vec4{1.f, 0.f, 0.f, 1.f});
	static void RenderInstancingBox(UINT instanceCnt);

private:
	static void BuildMeshes();
	static void ReleaseUploadBuffers();
	static void Release();
};