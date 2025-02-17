#pragma once

#pragma region Include
#include "UploadBuffer.h"
#pragma endregion

#pragma region ClassForwardDecl
#pragma endregion

#pragma region EnumClass
enum class BufferType : UINT {
    Pass = 0,
    PostPass,
    Object,
    SkinMesh,
    Ssao,
    Ability,
    Material,

    _count
};

enum {
    BufferTypeCount = static_cast<UINT>(BufferType::_count)
};
#pragma endregion

#pragma region Struct
struct PassConstants {
    Matrix  MtxView{};
    Matrix  MtxProj{};
    Matrix  MtxInvProj{};
    Matrix  MtxShadow{};
    Matrix  MtxNoLagView{};
    Vec3    CameraPos{};
    UINT    LightCount{};
    Vec3    CameraRight{};
    int     FrameBufferWidth{};
    Vec3    CameraUp{};
    int     FrameBufferHeight{};
    std::array<LightInfo, gkMaxSceneLight> Lights{};

    float   DeltaTime{};
    float   TotalTime{};
    float   FogStart = 100.f;
    float   FogRange = 300.f;

    Vec4    GlobalAmbient = {0.15f, 0.15f, 0.15f, 0.0f};
    Vec4    FogColor{};

    int     FilterOption{};
    float   ShadowIntensity{};

    int     SkyBoxIndex                  = -1;
    int     DefaultDsIndex               = -1;
    int     ShadowDsIndex                = -1;
    int     CustomDsIndex                = -1;

    int     RT0G_PositionIndex           = -1;
    int     RT1G_NormalIndex             = -1;
    int     RT2G_DiffuseIndex            = -1;
    int     RT3G_EmissiveIndex           = -1;

    int     RT4G_MetallicSmoothnessIndex = -1;
    int     RT5G_OcclusionIndex          = -1;
    int     RT0L_DiffuseIndex            = -1;
    int     RT1L_SpecularIndex           = -1;

    int     RT2L_AmbientIndex            = -1;
    int     RT0S_SsaoIndex               = -1;
    int     LiveObjectDissolveIndex      = -1;
    int     BuildingDissolveIndex        = -1;

    int     RT0O_OffScreenIndex          = -1;
};

struct PostPassConstants {
    int     RT0_OffScreenIndex = -1;
    Vec3    Padding;
};

struct SkinnedConstants {
    Matrix  BoneTransforms[gkSkinBoneSize];
};

struct SsaoConstants {
    Matrix  MtxInvProj{};
    Matrix  MtxProjTex{};
    Vec4    OffsetVectors[14];

    Vec4    BlurWeights[3];
    Vec2    InvRenderTargetSize = { 0.f, 0.f };
    
    float   OcclusionRadius     = 0.5f;
    float   OcclusionFadeStart  = 0.2f;
    float   OcclusionFadeEnd    = 2.f;
    float   SurfaceEpsilon      = 0.05f;

    int     AccessContrast      = 6;
    int     RandomVectorIndex   = -1;
};

struct MaterialData {
    Vec4    DiffuseAlbedo{};
    float   Metallic{};
    float   Roughness{};
    UINT    OcclusionMask{};
    UINT    Padding{};

    std::array<int, TextureMapCount> MapIndices;

public:
    MaterialData();
};
#pragma endregion

#pragma region Class
struct FrameResource : private UnCopyable {
public:
    UINT64                                      Fence{};
    ComPtr<ID3D12CommandAllocator>              CmdAllocator{};
    
    uptr<UploadBuffer<PassConstants>>           PassCB{};               // 패스 상수 버퍼
    uptr<UploadBuffer<PostPassConstants>>       PostPassCB{};           // 포스트 프로세싱 패스 상수 버퍼
    uptr<UploadBuffer<ObjectConstants>>         ObjectCB{};             // 오브젝트 상수 버퍼
    uptr<UploadBuffer<SkinnedConstants>>        SkinMeshCB{};           // 스킨메쉬 상수 버퍼
    uptr<UploadBuffer<SsaoConstants>>           SsaoCB{};               // SSAO 상수 버퍼
    uptr<UploadBuffer<AbilityConstants>>        AbilityCB{};            // Ability 상수 버퍼
    uptr<UploadBuffer<MaterialData>>            MaterialBuffer{};       // 머티리얼 버퍼

public:
#pragma region C/Dtor
    FrameResource(ID3D12Device* pDevice, const std::array<int, BufferTypeCount>& bufferCounts);
    ~FrameResource() = default;
#pragma endregion
};

// 모든 프레임 리소스를 관리하는 클래스
class FrameResourceMgr {
public:
    static constexpr int mkFrameResourceCount = 3;

private:

    std::array<int, BufferTypeCount> mBufferCounts;

    ID3D12Fence*                     mFence{};
    std::vector<uptr<FrameResource>> mFrameResources{};
    FrameResource*                   mCurrFrameResource{};       // 현재 프레임 리소스
    int								 mCurrFrameResourceIndex{};	// 현재 프레임 인덱스

    std::array<std::unordered_set<int>, BufferTypeCount> mActiveIndices{};       // 사용중인 인덱스 집합
    std::array<std::queue<int>, BufferTypeCount>         mAvailableIndices{};    // 사용가능 인덱스 큐

public:
#pragma region C/Dtor
    FrameResourceMgr(ID3D12Fence* fence);
    ~FrameResourceMgr() = default;
#pragma endregion

public:
#pragma region Getter
    FrameResource* GetCurrFrameResource() const { return mCurrFrameResource; }
    const D3D12_GPU_VIRTUAL_ADDRESS GetPassCBGpuAddr(int elementIndex = 0) const;
    const D3D12_GPU_VIRTUAL_ADDRESS GetObjCBGpuAddr(int elementIndex = 0) const;
    const D3D12_GPU_VIRTUAL_ADDRESS GetSKinMeshCBGpuAddr(int elementIndex = 0) const;
    const D3D12_GPU_VIRTUAL_ADDRESS GetPostPassCBGpuAddr(int elementIndex = 0) const;
    const D3D12_GPU_VIRTUAL_ADDRESS GetSSAOCBGpuAddr(int elementIndex = 0) const;
    const D3D12_GPU_VIRTUAL_ADDRESS GetAbilityCBGpuAddr(int elementIndex = 0) const;
    const D3D12_GPU_VIRTUAL_ADDRESS GetMatBufferGpuAddr(int elementIndex = 0) const;

    template<typename T>
    static const D3D12_GPU_VIRTUAL_ADDRESS GetBufferGpuAddr(int elementIndex, UploadBuffer<T>* buffer);

    int GetCurrFrameResourceIndex() const { return mCurrFrameResourceIndex; }

#pragma endregion
public:
    // 프레임 리소스 생성
    void CreateFrameResources(ID3D12Device* pDevice);

    // 프레임 리소스 배열을 순환하며 사용 가능 프레임 리소스를 얻어온다.
    void Update();

    void WaitForGpuComplete();

    // 패스 당 상수 버퍼에 데이터 복사
    void CopyData(int elementIndex, const PassConstants& data);
    // 포스트 패스 당 상수 버퍼에 데이터 복사
    void CopyData(const PostPassConstants& data);
    // SSAO 당 상수 버퍼에 데이터 복사
    void CopyData(const SsaoConstants& data);
    // 오브젝트 당 상수 버퍼에 데이터 복사
    void CopyData(int& elementIndex, const ObjectConstants& data);
    // 머티리얼 당 구조적 버퍼에 데이터 복사
    void CopyData(int& elementIndex, const MaterialData& data);
    // 스킨메쉬 당 상수 버퍼에 데이터 복사
    void CopyData(int& elementIndex, const SkinnedConstants& data);
    // 어빌리티 당 상수 버퍼에 데이터 복사
    void CopyData(int& elementIndex, const AbilityConstants& data);

    // 사용중이지 않은 인덱스를 할당
    void AllocIndex(int& elementIndex, BufferType bufferType);
    // 객체 소멸시 사용하지 않는 버퍼 인덱스를 반환
    void ReturnIndex(int elementIndex, BufferType bufferType);
};

#pragma endregion

template<typename T>
inline const D3D12_GPU_VIRTUAL_ADDRESS FrameResourceMgr::GetBufferGpuAddr(int elementIndex, UploadBuffer<T>* buffer)
{
    return buffer->Resource()->GetGPUVirtualAddress() + elementIndex * buffer->GetElementByteSize();
}
