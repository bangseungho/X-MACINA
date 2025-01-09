#include "Common.hlsl"

struct VSInput_Voxel {
    float3 PosL       : POSITION;
    float3 NormalL    : NORMAL;
    float2 UV         : UV;
    float3 TangentL   : TANGENT;
    float3 BiTangentL : BITANGENT;
    uint   ID         : SV_InstanceID;
};

struct VSOutput_Voxel {
    float4 PosH       : SV_POSITION;
    float3 PosW       : POSITION;
    float3 NormalW    : NORMAL;
    float3 TangentW   : TANGENT;
    float3 BiTangentW : BITANGENT;
    float2 UV         : UV;
    uint   ID         : ID;
};

VSOutput_Voxel VSVoxel(VSInput_Voxel input)
{
    VSOutput_Voxel output;

    matrix mtxWorld = gInstBuffer[input.ID].MtxObject;
    output.PosH = mul(mul(mul(float4(input.PosL, 1.f), mtxWorld), gPassCB.MtxView), gPassCB.MtxProj);
    output.NormalW = mul(input.NormalL, (float3x3) mtxWorld);
    output.UV = input.UV;
    output.ID = input.ID;

    return output;
}