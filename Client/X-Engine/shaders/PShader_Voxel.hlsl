#include "Common.hlsl"

struct VSOutput_Voxel {
    float4 PosH       : SV_POSITION;
    float3 PosW       : POSITION;
    float3 NormalW    : NORMAL;
    float3 TangentW   : TANGENT;
    float3 BiTangentW : BITANGENT;
    float2 UV         : UV;
	uint   ID         : ID;
};

float4 PSVoxel(VSOutput_Voxel input) : SV_TARGET
{
	// UV 좌표를 사용해 화면 중심으로부터의 거리 계산
	float2 uv = input.UV;
	float2 center = float2(0.5f, 0.5f);
	float distanceFromCenter = distance(uv, center);

	// 거리에 비례하는 강도 계산 (0에서 1로 정규화)
	float intensity = saturate(distanceFromCenter);
    float4 color = gInstBuffer[input.ID].Color;
	
    return float4(color.rgb, color.a * intensity); // RGBA 출력
}