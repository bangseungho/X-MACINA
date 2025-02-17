#include "Light.hlsl"

struct VSOutput_Water
{
    float4 PosH    : SV_POSITION;
    float3 PosW    : POSITION;
    float3 NormalW : NORMAL;
    float2 UV      : UV;
};

float4 PSWater(VSOutput_Water pin) : SV_TARGET
{
    MaterialInfo matInfo = gMaterialBuffer[gObjectCB.MatIndex];
    float4 diffuse       = matInfo.Diffuse;
    float metallic       = matInfo.Metallic;
    float roughness      = matInfo.Roughness;
    int diffuseMapIndex  = matInfo.DiffuseMap0Index;
    int normalMapIndex   = matInfo.NormalMapIndex;
    int metallicMapIndex = matInfo.MetallicMapIndex;
    int emissiveMapIndex = matInfo.EmissiveMapIndex;
    
    // diffuseMap을 사용할 경우 샘플링하여 계산
    if (diffuseMapIndex != -1)
    {
        diffuse *= GammaDecoding(gTextureMaps[diffuseMapIndex].Sample(gsamAnisotropicWrap, pin.UV));
    }
    
    pin.NormalW = normalize(pin.NormalW);
    
    // sampling emissiveMap
    float4 emissiveMapSample = (float4)0;
    if (metallicMapIndex != -1)
    {
        emissiveMapSample = gTextureMaps[emissiveMapIndex].Sample(gsamAnisotropicWrap, pin.UV);
    }
    
    // roughness map을 사용할 경우 샘플링하여 roughness 값 계산
    if (metallicMapIndex != -1)
    {
        metallic *= gTextureMaps[metallicMapIndex].Sample(gsamAnisotropicWrap, pin.UV).x;
    }
    
    // 해당 픽셀에서 카메라까지의 벡터
    float3 toCameraW = gPassCB.CameraPos - pin.PosW;
    
    // 전역 조명의 ambient 값을 계산한다.
    float4 ambient = gPassCB.GlobalAmbient * diffuse;
    
    // 메탈릭 값을 적용
    float3 diffuseAlbedo = lerp(diffuse.xyz, 0.0f, metallic);
    float3 specularAlbedo = lerp(0.03f, diffuse.xyz, metallic);
    Material mat = { diffuseAlbedo, specularAlbedo, metallic, roughness };
    
    float3 shadowFactor = 1.f;
    LightColor lightColor = ComputeLighting(mat, pin.PosW, pin.NormalW, toCameraW, shadowFactor);
    
    // specular reflection
    float3 r = reflect(-toCameraW, pin.NormalW);
    float4 reflectionColor = gSkyBoxMaps[gPassCB.SkyBoxIndex].Sample(gsamLinearWrap, r);
    float3 fresnelFactor = SchlickFresnel(specularAlbedo, pin.NormalW, r);
    float3 reflection = (metallic) * fresnelFactor * reflectionColor.rgb;
    
    // rim light
    float rimWidth = 0.7f;
    float gRimLightFactor = 0.f;
    float4 gRimLightColor = float4(1.f, 0.6f, 0.f, 0.f);
    float rim = 1.0f - max(0, dot(pin.NormalW, normalize(gPassCB.CameraPos - pin.PosW)));
    rim = smoothstep(1.0f - rimWidth, 1.0f, rim) * gRimLightFactor;
    
    float4 litColor = ambient + gRimLightColor * rim + float4(reflection, 0.f) + float4(lightColor.Diffuse, 0.f) + float4(lightColor.Specular, 0.f);
    litColor.a = diffuse.a;
    
    return litColor;
}