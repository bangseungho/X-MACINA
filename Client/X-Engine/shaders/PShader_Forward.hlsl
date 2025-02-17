#include "Light.hlsl"

struct VSOutput_Standard {
    float4 PosH       : SV_POSITION;
    float3 PosW       : POSITION;
    float3 NormalW    : NORMAL;
    float3 TangentW   : TANGENT;
    float3 BiTangentW : BITANGENT;
    float2 UV         : UV;
};

struct PSOutput {
    float4 Result     : SV_TARGET0;
    float4 Normal     : SV_TARGET1;
};

PSOutput PSForward(VSOutput_Standard pin)
{
    // material info
    MaterialInfo matInfo  = gMaterialBuffer[gObjectCB.MatIndex];
    float4 diffuse        = matInfo.Diffuse;
    float metallic        = matInfo.Metallic;
    float roughness       = matInfo.Roughness;
    float occlusion       = 1.f;
    int diffuseMapIndex   = matInfo.DiffuseMap0Index;
    int normalMapIndex    = matInfo.NormalMapIndex;
    int metallicMapIndex  = matInfo.MetallicMapIndex;
    int emissiveMapIndex  = matInfo.EmissiveMapIndex;
    int occlusionMapIndex = matInfo.OcclusionMapIndex;
    
    // sampling diffuseMap
    if (diffuseMapIndex != -1)
    {
        diffuse *= GammaDecoding(gTextureMaps[diffuseMapIndex].Sample(gsamAnisotropicWrap, pin.UV));
    }

    // normalize normal
    pin.NormalW = normalize(pin.NormalW);
    float3 bumpedNormalW = pin.NormalW;
    
    // sampling normalMap, to world space
    float4 normalMapSample = (float4)0;
    if (normalMapIndex != -1)
    {
        normalMapSample = gTextureMaps[normalMapIndex].Sample(gsamAnisotropicWrap, pin.UV);
        bumpedNormalW = NormalSampleToWorldSpace(normalMapSample.rgb, pin.NormalW, pin.TangentW);
    }
    
    // sampling emissiveMap
    float4 emissiveMapSample = (float4)0;
    if (metallicMapIndex != -1)
    {
        emissiveMapSample = gTextureMaps[emissiveMapIndex].Sample(gsamAnisotropicWrap, pin.UV);
    }
    
    // sampling metallicMap
    float4 metallicMapSample = (float4)0;
    if (metallicMapIndex != -1)
    {
        metallicMapSample = GammaDecoding(gTextureMaps[metallicMapIndex].Sample(gsamAnisotropicWrap, pin.UV));
        metallic = metallicMapSample.r;
        roughness = 1 - metallicMapSample.a;
    }
    
    if (occlusionMapIndex != -1)
    {
        occlusion = (float)GammaDecoding(gTextureMaps[occlusionMapIndex].Sample(gsamAnisotropicWrap, pin.UV).x);
    }
    
    float3 toCameraW = normalize(gPassCB.CameraPos - pin.PosW);
    
    float rimWidth = 0.6f;
    float4 gRimLightColor = float4(1.f, 0.f, 0.f, 0.f);
    float rim = 1.0f - max(0, dot(bumpedNormalW, normalize(gPassCB.CameraPos - pin.PosW)));
    rim = smoothstep(1.0f - rimWidth, 1.0f, rim) * gObjectCB.RimFactor;
    
    float4 emissive = emissiveMapSample + gRimLightColor * rim;
    float1 ambientAcess = 1.f;
    
    float2 uvRect = float2(pin.PosH.x / gPassCB.FrameBufferWidth, pin.PosH.y / gPassCB.FrameBufferHeight);
    if (gPassCB.FilterOption & Filter_Ssao)
        ambientAcess = gTextureMaps[gPassCB.RT0S_SsaoIndex].Sample(gsamAnisotropicWrap, uvRect).r * occlusion;
    
    // 메탈릭 값을 적용
    float3 diffuseAlbedo = lerp(diffuse.xyz, 0.0f, metallic);
    float3 specularAlbedo = lerp(0.03f, diffuse.xyz, metallic);
    Material mat = { diffuseAlbedo, specularAlbedo, metallic, roughness };
    
    // 조명 계산
    float4 shadowPosH = mul(float4(pin.PosW, 1.f), gPassCB.MtxShadow);
    float shadowFactor = clamp(ComputeShadowFactor(shadowPosH), gPassCB.ShadowIntensity, 1.f);
    LightColor lightColor = ComputeDirectionalLight(gPassCB.Lights[gObjectCB.LightIndex], mat, pin.PosW, bumpedNormalW, toCameraW, shadowFactor);
    
    // specular reflection
    float3 r = reflect(-toCameraW, bumpedNormalW);
    float4 reflectionColor = gSkyBoxMaps[gPassCB.SkyBoxIndex].Sample(gsamLinearWrap, r);
    float3 fresnelFactor = SchlickFresnel(specularAlbedo, bumpedNormalW, r);
    float3 reflection = (metallic) * fresnelFactor * reflectionColor.rgb;
    
    // litColor
    float4 litDiffuse = GammaEncoding(float4(lightColor.Diffuse, 1.f));
    float4 litSpecular = GammaEncoding(float4(lightColor.Specular, 1.f)) + float4(reflection, 1.f);
    float4 litAmbient = GammaEncoding(diffuse * gPassCB.GlobalAmbient * float4(ambientAcess.xxx, 1.f)) + emissive;
    
    float4 litColor = litAmbient + litDiffuse + litSpecular;
    
    // temp
    float3 dissolveColor = float3(3.f, 1.f, 0.f);
    float4 dissolve = Dissolve(dissolveColor, gTextureMaps[gPassCB.LiveObjectDissolveIndex].Sample(gsamAnisotropicWrap, pin.UV * 2.f).x, gObjectCB.DeathElapsed);
    
    litColor.a = dissolve.a;
    litColor.rgb += dissolve.rgb;
    
    PSOutput pout = (PSOutput)0;
    pout.Result = litColor;
    pout.Normal = float4(bumpedNormalW, 1.f) * dissolve.a;
    
    return pout;
}