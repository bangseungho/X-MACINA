#include "Common.hlsl"

struct VSOutput_Particle
{
    float3 PosW : POSITION;
    uint   ID   : ID;
};

struct GSOutput_Particle
{
    float4 PosH : SV_POSITION;
    float2 UV   : UV;
    uint   ID   : ID;
};

[maxvertexcount(6)]
void GSStretchedParticle(point VSOutput_Particle gin[1], inout TriangleStream<GSOutput_Particle> triStream)
{
    GSOutput_Particle gout[4] =
    {
        (GSOutput_Particle)0.f, 
        (GSOutput_Particle)0.f, 
        (GSOutput_Particle)0.f, 
        (GSOutput_Particle)0.f
    };
    
    uint id = (uint)gin[0].ID;
    if (gInputPraticles[id].Alive == 0)
        return;
    
    float3 up = gInputPraticles[id].MoveDir;
    float3 look = normalize(gPassCB.CameraPos - gin[0].PosW);
    float3 right = cross(up, look);
    
    RotationAxis(right, up, look, gInputPraticles[id].StartRotation);
    
    float halfWidth = gInputPraticles[id].StartSize.x / 2.f;
    float halfHeight = gInputPraticles[id].StartSize.y / 2.f;
    
    float4 posW[4];
    posW[0] = float4(gin[0].PosW + halfHeight * up - halfWidth * right + halfHeight * up, 1.f);
    posW[1] = float4(gin[0].PosW + halfHeight * up + halfWidth * right + halfHeight * up, 1.f);
    posW[2] = float4(gin[0].PosW + halfHeight * up - halfWidth * right - halfHeight * up, 1.f);
    posW[3] = float4(gin[0].PosW + halfHeight * up + halfWidth * right - halfHeight * up, 1.f);
    
    gout[0].PosH = mul(mul(posW[0], gPassCB.MtxView), gPassCB.MtxProj);
    gout[1].PosH = mul(mul(posW[1], gPassCB.MtxView), gPassCB.MtxProj);
    gout[2].PosH = mul(mul(posW[2], gPassCB.MtxView), gPassCB.MtxProj);
    gout[3].PosH = mul(mul(posW[3], gPassCB.MtxView), gPassCB.MtxProj);
    
    gout[0].UV = float2(0.f, 0.f);
    gout[1].UV = float2(1.f, 0.f);
    gout[2].UV = float2(0.f, 1.f);
    gout[3].UV = float2(1.f, 1.f);
    
    gout[0].ID = gin[0].ID;
    gout[1].ID = gin[0].ID;
    gout[2].ID = gin[0].ID;
    gout[3].ID = gin[0].ID;
    
    triStream.Append(gout[0]);
    triStream.Append(gout[1]);
    triStream.Append(gout[2]);
    
    triStream.Append(gout[1]);
    triStream.Append(gout[2]);
    triStream.Append(gout[3]);
}