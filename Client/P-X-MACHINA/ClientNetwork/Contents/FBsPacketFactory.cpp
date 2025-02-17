#include "stdafx.h"
#include "FBsPacketFactory.h"
#include "ClientNetwork/Include/PacketHeader.h"

/* RELATED FLAT BUFFERS HEADER */
#undef max
#include "ClientNetwork/Include/Protocol/Enum_generated.h"
#include "ClientNetwork/Include/Protocol/FBProtocol_generated.h"
#include "ClientNetwork/Include/Protocol/Struct_generated.h"
#include "ClientNetwork/Include/Protocol/Transform_generated.h"

#include "ClientNetwork/Include/SendBuffersFactory.h"
#include "ClientNetwork/Include/SocketData.h"

#include "ClientNetwork/Contents/Script_PlayerNetwork.h"
#include "ClientNetwork/Contents/Script_RemotePlayer.h"

#include "ServerSession.h"
#include "NetworkEvents.h"
#include "ClientNetworkManager.h"
#include "GameFramework.h"

DEFINE_SINGLETON(FBsPacketFactory);

std::atomic<long long> FBsPacketFactory::TotalLatency = 0;
std::atomic<int> FBsPacketFactory::LatencyCount       = 0;
std::atomic_int FBsPacketFactory::CurrLatency         = 0;



/// ★---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
///	◈ PROCESS SERVER PACKET ◈
/// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------★
bool FBsPacketFactory::ProcessFBsPacket(SPtr_Session session, BYTE* packetBuf, UINT32 Datalen)
{

	/* ▶ Packet Struct ------------------------------------------------- */
	/* [PacketHeader(ProtocolID, PktSize)][DATA-(FlatBuffers Serialized)] */
	/* ------------------------------------------------------------------ */

	PacketHeader* Head  = reinterpret_cast<PacketHeader*>(packetBuf);
	const void* DataPtr = packetBuf + sizeof(PacketHeader);
	switch (Head->ProtocolID)
	{	
	case FBsProtocolID::SPkt_LogIn:
	{
		LOG_MGR->Cout(session->GetID(), " - RECV - ", "[ SPkt_LogIn ]\n");
		const FBProtocol::SPkt_LogIn* packet = flatbuffers::GetRoot<FBProtocol::SPkt_LogIn>(DataPtr);
		if (!packet) return false;
		Process_SPkt_LogIn(session, *packet);
	}
	break;
	case FBsProtocolID::SPkt_EnterGame:
	{
		LOG_MGR->Cout(session->GetID(), " - RECV - ", "[ SPkt_EnterGame ]\n");

		const FBProtocol::SPkt_EnterGame* packet = flatbuffers::GetRoot<FBProtocol::SPkt_EnterGame>(DataPtr);
		if (!packet) return false;
		Process_SPkt_EnterGame(session, *packet);
	}
	break;
	case FBsProtocolID::SPkt_Chat:
	{
		LOG_MGR->Cout(session->GetID(), " - RECV - ", "[ SPkt_Chat ]\n");

		const FBProtocol::SPkt_Chat* packet = flatbuffers::GetRoot<FBProtocol::SPkt_Chat>(DataPtr);
		if (!packet) return false;
		Process_SPkt_Chat(session, *packet);
	}
	break;
	case FBsProtocolID::SPkt_NetworkLatency:
	{
		//LOG_MGR->Cout(session->GetID(), " - RECV - ", "[ SPkt_NetworkLatency ]\n");

		const FBProtocol::SPkt_NetworkLatency* packet = flatbuffers::GetRoot<FBProtocol::SPkt_NetworkLatency>(DataPtr);
		if (!packet) return false;
		Process_SPkt_NetworkLatency(session, *packet);
	}
	break;
	case FBsProtocolID::SPkt_Transform:
	{
		//LOG_MGR->Cout(session->GetID(), " - RECV - ", "[ SPkt_Transform ]\n");
		
		const FBProtocol::SPkt_Transform* packet = flatbuffers::GetRoot<FBProtocol::SPkt_Transform>(DataPtr);
		if (!packet) return false;
		Process_SPkt_Transform(session, *packet);
	}
	break;
	case FBsProtocolID::SPkt_NewPlayer:
	{
		LOG_MGR->Cout(session->GetID(), " - RECV - ", "[ SPkt_NewPlayer ]\n");

		const FBProtocol::SPkt_NewPlayer* packet = flatbuffers::GetRoot<FBProtocol::SPkt_NewPlayer>(DataPtr);
		if (!packet) return false;
		Process_SPkt_NewPlayer(session, *packet);
	}
	break;
	case FBsProtocolID::SPkt_RemovePlayer:
	{
		LOG_MGR->Cout(session->GetID(), " - RECV - ", "[ SPkt_RemovePlayer ]\n");

		const FBProtocol::SPkt_RemovePlayer* packet = flatbuffers::GetRoot<FBProtocol::SPkt_RemovePlayer>(DataPtr);
		if (!packet) return false;
		Process_SPkt_RemovePlayer(session, *packet);
	}
	break;
	case FBsProtocolID::SPkt_PlayerAnimation:
	{
		const FBProtocol::SPkt_PlayerAnimation* packet = flatbuffers::GetRoot<FBProtocol::SPkt_PlayerAnimation>(DataPtr);
		if (!packet) return false;
		Process_SPkt_PlayerAnimation(session, *packet);

	}
	break;

	}

	return true;
}

bool FBsPacketFactory::Process_SPkt_Invalid(SPtr_Session session, BYTE* packetBuf, UINT32 Datalen)
{
	return false;
}

bool FBsPacketFactory::Process_SPkt_Chat(SPtr_Session session, const FBProtocol::SPkt_Chat& pkt)
{
	std::cout << "SPkt CHAT [" << session->GetID() << "] - SESSION : " << session.get() << " DATA : " <<
		pkt.message()->c_str() << std::endl;
	return true;
}

bool FBsPacketFactory::Process_SPkt_PlayerAnimation(SPtr_Session session, const FBProtocol::SPkt_PlayerAnimation& pkt)
{
	int ObjectID                = pkt.object_id();
	int32_t animation_upper_idx = pkt.animation_upper_index();
	int32_t animation_lower_idx = pkt.animation_lower_index();
	int32_t animation_param_h   = pkt.animation_param_h();
	int32_t animation_param_v   = pkt.animation_param_v();
	
	sptr<NetworkEvent::Game::ChangeAnimation_RemotePlayer> EventData = CLIENT_NETWORK->CreateEvent_ChangeAnimation_RemotePlayer(ObjectID
																																, animation_upper_idx
																																, animation_lower_idx
																																, animation_param_h
																																, animation_param_v);
	CLIENT_NETWORK->RegisterEvent(EventData);

	return false;
}

bool FBsPacketFactory::Process_SPkt_NetworkLatency(SPtr_Session session, const FBProtocol::SPkt_NetworkLatency& pkt)
{
	// 패킷으로부터 long long으로 시간을 받음
	long long timestamp = pkt.timestamp();

	
	// 현재 시간 구하기
	auto currentTime = CLIENT_NETWORK->GetCurrentTimeMilliseconds();

	// 패킷의 타임스탬프와 현재 시간의 차이를 구하기
	auto latency = currentTime - timestamp;
	latency = latency / 2;

	TotalLatency += latency;
	LatencyCount += 1;

	// 변환된 값을 출력
	if (LatencyCount.load() >= 5) {
		CurrLatency.store(TotalLatency.load() / LatencyCount.load()); /* Latency 의 평균 저장 */
		//std::cout << std::chrono::milliseconds(CurrLatency.load()) << '\n';

		LatencyCount.store(0);
		TotalLatency.store(0);
	}

	return true;
}

bool FBsPacketFactory::Process_SPkt_LogIn(SPtr_Session session, const FBProtocol::SPkt_LogIn& pkt)
{
#ifdef CONNECT_WITH_TEST_CLIENT
	return true;
#endif

	GamePlayerInfo MyInfo = GetPlayerInfo(pkt.myinfo());
	GameFramework::I->InitPlayer(static_cast<int>(MyInfo.Id)); /* INIT PLAYER */


	LOG_MGR->SetColor(TextColor::BrightGreen);
	LOG_MGR->Cout("♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠\n");
	LOG_MGR->Cout("[MY] NAME : " , MyInfo.Name, " " , " SESSION ID : ", MyInfo.Id , '\n');
	LOG_MGR->Cout("♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠♠\n");
	LOG_MGR->SetColor(TextColor::Default);

	int PlayersCnt = pkt.players()->size();
	for (UINT16 i = 0; i < PlayersCnt; ++i) {
		GamePlayerInfo RemoteInfo = GetPlayerInfo(pkt.players()->Get(i));

		if (RemoteInfo.Id == MyInfo.Id) continue;
		LOG_MGR->SetColor(TextColor::BrightGreen);
		LOG_MGR->Cout("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■\n");
		LOG_MGR->Cout("[REMOTE] NAME : ", RemoteInfo.Name, " ", " SESSION ID : ", RemoteInfo.Id, '\n');
		LOG_MGR->Cout("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■\n");
		LOG_MGR->SetColor(TextColor::Default);

		sptr<NetworkEvent::Game::Add_RemotePlayer> EventData = CLIENT_NETWORK->CreateEvent_Add_RemotePlayer(RemoteInfo);
		CLIENT_NETWORK->RegisterEvent(EventData);
	}

	auto CPkt = FBS_FACTORY->CPkt_EnterGame(0); /* 0 : 의미없음 */
	session->Send(CPkt);

	return true;
}

bool FBsPacketFactory::Process_SPkt_EnterGame(SPtr_Session session, const FBProtocol::SPkt_EnterGame& pkt)
{
	

	return true;
}

bool FBsPacketFactory::Process_SPkt_NewPlayer(SPtr_Session session, const FBProtocol::SPkt_NewPlayer& pkt)
{
	GamePlayerInfo NewPInfo = GetPlayerInfo(pkt.newplayer());

	LOG_MGR->SetColor(TextColor::BrightGreen);
	LOG_MGR->Cout("▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣\n");
	LOG_MGR->Cout("[NEW REMOTE] NAME : ", NewPInfo.Name, " ", " SESSION ID : ", NewPInfo.Id, '\n');
	LOG_MGR->Cout("▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣▣\n");
	LOG_MGR->SetColor(TextColor::Default);

	sptr<NetworkEvent::Game::Add_RemotePlayer> EventData = CLIENT_NETWORK->CreateEvent_Add_RemotePlayer(NewPInfo);
	CLIENT_NETWORK->RegisterEvent(EventData);

	return true;
}

bool FBsPacketFactory::Process_SPkt_RemovePlayer(SPtr_Session session, const FBProtocol::SPkt_RemovePlayer& pkt)
{
	int32_t removeID = pkt.playerid();

	LOG_MGR->SetColor(TextColor::BrightRed);
	LOG_MGR->Cout("▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷\n");
	LOG_MGR->Cout("[REMOVE REMOTE] NAME SESSION ID : ", removeID, '\n');
	LOG_MGR->Cout("▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷▷\n");
	LOG_MGR->SetColor(TextColor::Default);

	sptr<NetworkEvent::Game::Remove_RemotePlayer> EventData = CLIENT_NETWORK->CreateEvent_Remove_RemotePlayer(removeID);
	CLIENT_NETWORK->RegisterEvent(EventData);

	return true;
}

bool FBsPacketFactory::Process_SPkt_Transform(SPtr_Session session, const FBProtocol::SPkt_Transform& pkt)
{
	long long latency   = pkt.latency();
	uint64_t id         = pkt.object_id();
	
	
	float	vel         = pkt.velocity();
	Vec3	moveDir		= GetVector3(pkt.movedir());
	Vec3	Packetpos	= GetVector3(pkt.trans()->position());
	Vec3	rot         = GetVector3(pkt.trans()->rotation());
	int32_t movestate   = pkt.move_state(); 
	Vec3	SDir        = GetVector3(pkt.spine_look());

	float   animparam_h   = pkt.animparam_h();
	float   animparam_v   = pkt.animparam_v();
	
	ExtData::MOVESTATE mState;
	if (movestate == PLAYER_MOVE_STATE::Start)			mState = ExtData::MOVESTATE::Start;
	else if (movestate == PLAYER_MOVE_STATE::Progress)	mState = ExtData::MOVESTATE::Progress;
	else if (movestate == PLAYER_MOVE_STATE::End)		mState = ExtData::MOVESTATE::End;


	sptr<NetworkEvent::Game::Move_RemotePlayer> Move_EventData = CLIENT_NETWORK->CreateEvent_Move_RemotePlayer(id, Packetpos, mState);
	CLIENT_NETWORK->RegisterEvent(Move_EventData);
	
	//LOG_MGR->Cout("MOVE DIR PKT : ", moveDir.x, " ", moveDir.y, " ", moveDir.z, '\n');

	/// +---------------------------
	///	Extrapolate Nexy Packet Pos 
	/// ---------------------------+

	/* CurrPos --------------- PacketPos --------------------------> TargetPos */

	ExtData data  = {};
	/* [Get Next Packet Duration] = (PKt Interval) + (Remote Cl Latency) + (My Latency) */
	data.PingTime = (PlayerNetworkInfo::SendInterval_CPkt_Trnasform * 1000) + (latency / 1000.0) + (CurrLatency.load() / 1000.0); 
	data.MoveDir  = moveDir;
	data.MoveState = mState;
	//LOG_MGR->Cout(data.PingTime, " ", data.PingTime / 1000.0, '\n');

	/* 위치 예측 ( TargetPos ) */
	data.TargetPos.x    = Packetpos.x + (data.MoveDir.x * vel * ((data.PingTime) / 1000.0));
	data.TargetPos.z    = Packetpos.z + (data.MoveDir.z * vel * ((data.PingTime) / 1000.0));
	
	data.TargetRot	  = rot;
	data.Velocity     = vel;

	data.Animdata.AnimParam_h = animparam_h;
	data.Animdata.AnimParam_v = animparam_v;

	sptr<NetworkEvent::Game::Extrapolate_RemotePlayer> Ext_EventData = CLIENT_NETWORK->CreateEvent_Extrapolate_RemotePlayer(id, data);
	CLIENT_NETWORK->RegisterEvent(Ext_EventData);


	return true;
}

bool FBsPacketFactory::Process_SPkt_KeyInput(SPtr_Session session, const FBProtocol::SPkt_KeyInput& pkt)
{
	return true;
}

bool FBsPacketFactory::Process_SPkt_PlayerState(SPtr_Session session, const FBProtocol::SPkt_PlayerState& pkt)
{
	return true;
}

/// ★---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
///	◈ CREATE FLATBUFFERS SERVER PACKET ◈
/// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------★
SPtr_SendPktBuf FBsPacketFactory::CPkt_Chat(UINT32 sessionID, std::string msg)
{
	flatbuffers::FlatBufferBuilder builder;

	auto msgOffset = builder.CreateString(msg);
	auto ServerPacket = FBProtocol::CreateCPkt_Chat(builder, msgOffset);

	builder.Finish(ServerPacket);

	const uint8_t* bufferPtr = builder.GetBufferPointer();
	const uint16_t serializedDataSize = static_cast<uint16_t>(builder.GetSize());

	return SENDBUF_FACTORY->CreatePacket(bufferPtr, serializedDataSize, FBsProtocolID::CPkt_Chat);
}

SPtr_SendPktBuf FBsPacketFactory::CPkt_NewtorkLatency(long long timestamp)
{
	flatbuffers::FlatBufferBuilder builder;

	auto ServerPacket = FBProtocol::CreateCPkt_NetworkLatency(builder, timestamp);

	builder.Finish(ServerPacket);

	const uint8_t* bufferPtr = builder.GetBufferPointer();
	const uint16_t serializedDataSize = static_cast<uint16_t>(builder.GetSize());

	return SENDBUF_FACTORY->CreatePacket(bufferPtr, serializedDataSize, FBsProtocolID::CPkt_NetworkLatency);
}

SPtr_SendPktBuf FBsPacketFactory::CPkt_LogIn()
{
	flatbuffers::FlatBufferBuilder builder;

	/* CREATE LOG IN PACKET */
	auto ServerPacket = FBProtocol::CreateCPkt_LogIn(builder);
	builder.Finish(ServerPacket);

	/* Create SendBuffer */
	const uint8_t* bufferPointer = builder.GetBufferPointer();
	const uint16_t SerializeddataSize = static_cast<uint16_t>(builder.GetSize());;

	return SENDBUF_FACTORY->CreatePacket(bufferPointer, SerializeddataSize, FBsProtocolID::CPkt_LogIn);
}

SPtr_SendPktBuf FBsPacketFactory::CPkt_EnterGame(uint64_t playerIdx)
{
	flatbuffers::FlatBufferBuilder builder;

	auto enterGamePkt = FBProtocol::CreateCPkt_EnterGame(builder, playerIdx);
	builder.Finish(enterGamePkt);

	const uint8_t* bufferPointer = builder.GetBufferPointer();
	const uint16_t SerializeddataSize = static_cast<uint16_t>(builder.GetSize());;

	return SENDBUF_FACTORY->CreatePacket(bufferPointer, SerializeddataSize, FBsProtocolID::CPkt_EnterGame);

}

SPtr_SendPktBuf FBsPacketFactory::CPkt_NewPlayer()
{
	flatbuffers::FlatBufferBuilder builder{};

	auto ServerPacket = FBProtocol::CreateCPkt_NewPlayer(builder);
	builder.Finish(ServerPacket);

	const uint8_t* bufferPointer = builder.GetBufferPointer();
	const uint16_t SerializeddataSize = static_cast<uint16_t>(builder.GetSize());;
	SPtr_SendPktBuf sendBuffer = SENDBUF_FACTORY->CreatePacket(bufferPointer, SerializeddataSize, FBsProtocolID::CPkt_NewPlayer);

	return sendBuffer;
}

SPtr_SendPktBuf FBsPacketFactory::CPkt_RemovePlayer(int removeSessionID)
{
	flatbuffers::FlatBufferBuilder builder{};

	int32_t id        = static_cast<int32_t>(removeSessionID);
	auto ServerPacket = FBProtocol::CreateCPkt_RemovePlayer(builder, id);
	builder.Finish(ServerPacket);

	const uint8_t* bufferPointer      = builder.GetBufferPointer();
	const uint16_t SerializeddataSize = static_cast<uint16_t>(builder.GetSize());;
	SPtr_SendPktBuf sendBuffer        = SENDBUF_FACTORY->CreatePacket(bufferPointer, SerializeddataSize, FBsProtocolID::CPkt_RemovePlayer);

	return sendBuffer;
}

SPtr_SendPktBuf FBsPacketFactory::CPkt_KeyInput(GameKeyInfo::KEY key, GameKeyInfo::KEY_STATE KeyState, GameKeyInfo::MoveKey moveKey, Vec2 mouseDelta)
{
	flatbuffers::FlatBufferBuilder builder{};

	uint8_t keyinput_state       = static_cast<uint8_t>(KeyState);
	uint8_t keyinput             = static_cast<uint8_t>(key);
	uint8_t keyinput_move        = static_cast<uint8_t>(moveKey);
	auto    mouse_delta          = FBProtocol::CreateVector2(builder, mouseDelta.x, mouseDelta.y);

	auto ServerPacket = FBProtocol::CreateCPkt_KeyInput(builder, keyinput_state, keyinput, keyinput_move, mouse_delta);
	builder.Finish(ServerPacket);

	const uint8_t* bufferPointer = builder.GetBufferPointer();
	const uint16_t SerializeddataSize = static_cast<uint16_t>(builder.GetSize());;
	SPtr_SendPktBuf sendBuffer = SENDBUF_FACTORY->CreatePacket(bufferPointer, SerializeddataSize, FBsProtocolID::CPkt_KeyInput);

	return sendBuffer;
}

SPtr_SendPktBuf FBsPacketFactory::CPkt_Transform(Vec3 Pos, Vec3 Rot, int32_t movestate, Vec3 movedir, float velocity, Vec3 SpineLookDir, long long latency, float animparam_h, float animparam_v)
{
	flatbuffers::FlatBufferBuilder builder{};

	auto moveDir	   = FBProtocol::CreateVector3(builder, movedir.x, movedir.y, movedir.z);
	auto position      = FBProtocol::CreateVector3(builder, Pos.x, Pos.y, Pos.z);
	auto rotation      = FBProtocol::CreateVector3(builder, Rot.x, Rot.y, Rot.z);
	auto transform     = FBProtocol::CreateTransform(builder, position, rotation);
	auto Spine_LookDir = FBProtocol::CreateVector3(builder, SpineLookDir.x, SpineLookDir.y, SpineLookDir.z);

	auto ServerPacket = FBProtocol::CreateCPkt_Transform(builder, latency, movestate, velocity, moveDir, transform, Spine_LookDir, animparam_h, animparam_v);
	builder.Finish(ServerPacket);


	const uint8_t* bufferPointer      = builder.GetBufferPointer();
	const uint16_t SerializeddataSize = static_cast<uint16_t>(builder.GetSize());;
	SPtr_SendPktBuf sendBuffer        = SENDBUF_FACTORY->CreatePacket(bufferPointer, SerializeddataSize, FBsProtocolID::CPkt_Transform);
	return sendBuffer;
}

SPtr_SendPktBuf FBsPacketFactory::CPkt_PlayerAnimation(int anim_upper_idx, int anim_lower_idx, float anim_param_h, float anim_param_v)
{
	flatbuffers::FlatBufferBuilder builder{};

	int32_t animation_upper_index = static_cast<int32_t>(anim_upper_idx);
	int32_t animation_lower_index = static_cast<int32_t>(anim_lower_idx);
	int32_t animation_param_h     = static_cast<int32_t>(anim_param_h);
	int32_t animation_param_v     = static_cast<int32_t>(anim_param_v);

	auto ServerPacket = FBProtocol::CreateCPkt_PlayerAnimation(builder, animation_upper_index, animation_lower_index, animation_param_h, animation_param_v);
	builder.Finish(ServerPacket);

	const uint8_t* bufferPointer      = builder.GetBufferPointer();
	const uint16_t SerializeddataSize = static_cast<uint16_t>(builder.GetSize());;
	SPtr_SendPktBuf sendBuffer        = SENDBUF_FACTORY->CreatePacket(bufferPointer, SerializeddataSize, FBsProtocolID::CPkt_PlayerAnimation);
	return sendBuffer;
}

GamePlayerInfo FBsPacketFactory::GetPlayerInfo(const FBProtocol::Player* player)
{
	GamePlayerInfo info = {};

	info.Id   = player->id();
	info.Name = player->name()->c_str();
	info.Type = player->player_type();


	const FBProtocol::Vector3* pos  = player->trans()->position();
	info.Pos = Vec3(pos->x(), pos->y(), pos->z());

	const FBProtocol::Vector3* Rot  = player->trans()->rotation();
	info.Rot = Vec3(Rot->x(), Rot->y(), Rot->z());

	const FBProtocol::Vector3* SDir = player->spine_look();
	info.SDir = Vec3(SDir->x(), SDir->y(), SDir->z());

	return info;
}

Vec3 FBsPacketFactory::GetVector3(const FBProtocol::Vector3* vec3)
{
	Vec3 Vector3 = Vec3(vec3->x(), vec3->y(), vec3->z());

	return Vector3;
}

Vec4 FBsPacketFactory::GetVector4(const FBProtocol::Vector4* vec4)
{
	Vec4 Vector4 = Vec4(vec4->x(), vec4->y(), vec4->z(), vec4->w());

	return Vector4;
}




Vec3 FBsPacketFactory::CalculateDirection(float yAngleRadian)
{
	// x 및 z 방향 벡터 계산
	float xDir = std::sin(yAngleRadian);
	float zDir = std::cos(yAngleRadian);

	Vec3 dir = Vec3(xDir, 0.0f, zDir); // y 방향은 고려하지 않음
	dir.Normalize();
	return dir;
}

Vec3 FBsPacketFactory::lerp(Vec3 CurrPos, Vec3 TargetPos, float PosLerpParam)
{
	return Vec3((1.0f - PosLerpParam) * CurrPos.x + PosLerpParam * TargetPos.x,
		(1.0f - PosLerpParam) * CurrPos.y + PosLerpParam * TargetPos.y,
		(1.0f - PosLerpParam) * CurrPos.z + PosLerpParam * TargetPos.z);
}

