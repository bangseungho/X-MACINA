#pragma once

#pragma region ClassForwardDecl
class Transform;
#pragma endregion


#pragma region NameSpace
namespace MD3DUtil {
	// create buffer resource from data
	void CreateBufferResource(
		const void* data,
		size_t byteSize,
		D3D12_HEAP_TYPE heapType,
		D3D12_RESOURCE_STATES resourceStates,
		ComPtr<ID3D12Resource>& uploadBuffer,
		ComPtr<ID3D12Resource>& buffer);

	// upload buffer can be nullptr
	inline void CreateBufferResource(
		const void* data,
		size_t byteSize,
		D3D12_HEAP_TYPE heapType,
		D3D12_RESOURCE_STATES resourceStates,
		ID3D12Resource** uploadBuffer,
		ComPtr<ID3D12Resource>& buffer)
	{
		assert(!uploadBuffer);
		ComPtr<ID3D12Resource> tempUploadBuffer{};
		MD3DUtil::CreateBufferResource(data, byteSize, heapType, resourceStates, tempUploadBuffer, buffer);
	}

	template<class DataType>
	inline void CreateVertexBufferResource(
		const std::vector<DataType>& data,
		ComPtr<ID3D12Resource>& uploadBuffer,
		ComPtr<ID3D12Resource>& buffer)
	{
		if (data.empty()) {
			return;
		}

		MD3DUtil::CreateBufferResource(data.data(), sizeof(DataType) * data.size(), D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER, uploadBuffer, buffer);
	}

	inline void CreateIndexBufferResource(
		const std::vector<UINT>& indices,
		ComPtr<ID3D12Resource>& uploadBuffer,
		ComPtr<ID3D12Resource>& buffer)
	{
		MD3DUtil::CreateBufferResource(indices.data(), sizeof(UINT) * indices.size(), D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_INDEX_BUFFER, uploadBuffer, buffer);
	}

	template<class DataType>
	inline void CreateVertexBufferView(D3D12_VERTEX_BUFFER_VIEW& vertexBufferView, size_t vertexCount, RComPtr<ID3D12Resource> vertexBuffer)
	{
		assert(vertexBuffer);
		vertexBufferView.BufferLocation = vertexBuffer->GetGPUVirtualAddress();
		vertexBufferView.StrideInBytes = sizeof(DataType);
		vertexBufferView.SizeInBytes = (UINT)(sizeof(DataType) * vertexCount);
	}


	inline void CreateIndexBufferView(D3D12_INDEX_BUFFER_VIEW& IndexBufferView, size_t indexCount, RComPtr<ID3D12Resource> indexBuffer)
	{
		assert(indexBuffer);
		IndexBufferView.BufferLocation = indexBuffer->GetGPUVirtualAddress();
		IndexBufferView.Format = DXGI_FORMAT_R32_UINT;
		IndexBufferView.SizeInBytes = (UINT)(sizeof(UINT) * indexCount);
	}


	void CreateTextureResourceFromDDSFile(
		const std::wstring& fileName,
		ComPtr<ID3D12Resource>& uploadBuffer,
		ComPtr<ID3D12Resource>& texture,
		D3D12_RESOURCE_STATES resourceStates = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

	ComPtr<ID3D12Resource> CreateTexture2DResource(
		UINT width,
		UINT height,
		UINT elements,
		UINT mipLevels,
		DXGI_FORMAT dxgiFormat,
		D3D12_RESOURCE_FLAGS resourceFlags,
		D3D12_RESOURCE_STATES resourceStates,
		Vec4 clearValue);

	D3D12_RESOURCE_BARRIER ResourceTransition(
		RComPtr<ID3D12Resource> resource,
		D3D12_RESOURCE_STATES stateBefore,
		D3D12_RESOURCE_STATES stateAfter);

	D3D12_SHADER_BYTECODE CompileShaderFile(
		const std::wstring& fileName,
		LPCSTR shaderName,
		LPCSTR shaderProfile,
		ComPtr<ID3DBlob>& shaderBlob);

	ComPtr<ID3DBlob> ReadCompiledShaderFile(
		const std::string& fileName);

	inline UINT CalcConstantBuffSize(UINT byteSize)
	{
		// ��� ������ ũ��� �ϵ������ �ּ� �޸� �Ҵ� ũ�⿡ ���(256���)�� �Ǿ�� �Ѵ�.
		// ����, 255�� ���ϰ� 256���� ���� ��� ��Ʈ�� �����Ѵ�.
		return (byteSize + 255) & ~255;
	}
}


// �������� ��� ���۸� �� ���� �����ϱ� ���� Ŭ����
// ���簡 ���� �ʾƾ� �Ѵ�.
template<typename T>
class UploadBuffer  {
private:
    ComPtr<ID3D12Resource>  mUploadBuffer{};
    BYTE* mMappedData{};

    int     mElementCount{};
    size_t  mElementByteSize{};
    bool    mIsConstantBuffer{};

public:
    // mElementByteSize * mElementCount ��ŭ�� ���۸� ����Ѵ�.
    UploadBuffer(ID3D12Device* pDevice, int elementCount, bool isConstantBuffer)
        :
        mIsConstantBuffer(isConstantBuffer),
        mElementCount(elementCount),
        mElementByteSize(sizeof(T))
    {
        // ��� ���۷� ����� ��� 256�� ����� �ǵ��� �Ѵ�.
        if (isConstantBuffer)
            mElementByteSize = MD3DUtil::CalcConstantBuffSize(sizeof(T));
        
        D3D12_HEAP_PROPERTIES heapProperties{};
        heapProperties.Type = D3D12_HEAP_TYPE_UPLOAD;
        heapProperties.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
        heapProperties.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
        heapProperties.CreationNodeMask = 1;
        heapProperties.VisibleNodeMask = 1;

        D3D12_RESOURCE_DESC resourceDesc{};
        resourceDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
        resourceDesc.Alignment = 0;
        resourceDesc.Width = static_cast<UINT64>(mElementByteSize) * mElementCount;
        resourceDesc.Height = 1;
        resourceDesc.DepthOrArraySize = 1;
        resourceDesc.MipLevels = 1;
        resourceDesc.Format = DXGI_FORMAT_UNKNOWN;
        resourceDesc.SampleDesc.Count = 1;
        resourceDesc.SampleDesc.Quality = 0;
        resourceDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
        resourceDesc.Flags = D3D12_RESOURCE_FLAG_NONE;
        
        pDevice->CreateCommittedResource(
            &heapProperties,
            D3D12_HEAP_FLAG_NONE,
            &resourceDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&mUploadBuffer));

        mUploadBuffer->Map(0, nullptr, reinterpret_cast<void**>(&mMappedData));
    }

    // �Ҹ��� ȣ��� ���ε� ���� ���� ����
    ~UploadBuffer() {
        if (mUploadBuffer != nullptr)
            mUploadBuffer->Unmap(0, nullptr);

        mMappedData = nullptr;
    }

public:
    const size_t GetElementByteSize() const {
        return mElementByteSize;
    }

    ID3D12Resource* Resource()const {
        return mUploadBuffer.Get();
    }

    // ���ε� �޸𸮿� �����͸� �����ϴ� �Լ�
    void CopyData(int elementIndex, const T& data) {
        memcpy(&mMappedData[elementIndex * mElementByteSize], &data, sizeof(T));
    }
};



