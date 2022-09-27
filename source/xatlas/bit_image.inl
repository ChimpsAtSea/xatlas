
class BitImage
{
public:
	BitImage() :
		m_width(0),
		m_height(0),
		m_rowStride(0)
	{
		XATLAS_VECTOR_INIT(uint64_t, data);
	}

	BitImage(uint32_t w, uint32_t h) :
		m_width(w),
		m_height(h)
	{
		m_rowStride = (m_width + 63) >> 6;
		XATLAS_VECTOR_INIT(uint64_t, data);
		XATLAS_VECTOR_RESIZE(data, m_rowStride * m_height);
	}

	BitImage(const BitImage& other) = delete;
	BitImage& operator=(const BitImage& other) = delete;
	uint32_t width() const { return m_width; }
	uint32_t height() const { return m_height; }

	void copyTo(BitImage& other)
	{
		other.m_width = m_width;
		other.m_height = m_height;
		other.m_rowStride = m_rowStride;

		XATLAS_VECTOR_RESIZE(other.data, XATLAS_VECTOR_SIZE(data));
		memcpy(other.data, data, XATLAS_VECTOR_DATASIZE(data));
	}

	void resize(uint32_t w, uint32_t h, bool discard)
	{
		const uint32_t rowStride = (w + 63) >> 6;
		if (discard)
		{
			XATLAS_VECTOR_RESIZE(data, rowStride * h);
			XATLAS_VECTOR_ZERO(data);
		}
		else
		{
			unsigned int new_data_buffer_count = rowStride * h;
			size_t new_data_buffer_bytes = new_data_buffer_count * sizeof(unsigned long long);
			unsigned long long* temp_data_buffer = (unsigned long long*)malloc(new_data_buffer_bytes);
			memset(temp_data_buffer, 0, new_data_buffer_bytes);

			// If only height has changed, can copy all rows at once.
			if (rowStride == m_rowStride) {
				memcpy(
					temp_data_buffer,
					data,
					m_rowStride * __min(m_height, h) * sizeof(uint64_t));
			}
			else if (m_width > 0 && m_height > 0) {
				const uint32_t height = __min(m_height, h);
				for (uint32_t i = 0; i < height; i++)
				{
					memcpy(
						&temp_data_buffer[i * rowStride],
						data + i * m_rowStride,
						__min(rowStride, m_rowStride) * sizeof(uint64_t));
				}
			}

			XATLAS_VECTOR_RESIZE(data, rowStride * h);
			memcpy(data, temp_data_buffer, new_data_buffer_bytes);
			free(temp_data_buffer);
		}
		m_width = w;
		m_height = h;
		m_rowStride = rowStride;
	}

	bool get(uint32_t x, uint32_t y) const
	{
		XA_DEBUG_ASSERT(x < m_width&& y < m_height);
		const uint32_t index = (x >> 6) + y * m_rowStride;
		return (data[index] & (UINT64_C(1) << (uint64_t(x) & UINT64_C(63)))) != 0;
	}

	void set(uint32_t x, uint32_t y)
	{
		XA_DEBUG_ASSERT(x < m_width&& y < m_height);
		const uint32_t index = (x >> 6) + y * m_rowStride;
		data[index] |= UINT64_C(1) << (uint64_t(x) & UINT64_C(63));
		XA_DEBUG_ASSERT(get(x, y));
	}

	void zeroOutMemory()
	{
		//m_data.zeroOutMemory();
		memset(data, 0, sizeof(*data) * XATLAS_VECTOR_SIZE(data));
	}

	bool canBlit(const BitImage& image, uint32_t offsetX, uint32_t offsetY) const
	{
		for (uint32_t y = 0; y < image.m_height; y++) {
			const uint32_t thisY = y + offsetY;
			if (thisY >= m_height)
				continue;
			uint32_t x = 0;
			for (;;) {
				const uint32_t thisX = x + offsetX;
				if (thisX >= m_width)
					break;
				const uint32_t thisBlockShift = thisX % 64;
				const uint64_t thisBlock = data[(thisX >> 6) + thisY * m_rowStride] >> thisBlockShift;
				const uint32_t blockShift = x % 64;
				const uint64_t block = image.data[(x >> 6) + y * image.m_rowStride] >> blockShift;
				if ((thisBlock & block) != 0)
					return false;
				x += 64 - __max(thisBlockShift, blockShift);
				if (x >= image.m_width)
					break;
			}
		}
		return true;
	}

	void dilate(uint32_t padding)
	{
		BitImage tmp(m_width, m_height);
		for (uint32_t p = 0; p < padding; p++) {
			tmp.zeroOutMemory();
			for (uint32_t y = 0; y < m_height; y++) {
				for (uint32_t x = 0; x < m_width; x++) {
					bool b = get(x, y);
					if (!b) {
						if (x > 0) {
							b |= get(x - 1, y);
							if (y > 0) b |= get(x - 1, y - 1);
							if (y < m_height - 1) b |= get(x - 1, y + 1);
						}
						if (y > 0) b |= get(x, y - 1);
						if (y < m_height - 1) b |= get(x, y + 1);
						if (x < m_width - 1) {
							b |= get(x + 1, y);
							if (y > 0) b |= get(x + 1, y - 1);
							if (y < m_height - 1) b |= get(x + 1, y + 1);
						}
					}
					if (b)
						tmp.set(x, y);
				}
			}
			tmp.copyTo(*this);
		}
	}

private:
	uint32_t m_width;
	uint32_t m_height;
	uint32_t m_rowStride; // In uint64_t's
	//Array<uint64_t> m_data;

	XATLAS_VECTOR_DECLARE(uint64_t, data);
};
