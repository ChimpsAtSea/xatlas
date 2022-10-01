
// From Fast-BVH
struct AABB
{
	AABB() : min(FLT_MAX, FLT_MAX, FLT_MAX), max(-FLT_MAX, -FLT_MAX, -FLT_MAX) {}
	AABB(const Vector3& _min, const Vector3& _max) : min(_min), max(_max) { }
	AABB(const Vector3& p, float radius = 0.0f) : min(p), max(p) { if (radius > 0.0f) expand(radius); }

	bool intersect(const AABB& other) const
	{
		return min.x <= other.max.x && max.x >= other.min.x && min.y <= other.max.y && max.y >= other.min.y && min.z <= other.max.z && max.z >= other.min.z;
	}

	void expandToInclude(const Vector3& p)
	{
		min = internal::min(min, p);
		max = internal::max(max, p);
	}

	void expandToInclude(const AABB& aabb)
	{
		min = internal::min(min, aabb.min);
		max = internal::max(max, aabb.max);
	}

	void expand(float amount)
	{
		min -= Vector3(amount);
		max += Vector3(amount);
	}

	Vector3 centroid() const
	{
		return min + (max - min) * 0.5f;
	}

	uint32_t maxDimension() const
	{
		const Vector3 extent = max - min;
		uint32_t result = 0;
		if (extent.y > extent.x) {
			result = 1;
			if (extent.z > extent.y)
				result = 2;
		}
		else if (extent.z > extent.x)
			result = 2;
		return result;
	}

	Vector3 min, max;
};
