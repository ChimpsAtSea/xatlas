
// From Fast-BVH
class BVH
{
public:
	BVH(AABB* _object_aabbs, size_t num_object_aabbs, uint32_t leafSize = 4) :
		object_aabbs(_object_aabbs)
	{
		if (num_object_aabbs == 0)
			return;

		XATLAS_VECTOR_INIT(Node, nodes);
		XATLAS_VECTOR_INIT(unsigned int, object_ids);

		XATLAS_VECTOR_RESIZE(object_ids, num_object_aabbs);
		for (uint32_t object_id_index = 0; object_id_index < XATLAS_VECTOR_SIZE(object_ids); object_id_index++)
		{
			object_ids[object_id_index] = object_id_index;
		}

		XATLAS_VECTOR_REALLOC(nodes, num_object_aabbs * 2);

		constexpr uint32_t kRoot = 0xfffffffc;
		constexpr uint32_t kUntouched = 0xffffffff;
		constexpr uint32_t kTouchedTwice = 0xfffffffd;

		struct s_build_entry
		{
			uint32_t parent; // If non-zero then this is the index of the parent. (used in offsets)
			uint32_t start, end; // The range of objects in the object list covered by this node.
			s_build_entry* previous;
		};
		s_build_entry root;

		// Push the root
		root.start = 0;
		root.end = num_object_aabbs;
		root.parent = kRoot;

		s_build_entry* build_entry = &root;
		Node node;
		uint32_t nNodes = 0;
		do
		{
			const uint32_t start = build_entry->start;
			const uint32_t end = build_entry->end;
			const uint32_t nPrims = end - start;
			nNodes++;
			node.start = start;
			node.nPrims = nPrims;
			node.rightOffset = kUntouched;
			// Calculate the bounding box for this node
			AABB bb(object_aabbs[object_ids[start]]);
			AABB bc(object_aabbs[object_ids[start]].centroid());
			for (uint32_t p = start + 1; p < end; ++p) {
				bb.expandToInclude(object_aabbs[object_ids[p]]);
				bc.expandToInclude(object_aabbs[object_ids[p]].centroid());
			}
			node.aabb = bb;
			// If the number of primitives at this point is less than the leaf
			// size, then this will become a leaf. (Signified by rightOffset == 0)
			if (nPrims <= leafSize)
				node.rightOffset = 0;
			XATLAS_VECTOR_PUSH_BACK(nodes, node);
			// Child touches parent...
			// Special case: Don't do this for the root.
			if (build_entry->parent != kRoot) {
				nodes[build_entry->parent].rightOffset--;
				// When this is the second touch, this is the right child.
				// The right child sets up the offset for the flat tree.
				if (nodes[build_entry->parent].rightOffset == kTouchedTwice)
					nodes[build_entry->parent].rightOffset = nNodes - 1 - build_entry->parent;
			}
			// If this is a leaf, no need to subdivide.
			if (node.rightOffset == 0)
				continue;
			// Set the split dimensions
			const uint32_t split_dim = bc.maxDimension();
			// Split on the center of the longest axis
			const float split_coord = 0.5f * ((&bc.min.x)[split_dim] + (&bc.max.x)[split_dim]);
			// Partition the list of objects on this split
			uint32_t mid = start;
			for (uint32_t i = start; i < end; ++i) {
				const Vector3 centroid(object_aabbs[object_ids[i]].centroid());
				if ((&centroid.x)[split_dim] < split_coord) {
					swap(object_ids[i], object_ids[mid]);
					++mid;
				}
			}
			// If we get a bad split, just choose the center...
			if (mid == start || mid == end)
				mid = start + (end - start) / 2;


			{
				s_build_entry* right_build_entry = (s_build_entry*)alloca(sizeof(s_build_entry));

				// Push right child
				right_build_entry->start = mid;
				right_build_entry->end = end;
				right_build_entry->parent = nNodes - 1;
				right_build_entry->previous = build_entry;
				build_entry = right_build_entry;
			}
			{
				s_build_entry* left_build_entry = (s_build_entry*)alloca(sizeof(s_build_entry));

				// Push left child
				left_build_entry->start = start;
				left_build_entry->end = mid;
				left_build_entry->parent = nNodes - 1;
				left_build_entry->previous = build_entry;
				build_entry = left_build_entry;
			}

			// Pop the next item off of the stack
			build_entry = build_entry->previous;
		} while (build_entry != nullptr);
	}

	void query(const AABB& queryAabb, Array<uint32_t>& result) const
	{
		result.clear();
		// Working set
		uint32_t todo[64];
		int32_t stackptr = 0;
		// "Push" on the root node to the working set
		todo[stackptr] = 0;
		while (stackptr >= 0) {
			// Pop off the next node to work on.
			const int ni = todo[stackptr--];
			const Node& node = nodes[ni];
			// Is leaf -> Intersect
			if (node.rightOffset == 0) {
				for (uint32_t o = 0; o < node.nPrims; ++o) {
					const uint32_t obj = node.start + o;
					if (queryAabb.intersect(object_aabbs[object_ids[obj]]))
						result.push_back(object_ids[obj]);
				}
			}
			else { // Not a leaf
				const uint32_t left = ni + 1;
				const uint32_t right = ni + node.rightOffset;
				if (queryAabb.intersect(nodes[left].aabb))
					todo[++stackptr] = left;
				if (queryAabb.intersect(nodes[right].aabb))
					todo[++stackptr] = right;
			}
		}
	}

private:
	struct Node
	{
		AABB aabb;
		uint32_t start, nPrims, rightOffset;
	};

	XATLAS_VECTOR_DECLARE(Node, nodes);
	XATLAS_VECTOR_DECLARE(unsigned int, object_ids);

	const AABB* object_aabbs;
};
