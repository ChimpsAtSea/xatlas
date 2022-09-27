
class Mesh
{
public:
	Mesh(
		float epsilon, 
		uint32_t approxVertexCount, 
		uint32_t approxFaceCount, 
		uint32_t flags = 0, 
		uint32_t id = UINT32_MAX) : 
		m_epsilon(epsilon), 
		m_flags(flags), 
		m_id(id), 
		//_face_ignore(MemTag::Mesh), 
		//_face_materials(MemTag::Mesh),
		//_indices(MemTag::MeshIndices),
		//_positions(MemTag::MeshPositions),
		//_normals(MemTag::MeshNormals), 
		//_texcoords(MemTag::MeshTexcoords), 
		m_nextColocalVertex(MemTag::MeshColocals), 
		m_firstColocalVertex(MemTag::MeshColocals), 
		m_boundaryEdges(MemTag::MeshBoundaries),
		m_oppositeEdges(MemTag::MeshBoundaries), 
		m_edgeMap(MemTag::MeshEdgeMap, approxFaceCount * 3)
	{
		XATLAS_VECTOR_INIT(bool, _face_ignore);
		XATLAS_VECTOR_INIT(uint32_t, _face_materials);
		XATLAS_VECTOR_INIT(uint32_t, _indices);
		XATLAS_VECTOR_INIT(Vector3, _positions);
		XATLAS_VECTOR_INIT(Vector3, _normals);
		XATLAS_VECTOR_INIT(Vector2, _texcoords);

		XATLAS_VECTOR_REALLOC(_indices, approxFaceCount * 3);
		XATLAS_VECTOR_REALLOC(_positions, approxVertexCount);
		XATLAS_VECTOR_REALLOC(_texcoords, approxVertexCount);
		if (m_flags & MeshFlags::HasIgnoredFaces)
		{
			XATLAS_VECTOR_REALLOC(_face_ignore, approxVertexCount);
		}
		if (m_flags & MeshFlags::HasNormals)
		{
			XATLAS_VECTOR_REALLOC(_normals, approxVertexCount);
		}
		if (m_flags & MeshFlags::HasMaterials)
		{
			XATLAS_VECTOR_REALLOC(_face_materials, approxVertexCount);
		}
	}

	uint32_t flags() const { return m_flags; }
	uint32_t id() const { return m_id; }

	void addVertex(const Vector3& pos, const Vector3& normal = Vector3(0.0f), const Vector2& texcoord = Vector2(0.0f))
	{
		XA_DEBUG_ASSERT(isfinite(pos));
		XATLAS_VECTOR_PUSH_BACK(_positions, pos);
		if (m_flags & MeshFlags::HasNormals)
		{
			XATLAS_VECTOR_PUSH_BACK(_normals, normal);
		}
		XATLAS_VECTOR_PUSH_BACK(_texcoords, texcoord);
	}

	void addFace(const uint32_t* indices, bool ignore = false, uint32_t material = UINT32_MAX)
	{
		if (m_flags & MeshFlags::HasIgnoredFaces)
		{
			XATLAS_VECTOR_PUSH_BACK(_face_ignore, ignore);
		}
		if (m_flags & MeshFlags::HasMaterials)
		{
			XATLAS_VECTOR_PUSH_BACK(_face_materials, material);
		}
		const uint32_t firstIndex = XATLAS_VECTOR_SIZE(_indices);
		for (uint32_t i = 0; i < 3; i++)
		{
			XATLAS_VECTOR_PUSH_BACK(_indices, indices[i]);
		}
		for (uint32_t i = 0; i < 3; i++) 
		{
			const uint32_t vertex0 = _indices[firstIndex + i];
			const uint32_t vertex1 = _indices[firstIndex + (i + 1) % 3];
			m_edgeMap.add(EdgeKey(vertex0, vertex1));
		}
	}

	void createColocalsBVH()
	{
		const uint32_t vertexCount = XATLAS_VECTOR_SIZE(_positions);
		Array<AABB> aabbs(MemTag::BVH);
		aabbs.resize(vertexCount);
		for (uint32_t i = 0; i < XATLAS_VECTOR_SIZE(_positions); i++)
			aabbs[i] = AABB(_positions[i], m_epsilon);
		BVH bvh(aabbs.data(), aabbs.size());
		Array<uint32_t> colocals(MemTag::MeshColocals);
		Array<uint32_t> potential(MemTag::MeshColocals);
		m_nextColocalVertex.resize(vertexCount);
		m_nextColocalVertex.fillBytes(0xff);
		m_firstColocalVertex.resize(vertexCount);
		m_firstColocalVertex.fillBytes(0xff);
		for (uint32_t i = 0; i < vertexCount; i++) {
			if (m_nextColocalVertex[i] != UINT32_MAX)
				continue; // Already linked.
			// Find other vertices colocal to this one.
			colocals.clear();
			colocals.push_back(i); // Always add this vertex.
			bvh.query(AABB(_positions[i], m_epsilon), potential);
			for (uint32_t j = 0; j < potential.size(); j++) {
				const uint32_t otherVertex = potential[j];
				if (otherVertex != i && equal(_positions[i], _positions[otherVertex], m_epsilon) && m_nextColocalVertex[otherVertex] == UINT32_MAX)
					colocals.push_back(otherVertex);
			}
			if (colocals.size() == 1) {
				// No colocals for this vertex.
				m_nextColocalVertex[i] = i;
				m_firstColocalVertex[i] = i;
				continue;
			}
			// Link in ascending order.
			insertionSort(colocals.data(), colocals.size());
			for (uint32_t j = 0; j < colocals.size(); j++) {
				m_nextColocalVertex[colocals[j]] = colocals[(j + 1) % colocals.size()];
				m_firstColocalVertex[colocals[j]] = colocals[0];
			}
			XA_DEBUG_ASSERT(m_nextColocalVertex[i] != UINT32_MAX);
		}
	}

	void createColocalsHash()
	{
		const uint32_t vertexCount = XATLAS_VECTOR_SIZE(_positions);
		HashMap<Vector3> positionToVertexMap(MemTag::Default, vertexCount);
		for (uint32_t i = 0; i < vertexCount; i++)
			positionToVertexMap.add(_positions[i]);
		Array<uint32_t> colocals(MemTag::MeshColocals);
		m_nextColocalVertex.resize(vertexCount);
		m_nextColocalVertex.fillBytes(0xff);
		m_firstColocalVertex.resize(vertexCount);
		m_firstColocalVertex.fillBytes(0xff);
		for (uint32_t i = 0; i < vertexCount; i++) {
			if (m_nextColocalVertex[i] != UINT32_MAX)
				continue; // Already linked.
			// Find other vertices colocal to this one.
			colocals.clear();
			colocals.push_back(i); // Always add this vertex.
			uint32_t otherVertex = positionToVertexMap.get(_positions[i]);
			while (otherVertex != UINT32_MAX) {
				if (otherVertex != i && equal(_positions[i], _positions[otherVertex], m_epsilon) && m_nextColocalVertex[otherVertex] == UINT32_MAX)
					colocals.push_back(otherVertex);
				otherVertex = positionToVertexMap.getNext(_positions[i], otherVertex);
			}
			if (colocals.size() == 1) {
				// No colocals for this vertex.
				m_nextColocalVertex[i] = i;
				m_firstColocalVertex[i] = i;
				continue;
			}
			// Link in ascending order.
			insertionSort(colocals.data(), colocals.size());
			for (uint32_t j = 0; j < colocals.size(); j++) {
				m_nextColocalVertex[colocals[j]] = colocals[(j + 1) % colocals.size()];
				m_firstColocalVertex[colocals[j]] = colocals[0];
			}
			XA_DEBUG_ASSERT(m_nextColocalVertex[i] != UINT32_MAX);
		}
	}

	void createColocals()
	{
		if (m_epsilon <= FLT_EPSILON)
			createColocalsHash();
		else
			createColocalsBVH();
	}

	void createBoundaries()
	{
		const uint32_t edgeCount = XATLAS_VECTOR_SIZE(_indices);
		const uint32_t vertexCount = XATLAS_VECTOR_SIZE(_positions);
		m_oppositeEdges.resize(edgeCount);
		m_boundaryEdges.reserve(uint32_t(edgeCount * 0.1f));
		m_isBoundaryVertex.resize(vertexCount);
		m_isBoundaryVertex.zeroOutMemory();
		for (uint32_t i = 0; i < edgeCount; i++)
			m_oppositeEdges[i] = UINT32_MAX;
		const uint32_t faceCount = XATLAS_VECTOR_SIZE(_indices) / 3;
		for (uint32_t i = 0; i < faceCount; i++) {
			if (isFaceIgnored(i))
				continue;
			for (uint32_t j = 0; j < 3; j++) {
				const uint32_t edge = i * 3 + j;
				const uint32_t vertex0 = _indices[edge];
				const uint32_t vertex1 = _indices[i * 3 + (j + 1) % 3];
				// If there is an edge with opposite winding to this one, the edge isn't on a boundary.
				const uint32_t oppositeEdge = findEdge(vertex1, vertex0);
				if (oppositeEdge != UINT32_MAX) {
					m_oppositeEdges[edge] = oppositeEdge;
				}
				else {
					m_boundaryEdges.push_back(edge);
					m_isBoundaryVertex.set(vertex0);
					m_isBoundaryVertex.set(vertex1);
				}
			}
		}
	}

	/// Find edge, test all colocals.
	uint32_t findEdge(uint32_t vertex0, uint32_t vertex1) const
	{
		// Try to find exact vertex match first.
		{
			EdgeKey key(vertex0, vertex1);
			uint32_t edge = m_edgeMap.get(key);
			while (edge != UINT32_MAX) {
				// Don't find edges of ignored faces.
				if (!isFaceIgnored(meshEdgeFace(edge)))
					return edge;
				edge = m_edgeMap.getNext(key, edge);
			}
		}
		// If colocals were created, try every permutation.
		if (!m_nextColocalVertex.isEmpty()) {
			uint32_t colocalVertex0 = vertex0;
			for (;;) {
				uint32_t colocalVertex1 = vertex1;
				for (;;) {
					EdgeKey key(colocalVertex0, colocalVertex1);
					uint32_t edge = m_edgeMap.get(key);
					while (edge != UINT32_MAX) {
						// Don't find edges of ignored faces.
						if (!isFaceIgnored(meshEdgeFace(edge)))
							return edge;
						edge = m_edgeMap.getNext(key, edge);
					}
					colocalVertex1 = ARRAY_GET_CONST(m_nextColocalVertex, colocalVertex1);
					if (colocalVertex1 == vertex1)
						break; // Back to start.
				}
				colocalVertex0 = ARRAY_GET_CONST(m_nextColocalVertex, colocalVertex0);
				if (colocalVertex0 == vertex0)
					break; // Back to start.
			}
		}
		return UINT32_MAX;
	}

	// Edge map can be destroyed when no longer used to reduce memory usage. It's used by:
	//   * Mesh::createBoundaries()
	//   * Mesh::edgeMap() (used by MeshFaceGroups)
	void destroyEdgeMap()
	{
		m_edgeMap.destroy();
	}

#if XA_DEBUG_EXPORT_OBJ
	void writeObjVertices(FILE* file) const
	{
		for (uint32_t i = 0; i < _positions.size(); i++)
			fprintf(file, "v %g %g %g\n", _positions[i].x, _positions[i].y, _positions[i].z);
		if (m_flags & MeshFlags::HasNormals) {
			for (uint32_t i = 0; i < _normals.size(); i++)
				fprintf(file, "vn %g %g %g\n", _normals[i].x, _normals[i].y, _normals[i].z);
		}
		for (uint32_t i = 0; i < _texcoords.size(); i++)
			fprintf(file, "vt %g %g\n", _texcoords[i].x, _texcoords[i].y);
	}

	void writeObjFace(FILE* file, uint32_t face, uint32_t offset = 0) const
	{
		fprintf(file, "f ");
		for (uint32_t j = 0; j < 3; j++) {
			const uint32_t index = _indices[face * 3 + j] + 1 + offset; // 1-indexed
			fprintf(file, "%d/%d/%d%c", index, index, index, j == 2 ? '\n' : ' ');
		}
	}

	void writeObjBoundaryEges(FILE* file) const
	{
		if (m_oppositeEdges.isEmpty())
			return; // Boundaries haven't been created.
		fprintf(file, "o boundary_edges\n");
		for (uint32_t i = 0; i < edgeCount(); i++) {
			if (m_oppositeEdges[i] != UINT32_MAX)
				continue;
			fprintf(file, "l %d %d\n", _indices[meshEdgeIndex0(i)] + 1, _indices[meshEdgeIndex1(i)] + 1); // 1-indexed
		}
	}

	void writeObjFile(const char* filename) const
	{
		FILE* file;
		XA_FOPEN(file, filename, "w");
		if (!file)
			return;
		writeObjVertices(file);
		fprintf(file, "s off\n");
		fprintf(file, "o object\n");
		for (uint32_t i = 0; i < faceCount(); i++)
			writeObjFace(file, i);
		writeObjBoundaryEges(file);
		fclose(file);
	}
#endif

	float computeSurfaceArea() const
	{
		float area = 0;
		for (uint32_t f = 0; f < faceCount(); f++)
			area += computeFaceArea(f);
		XA_DEBUG_ASSERT(area >= 0);
		return area;
	}

	// Returned value is always positive, even if some triangles are flipped.
	float computeParametricArea() const
	{
		float area = 0;
		for (uint32_t f = 0; f < faceCount(); f++)
			area += fabsf(computeFaceParametricArea(f)); // May be negative, depends on texcoord winding.
		return area;
	}

	float computeFaceArea(uint32_t face) const
	{
		const Vector3& p0 = _positions[_indices[face * 3 + 0]];
		const Vector3& p1 = _positions[_indices[face * 3 + 1]];
		const Vector3& p2 = _positions[_indices[face * 3 + 2]];
		return length(cross(p1 - p0, p2 - p0)) * 0.5f;
	}

	Vector3 computeFaceCentroid(uint32_t face) const
	{
		Vector3 sum(0.0f);
		for (uint32_t i = 0; i < 3; i++)
			sum += _positions[_indices[face * 3 + i]];
		return sum / 3.0f;
	}

	// Average of the edge midpoints weighted by the edge length.
	// I want a point inside the triangle, but closer to the cirumcenter.
	Vector3 computeFaceCenter(uint32_t face) const
	{
		const Vector3& p0 = _positions[_indices[face * 3 + 0]];
		const Vector3& p1 = _positions[_indices[face * 3 + 1]];
		const Vector3& p2 = _positions[_indices[face * 3 + 2]];
		const float l0 = length(p1 - p0);
		const float l1 = length(p2 - p1);
		const float l2 = length(p0 - p2);
		const Vector3 m0 = (p0 + p1) * l0 / (l0 + l1 + l2);
		const Vector3 m1 = (p1 + p2) * l1 / (l0 + l1 + l2);
		const Vector3 m2 = (p2 + p0) * l2 / (l0 + l1 + l2);
		return m0 + m1 + m2;
	}

	Vector3 computeFaceNormal(uint32_t face) const
	{
		const Vector3& p0 = _positions[_indices[face * 3 + 0]];
		const Vector3& p1 = _positions[_indices[face * 3 + 1]];
		const Vector3& p2 = _positions[_indices[face * 3 + 2]];
		const Vector3 e0 = p2 - p0;
		const Vector3 e1 = p1 - p0;
		const Vector3 normalAreaScaled = cross(e0, e1);
		return normalizeSafe(normalAreaScaled, Vector3(0, 0, 1));
	}

	float computeFaceParametricArea(uint32_t face) const
	{
		const Vector2& t0 = _texcoords[_indices[face * 3 + 0]];
		const Vector2& t1 = _texcoords[_indices[face * 3 + 1]];
		const Vector2& t2 = _texcoords[_indices[face * 3 + 2]];
		return triangleArea(t0, t1, t2);
	}

	// @@ This is not exactly accurate, we should compare the texture coordinates...
	bool isSeam(uint32_t edge) const
	{
		const uint32_t oppositeEdge = m_oppositeEdges[edge];
		if (oppositeEdge == UINT32_MAX)
			return false; // boundary edge
		const uint32_t e0 = meshEdgeIndex0(edge);
		const uint32_t e1 = meshEdgeIndex1(edge);
		const uint32_t oe0 = meshEdgeIndex0(oppositeEdge);
		const uint32_t oe1 = meshEdgeIndex1(oppositeEdge);
		return _indices[e0] != _indices[oe1] || _indices[e1] != _indices[oe0];
	}

	bool isTextureSeam(uint32_t edge) const
	{
		const uint32_t oppositeEdge = m_oppositeEdges[edge];
		if (oppositeEdge == UINT32_MAX)
			return false; // boundary edge
		const uint32_t e0 = meshEdgeIndex0(edge);
		const uint32_t e1 = meshEdgeIndex1(edge);
		const uint32_t oe0 = meshEdgeIndex0(oppositeEdge);
		const uint32_t oe1 = meshEdgeIndex1(oppositeEdge);
		return _texcoords[_indices[e0]] != _texcoords[_indices[oe1]] || _texcoords[_indices[e1]] != _texcoords[_indices[oe0]];
	}

	uint32_t firstColocalVertex(uint32_t vertex) const
	{
		XA_DEBUG_ASSERT(m_firstColocalVertex.size() == XATLAS_VECTOR_SIZE(_positions));
		return m_firstColocalVertex[vertex];
	}

	XA_INLINE float epsilon() const { return m_epsilon; }
	XA_INLINE uint32_t edgeCount() const { return XATLAS_VECTOR_SIZE(_indices); }
	XA_INLINE uint32_t oppositeEdge(uint32_t edge) const { return m_oppositeEdges[edge]; }
	XA_INLINE bool isBoundaryEdge(uint32_t edge) const { return m_oppositeEdges[edge] == UINT32_MAX; }
	XA_INLINE const Array<uint32_t>& boundaryEdges() const { return m_boundaryEdges; }
	XA_INLINE bool isBoundaryVertex(uint32_t vertex) const { return m_isBoundaryVertex.get(vertex); }
	XA_INLINE uint32_t vertexCount() const { return XATLAS_VECTOR_SIZE(_positions); }
	//XA_INLINE uint32_t vertexAt(uint32_t i) const { return _indices[i]; }
	XA_INLINE const Vector3& position(uint32_t vertex) const { return _positions[vertex]; }
	XA_INLINE ConstArrayView<Vector3> positions() const { return { _positions, XATLAS_VECTOR_SIZE(_positions) }; }
	XA_INLINE const Vector3& normal(uint32_t vertex) const { XA_DEBUG_ASSERT(m_flags & MeshFlags::HasNormals); return _normals[vertex]; }
	XA_INLINE const Vector2& texcoord(uint32_t vertex) const { return _texcoords[vertex]; }
	XA_INLINE Vector2& texcoord(uint32_t vertex) { return _texcoords[vertex]; }
	XA_INLINE const ConstArrayView<Vector2> texcoords() const { return { _texcoords, XATLAS_VECTOR_SIZE(_texcoords) }; }
	XA_INLINE ArrayView<Vector2> texcoords() { return { _texcoords, XATLAS_VECTOR_SIZE(_texcoords) }; }
	XA_INLINE uint32_t faceCount() const { return XATLAS_VECTOR_SIZE(_indices) / 3; }
	XA_INLINE ConstArrayView<uint32_t> indices() const { return { _indices, XATLAS_VECTOR_SIZE(_indices) }; }
	XA_INLINE uint32_t indexCount() const { return XATLAS_VECTOR_SIZE(_indices); }
	XA_INLINE bool isFaceIgnored(uint32_t face) const { return (m_flags & MeshFlags::HasIgnoredFaces) && _face_ignore[face]; }
	XA_INLINE uint32_t faceMaterial(uint32_t face) const { return (m_flags & MeshFlags::HasMaterials) ? _face_materials[face] : UINT32_MAX; }
	XA_INLINE const HashMap<EdgeKey, EdgeHash>& edgeMap() const { return m_edgeMap; }

public:
	float m_epsilon;
	uint32_t m_flags;
	uint32_t m_id;

	XATLAS_VECTOR_DECLARE(bool, _face_ignore);
	XATLAS_VECTOR_DECLARE(uint32_t, _face_materials);
	XATLAS_VECTOR_DECLARE(uint32_t, _indices);
	XATLAS_VECTOR_DECLARE(Vector3, _positions);
	XATLAS_VECTOR_DECLARE(Vector3, _normals);
	XATLAS_VECTOR_DECLARE(Vector2, _texcoords);



	// Populated by createColocals
	Array<uint32_t> m_nextColocalVertex; // In: vertex index. Out: the vertex index of the next colocal position.
	Array<uint32_t> m_firstColocalVertex;

	// Populated by createBoundaries
	BitArray m_isBoundaryVertex;
	Array<uint32_t> m_boundaryEdges;
	Array<uint32_t> m_oppositeEdges; // In: edge index. Out: the index of the opposite edge (i.e. wound the opposite direction). UINT32_MAX if the input edge is a boundary edge.

	HashMap<EdgeKey, EdgeHash> m_edgeMap;

public:
	class FaceEdgeIterator
	{
	public:
		FaceEdgeIterator(const Mesh* mesh, uint32_t face) : m_mesh(mesh), m_face(face), m_relativeEdge(0)
		{
			m_edge = m_face * 3;
		}

		void advance()
		{
			if (m_relativeEdge < 3) {
				m_edge++;
				m_relativeEdge++;
			}
		}

		bool isDone() const
		{
			return m_relativeEdge == 3;
		}

		bool isBoundary() const { return ARRAY_GET_CONST(m_mesh->m_oppositeEdges, m_edge) == UINT32_MAX; }
		bool isSeam() const { return m_mesh->isSeam(m_edge); }
		bool isTextureSeam() const { return m_mesh->isTextureSeam(m_edge); }
		uint32_t edge() const { return m_edge; }
		uint32_t relativeEdge() const { return m_relativeEdge; }
		uint32_t face() const { return m_face; }
		uint32_t oppositeEdge() const { return m_mesh->m_oppositeEdges[m_edge]; }

		uint32_t oppositeFace() const
		{
			const uint32_t oedge = ARRAY_GET_CONST(m_mesh->m_oppositeEdges, m_edge);
			if (oedge == UINT32_MAX)
				return UINT32_MAX;
			return meshEdgeFace(oedge);
		}

		uint32_t vertex0() const { return m_mesh->_indices[m_face * 3 + m_relativeEdge]; }
		uint32_t vertex1() const { return m_mesh->_indices[m_face * 3 + (m_relativeEdge + 1) % 3]; }
		const Vector3& position0() const { return m_mesh->_positions[vertex0()]; }
		const Vector3& position1() const { return m_mesh->_positions[vertex1()]; }
		const Vector3& normal0() const { return m_mesh->_normals[vertex0()]; }
		const Vector3& normal1() const { return m_mesh->_normals[vertex1()]; }
		const Vector2& texcoord0() const { return m_mesh->_texcoords[vertex0()]; }
		const Vector2& texcoord1() const { return m_mesh->_texcoords[vertex1()]; }

	private:
		const Mesh* m_mesh;
		uint32_t m_face;
		uint32_t m_edge;
		uint32_t m_relativeEdge;
	};
};
