#pragma once
#include <cassert>

#include "Math.h"
#include "vector"

namespace dae
{
#pragma region GEOMETRY
	struct Sphere
	{
		Vector3 origin;
		float radius;

		unsigned char materialIndex{ 0 };
	};

	struct Plane
	{
		Vector3 origin;
		Vector3 normal;

		unsigned char materialIndex{ 0 };
	};

	enum class TriangleCullMode
	{
		FrontFaceCulling,
		BackFaceCulling,
		NoCulling
	};

	struct Triangle
	{
		Triangle() = default;
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal):
			v0{_v0}, v1{_v1}, v2{_v2}, normal{_normal.Normalized()}{}

		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
		{
			const Vector3 edgeV0V1 = v1 - v0;
			const Vector3 edgeV0V2 = v2 - v0;
			normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
		}

		Vector3 v0;
		Vector3 v1;
		Vector3 v2;

		Vector3 normal;

		TriangleCullMode cullMode{};
		unsigned char materialIndex{};
	};

	struct TriangleMesh
	{
		TriangleMesh() = default;
		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, TriangleCullMode _cullMode):
		positions(_positions), indices(_indices), cullMode(_cullMode)
		{
			//Calculate Normals
			//CalculateNormals();

			//Update Transforms
			UpdateTransforms();
		}

		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
			positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
		{
			UpdateTransforms();
		}

		std::vector<Vector3> positions;
		std::vector<Vector3> normals;
		std::vector<int> indices;
		unsigned char materialIndex;

		TriangleCullMode cullMode{TriangleCullMode::BackFaceCulling};

		Matrix rotationTransform;
		Matrix translationTransform;
		Matrix scaleTransform;

		Vector3 minAABB;
		Vector3 maxAABB;

		Vector3 transformedMinAABB;
		Vector3 transformedMaxAABB;

		std::vector<Vector3> transformedPositions;
		std::vector<Vector3> transformedNormals;

		void Translate(const Vector3& translation)
		{
			translationTransform = Matrix::CreateTranslation(translation);
		}

		void RotateY(float yaw)
		{
			rotationTransform = Matrix::CreateRotationY(yaw);
		}

		void Scale(const Vector3& scale)
		{
			scaleTransform = Matrix::CreateScale(scale);
		}

		void AppendTriangle(const Triangle& triangle, bool ignoreTransformUpdate = false)
		{
			int startIndex = static_cast<int>(positions.size());

			positions.push_back(triangle.v0);
			positions.push_back(triangle.v1);
			positions.push_back(triangle.v2);

			indices.push_back(startIndex);
			indices.push_back(++startIndex);
			indices.push_back(++startIndex);

			normals.push_back(triangle.normal);

			//Not ideal, but making sure all vertices are updated
			if(!ignoreTransformUpdate)
				UpdateTransforms();
		}

		void CalculateNormals()
		{
			normals.clear();
			normals.reserve(indices.size() * 0.333f );
			
			for (int idx = 0; idx < indices.size(); idx += 3)
			{
				const Vector3 a{ positions[indices[idx + 1]] - positions[indices[idx]] };
				const Vector3 b{ positions[indices[idx + 2]] - positions[indices[idx + 1]] };

				normals.push_back(Vector3::Cross(a, b).Normalized());
			}
		}

		void UpdateAABB()
		{
			if (positions.size() > 0)
			{
				minAABB = positions[0];
				maxAABB = positions[0];

				for (auto& pos : positions)
				{
					minAABB = Vector3::Min(pos, minAABB);
					maxAABB = Vector3::Max(pos, maxAABB);
				}
			}
		}

		void UpdateTransforms()
		{
			//calculate final transform 
			const Matrix finalTransform{ scaleTransform * rotationTransform * translationTransform };

			//clear normals and positions, reserving memory for vectors
			transformedNormals.clear();
			transformedNormals.reserve(normals.size());

			transformedPositions.clear();
			transformedPositions.reserve(positions.size());

			//transform positions
			//loops over position, applies finalTransform to each and stores transformed point in vector
			for (int positionIdx = 0; positionIdx < positions.size(); ++positionIdx)
			{
				Vector3 result{ finalTransform.TransformPoint(positions[positionIdx]) };
				transformedPositions.emplace_back(result);
			}

			//transform normals
			//loops over normals, applies finalTransform to each and stores transformed normal in vector
			for (int normalIdx = 0; normalIdx < normals.size(); ++normalIdx)
			{
				Vector3 result{ finalTransform.TransformVector(normals[normalIdx]) };
				transformedNormals.emplace_back(result);
			}

			//update transforms
			UpdateTransformedAABB(finalTransform);
		}

		void UpdateTransformedAABB(const Matrix& finalTransform)
		{
			//AABB update : be careful -> transform the 8 vertices of the AABB
			//and calculate new min. and max.
			Vector3 tMinAABB{ finalTransform.TransformPoint(minAABB) };
			Vector3 tMaxAABB{ tMinAABB };
			//(xmax, ymin, zmin)
			Vector3 tAABB{ finalTransform.TransformPoint(maxAABB.x, minAABB.y, minAABB.z) };
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmax, ymin, zmax)
			tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmin, ymin, zmax)
			tAABB = finalTransform.TransformPoint(minAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmin, ymax, zmin)
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmax, ymax, zmin)
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmax, ymax, zmax)
			tAABB = finalTransform.TransformPoint(maxAABB);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//(xmin, ymax, zmin)
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			transformedMinAABB = tMinAABB;
			transformedMaxAABB = tMaxAABB;
		}
	};
#pragma endregion
#pragma region LIGHT
	enum class LightType
	{
		Point,
		Directional
	};

	struct Light
	{
		Vector3 origin;
		Vector3 direction;
		ColorRGB color;
		float intensity;

		LightType type;
	};
#pragma endregion
#pragma region MISC
	struct Ray
	{
		Vector3 origin;
		Vector3 direction;

		float min{ 0.0001f };
		float max{ FLT_MAX };
	};

	struct HitRecord
	{
		Vector3 origin;
		Vector3 normal;
		//distance from origin to hitPixel along the normal/ray
		float t = FLT_MAX;

		bool didHit{ false };
		unsigned char materialIndex{ 0 };
	};
#pragma endregion
}