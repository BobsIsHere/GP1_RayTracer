#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline float Squared(const float nr) { return nr * nr; }

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//if (ignoreHitRecord){ return false; }
			
			Vector3 vecToCenter{ sphere.origin - ray.origin };
			const float dotProduct{ Vector3::Dot(vecToCenter, ray.direction) };
			const float oppDistanceSquared{ vecToCenter.SqrMagnitude() - Squared(dotProduct)};

			//differnce between the hit and opposite distance
			const float adjacentSideSquared{ Squared(sphere.radius) - oppDistanceSquared };
			//distance from camera to the hit
			const float distanceCameraToHit{ Squared(oppDistanceSquared) - adjacentSideSquared };

			// If radius is smaller than adjacent side = no hit
			if (adjacentSideSquared < 0)
			{
				return hitRecord.didHit = false;
			}
			const Vector3 hitPos{ ray.direction * distanceCameraToHit + ray.origin };

			hitRecord.t = dotProduct - sqrt(adjacentSideSquared);
			//location of intersection
			hitRecord.origin = hitPos;
			//normal of origin
			hitRecord.normal = (hitPos - sphere.origin).Normalized();

			hitRecord.materialIndex = sphere.materialIndex;
			return hitRecord.didHit = true;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//if t is within limits (min and max)
			// -> intersection
			//if (ignoreHitRecord) { return false; }
			
			Vector3 vecPlaneToOrigin{ ray.origin, plane.origin };
			const float dotProduct{ Vector3::Dot(vecPlaneToOrigin,plane.normal) };
			const float dotNormals{ Vector3::Dot(ray.direction, plane.normal)};
			const float t{ dotProduct / dotNormals };

			if (t < ray.max && t > ray.min)
			{
				//from start of ray in direction of ray, move forward by distance to hitpoint (= t)
				hitRecord.origin = ray.origin + t * ray.direction;
				hitRecord.normal = plane.normal;
				hitRecord.t = t;
				hitRecord.materialIndex = plane.materialIndex;
				return hitRecord.didHit = true;
			}

			return hitRecord.didHit = false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//todo W5
			assert(false && "No Implemented Yet!");
			return false;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//todo W5
			assert(false && "No Implemented Yet!");
			return false;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			return (light.origin - origin);
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			//todo W3
			assert(false && "No Implemented Yet!");
			return {};
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}