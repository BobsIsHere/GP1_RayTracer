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
			if (ignoreHitRecord){ return false; }

			const Vector3 vecToCenter{ sphere.origin - ray.origin };
			const float dotProduct{ Vector3::Dot(vecToCenter, ray.direction) };

			const float discriminant{ sphere.radius * sphere.radius - Vector3::Dot(vecToCenter, vecToCenter) + dotProduct * dotProduct };
			if (discriminant <= 0)
			{
				return false;
			}

			const float tHC{ sqrtf(discriminant) };
			const float t0{ dotProduct - tHC };
			const float t1{ dotProduct + tHC };
			if (t0 <= ray.min || t1 >= ray.max)
			{
				return false;
			}

			hitRecord.t = t0;
			//location of intersection
			hitRecord.origin = ray.origin + t0 * ray.direction;
			//normal of origin
			hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
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
			if (ignoreHitRecord) { return false; }
			
			Vector3 vecPlaneToOrigin{ ray.origin, plane.origin };
			const float dotProduct{ Vector3::Dot(vecPlaneToOrigin,plane.normal) };
			const float dotNormals{ Vector3::Dot(ray.direction, plane.normal)};
			const float t{ dotProduct / dotNormals };

			if (t >= ray.min && t <= ray.max)
			{
				hitRecord.t = t;
				//from start of ray in direction of ray, move forward by distance to hitpoint (= t)
				hitRecord.origin = ray.origin + t * ray.direction;
				hitRecord.normal = plane.normal;
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
			//Variables
			const int nrOfVertices{ 3 };
			const float epsilon{ 0.0000001f };
			const float normalViewDot{ Vector3::Dot(triangle.normal, ray.direction) };

			//Cull Mode Check
			if (triangle.cullMode == TriangleCullMode::FrontFaceCulling /*&& normalViewDot < 0.f*/)
			{
				if (!ignoreHitRecord && normalViewDot < epsilon ||
					ignoreHitRecord && normalViewDot > epsilon)
				{
					return false;
				}
			}
			if (triangle.cullMode == TriangleCullMode::BackFaceCulling /*&& normalViewDot > 0.f*/)
			{
				if (!ignoreHitRecord && normalViewDot > epsilon ||
					ignoreHitRecord && normalViewDot < epsilon)
				{
					return false;
				}
			}

			//Normal VS Ray-Direction Check (Perpendicular?, then don't do calculation)
			const Vector3 a{ triangle.v1 - triangle.v0 };
			const Vector3 b{ triangle.v2 - triangle.v0 };
			const Vector3 n{ Vector3::Cross(a,b) };
			const float dotNormalRay{ Vector3::Dot(n,ray.direction) };

			//absolute makes dot product positive
			if (std::abs(normalViewDot) < epsilon)
			{
				return false;
			}

			if (AreEqual(dotNormalRay, 0))
			{
				return false;
			}
			
			//Ray-Plane test (plane defined by Triangle) + T range check
			const Vector3 L{ triangle.v0 - ray.origin };

			const float dotLToNormal{ Vector3::Dot(L,n) };
			const float dotRayToNormal{ Vector3::Dot(ray.direction,n) };
			const float t{ dotLToNormal / dotRayToNormal };

			if (t < ray.min || t > ray.max)
			{
				return false;
			}

			//line to central of triangle from camera 
			const Vector3 P{ ray.origin + ray.direction * t };

			//Check if hitpoint is inside the Triangle
			std::vector<Vector3> vertices{ triangle.v0,  triangle.v1 , triangle.v2 };

			for (int verticeIdx = 0; verticeIdx < nrOfVertices; ++verticeIdx)
			{
				Vector3 e{ vertices[(verticeIdx + 1) % nrOfVertices] - vertices[verticeIdx] };
				Vector3 p{ P - vertices[verticeIdx] };
				Vector3 crossProduct{ Vector3::Cross(e,p) };
				
				if (Vector3::Dot(crossProduct,n) < 0)
				{
					return false;
				}
			}

			//Fill-in HitRecord (if required)
			if (!ignoreHitRecord)
			{
				hitRecord.t = t;
				hitRecord.origin = P;
				hitRecord.normal = triangle.normal;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.didHit = true;
			}
			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}

		inline bool HitTest_Triangle_MullerTrombore(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const float epsilon{ 0.0000001f };
			const float normalViewDot{ Vector3::Dot(triangle.normal, ray.direction) };

			if (std::abs(normalViewDot) < epsilon)
			{
				return false;
			}

			//Cull Mode Check
			if (triangle.cullMode == TriangleCullMode::FrontFaceCulling)
			{
				if (!ignoreHitRecord && normalViewDot < 0.f ||
					ignoreHitRecord && normalViewDot > 0.f)
				{
					return false;
				}
			}
			if (triangle.cullMode == TriangleCullMode::BackFaceCulling)
			{
				if (!ignoreHitRecord && normalViewDot > 0.f ||
					ignoreHitRecord && normalViewDot < 0.f)
				{
					return false;
				}
			}

			const Vector3 edg1{ triangle.v1 - triangle.v0 };
			const Vector3 edg2{ triangle.v2 - triangle.v0 };
			const Vector3 n{ Vector3::Cross(ray.direction, edg2) };
			const float dotNormalToRay{ Vector3::Dot(edg1,n) };

			if (std::abs(dotNormalToRay) < epsilon)
			{
				return false;
			}



			//Fill-in HitRecord (if required)
			if (!ignoreHitRecord)
			{
				//hitRecord.t = t;
				//hitRecord.origin = P;
				hitRecord.normal = triangle.normal;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.didHit = true;
			}
			return true;
		}

		inline bool HitTest_Triangle_MullerTrombore(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle_MullerTrombore(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			int normalCounter{ 0 };
			float distance{ ray.max };
			Triangle triangle{};
			HitRecord tempHit{};

			//loops through triangles of triangle meshes
			for (int idx{}; idx < mesh.indices.size(); idx += 3)
			{
				//for each group of 3 indices, 
				//extract corresponding transformed normal vector and 3 transformed positions for triangle
				triangle.normal = mesh.transformedNormals[normalCounter];
				triangle.v0 = mesh.transformedPositions[mesh.indices[idx + 2]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[idx + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[idx]];

				triangle.cullMode = mesh.cullMode;
				triangle.materialIndex = mesh.materialIndex;

				++normalCounter;

				//checks for intersection with viewRay 
				if (HitTest_Triangle(triangle, ray, hitRecord))
				{
					if (tempHit.didHit && tempHit.t < distance)
					{
						distance = tempHit.t;
						hitRecord = tempHit;
					}
				}
			}
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
			switch (light.type)
			{
			case LightType::Point :
				return (light.origin - origin);
				break;
			case LightType::Directional :
				return (-light.direction);
				break;
			}
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			switch (light.type)
			{
			case LightType::Point :
				return light.color * (light.intensity / Square((light.origin - target).Magnitude()));
				break;
			case LightType::Directional :
				return light.color * light.intensity;
				break;
			}
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