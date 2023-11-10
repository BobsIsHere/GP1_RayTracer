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
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{	
			//variables
			const Vector3 vecToCenter{ ray.origin - sphere.origin};

			//variables for quadratic equation
			const float a{ ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y + ray.direction.z * ray.direction.z };
			const float b{ Vector3::Dot(2.f * ray.direction, vecToCenter) };
			const float c{ (vecToCenter.x * vecToCenter.x + vecToCenter.y * vecToCenter.y + vecToCenter.z * vecToCenter.z) - sphere.radius * sphere.radius };

			//discriminant of equation
			const float discriminant{ b * b - 4.f * a * c };
			//if discriminant is negative or 0, then no intersection
			if (discriminant <= 0.f)
			{
				return false;
			}

			const float sqrtDiscriminant{ sqrt(discriminant) };
			const float invA{ 1.f / (2.f * a) };

			//first intersection along ray
			const float t0{ (-b - sqrtDiscriminant) * invA };
			if (t0 < ray.min || t0 > ray.max)
			{
				//second intersection along ray
				const float t1{ ( -b + sqrtDiscriminant) * invA};
				if (t1 < ray.min || t1 > ray.max)
				{
					return false;
				}

				if (ignoreHitRecord)
				{
					return true;
				}
				//parameter along ray which intersetcion occured
				hitRecord.t = t1;
				//location of intersection
				hitRecord.origin = ray.origin + t1 * ray.direction;
				//normal of origin
				hitRecord.normal = (hitRecord.origin - sphere.origin) / sphere.radius;
				//represents material of geometry
				hitRecord.materialIndex = sphere.materialIndex;
				//whether ray intersected object
				return hitRecord.didHit = true;
			}

			if (ignoreHitRecord)
			{
				return true;
			}
			//parameter along ray which intersetcion occured
			hitRecord.t = t0;
			//location of intersection
			hitRecord.origin = ray.origin + t0 * ray.direction;
			//normal of origin
			hitRecord.normal = (hitRecord.origin - sphere.origin) / sphere.radius;
			//represents material of geometry
			hitRecord.materialIndex = sphere.materialIndex;
			//whether ray intersected object
			return hitRecord.didHit = true;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			temp.t = ray.max;
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//variables
			const Vector3 vecPlaneToOrigin{ ray.origin, plane.origin };
			const float dotNormals{ Vector3::Dot(ray.direction, plane.normal) };

			//check if ray is not parallel to plane
			if (dotNormals != 0.0f)
			{
				const float t{ Vector3::Dot(vecPlaneToOrigin,plane.normal) / dotNormals};

				if (t >= ray.min && t <= ray.max)
				{
					hitRecord.t = t;
					//from start of ray in direction of ray, move forward by distance to hitpoint (= t)
					hitRecord.origin = ray.origin + t * ray.direction;
					hitRecord.normal = plane.normal;
					hitRecord.materialIndex = plane.materialIndex;
					return hitRecord.didHit = true;
				}
			}

			return hitRecord.didHit = false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			temp.t = ray.max;
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//variables
			const int nrOfVertices{ 3 };
			const float epsilon{ 0.0000001f };
			const float normalViewDot{ Vector3::Dot(triangle.normal, ray.direction) };

			//cull Mode Check
			if (triangle.cullMode == TriangleCullMode::FrontFaceCulling && normalViewDot < 0.f)
			{
				return false;
			}
			if (triangle.cullMode == TriangleCullMode::BackFaceCulling && normalViewDot > 0.f)
			{
				return false;
			}

			//normal vs ray-direction check (perpendicular?, then don't do calculation)
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
			
			//ray-plane test (plane defined by Triangle) + t range check
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

			//check if hitpoint is inside the triangle
			const std::vector<Vector3> vertices{ triangle.v0,  triangle.v1 , triangle.v2 };

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

			//fill-in hitRecord
			hitRecord.t = t;
			hitRecord.origin = P;
			hitRecord.normal = triangle.normal;
			hitRecord.materialIndex = triangle.materialIndex;
			return hitRecord.didHit = true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			temp.t = ray.max;
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

			//culling mode check based on direction of normal & ray direction
			if (!ignoreHitRecord)
			{
				if (triangle.cullMode == TriangleCullMode::FrontFaceCulling && normalViewDot < 0.f ||
					triangle.cullMode == TriangleCullMode::BackFaceCulling && normalViewDot > 0.f)
				{
					//ray's direction aligned with orientation of cull mode, so no intersection
					return false;
				}
			}
			//if ignoreHitRecord is true, want to ignore back-face/front-face culling
			else
			{
				if (triangle.cullMode == TriangleCullMode::FrontFaceCulling && normalViewDot > 0.f ||
					triangle.cullMode == TriangleCullMode::BackFaceCulling && normalViewDot < 0.f)
				{
					//ray's direction opposite of orientation of cull mode, so no intersection
					return false;
				}
			}
			

			//vectors for 2 edges sharing vertice 0
			const Vector3 edge1{ triangle.v1 - triangle.v0 };
			const Vector3 edge2{ triangle.v2 - triangle.v0 };
			const Vector3 crossRayEdge{ Vector3::Cross(ray.direction, edge2) };

			//if dot is near epsilon/ 0 -> ray lies in plane of triangle
			const float determinant{ Vector3::Dot(edge1,crossRayEdge) };
			if (std::abs(determinant) < epsilon)
			{
				return false;
			}

			const float inverseDet{ 1.f / determinant };

			//calculate u parameter and testbounds
			const Vector3 vertToRayOrigin{ ray.origin - triangle.v0 };
			const float u{ Vector3::Dot(vertToRayOrigin, crossRayEdge) * inverseDet };
			if (u < 0.f || u > 1.f)
			{
				return false;
			}

			//calculate v parameter and test bounds
			const Vector3 qVec{ Vector3::Cross(vertToRayOrigin, edge1) };
			const float v{ Vector3::Dot(ray.direction ,qVec) * inverseDet };
			if (v < 0.f || u + v > 1.f)
			{
				return false;
			}

			//calculate t, ray intersect triangle
			const float t{ Vector3::Dot(edge2, qVec) * inverseDet };
			if (t < ray.min || t > ray.max)
			{
				return false;
			}

			//fill-in HitRecord
			hitRecord.t = t;
			hitRecord.origin = ray.origin + t * ray.direction;
			hitRecord.normal = triangle.normal;
			hitRecord.materialIndex = triangle.materialIndex;
			return hitRecord.didHit = true;
		}

		inline bool HitTest_Triangle_MullerTrombore(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			temp.t = ray.max;
			return HitTest_Triangle_MullerTrombore(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool SlabTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			const float tx1{ (mesh.transformedMinAABB.x - ray.origin.x) / ray.direction.x };
			const float tx2{ (mesh.transformedMaxAABB.x - ray.origin.x) / ray.direction.x };

			float tMin{ std::min(tx1, tx2) };
			float tMax{ std::max(tx1,tx2) };

			const float ty1{ (mesh.transformedMinAABB.y - ray.origin.y) / ray.direction.y };
			const float ty2{ (mesh.transformedMaxAABB.y - ray.origin.y) / ray.direction.y };

			tMin = std::max(tMin, std::min(ty1, ty2));
			tMax = std::min(tMax, std::max(ty1, ty2));

			const float tz1{ (mesh.transformedMinAABB.z - ray.origin.z) / ray.direction.z };
			const float tz2{ (mesh.transformedMaxAABB.z - ray.origin.z) / ray.direction.z };

			tMin = std::max(tMin, std::min(tz1, tz2));
			tMax = std::min(tMax, std::max(tz1, tz2));

			return tMax > 0 && tMax >= tMin;
		}

		//HELPED BY INE HOCEDEZ
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//SlabTest
			if (!SlabTest_TriangleMesh(mesh, ray))
			{
				return false;
			}
			
			//variables
			Ray tempRay{ ray };
			Triangle tempTriangle{};
			
			//checks for intersection with tempRay 
			if (ignoreHitRecord)
			{
				//for shadows
				for (int idx{}; idx < mesh.indices.size() / 3.f; ++idx)
				{
					//for every 3 points, get 3 transformed positions for triangle
					tempTriangle = { mesh.transformedPositions[mesh.indices[idx * 3]],
									 mesh.transformedPositions[mesh.indices[idx * 3 + 1]],
									 mesh.transformedPositions[mesh.indices[idx * 3 + 2]] };

					//set tempTriangle cull and material to mesh's cull and material
					tempTriangle.cullMode = mesh.cullMode;
					if (HitTest_Triangle_MullerTrombore(tempTriangle, tempRay))
					{
						return true;
					}
				}
			}
			else
			{
				//for lighting
				for (int idx{}; idx < mesh.indices.size() / 3.f; ++idx)
				{
					//for every 3 points, get 3 transformed positions for triangle
					tempTriangle = { mesh.transformedPositions[mesh.indices[idx * 3]], mesh.transformedPositions[mesh.indices[idx * 3 + 1]],
									 mesh.transformedPositions[mesh.indices[idx * 3 + 2]] };

					//set tempTriangle cull and material to mesh's cull and material
					tempTriangle.cullMode = mesh.cullMode;
					tempTriangle.materialIndex = mesh.materialIndex;
					if (HitTest_Triangle_MullerTrombore(tempTriangle, tempRay, hitRecord))
					{
						tempRay.max = hitRecord.t;
					}
				}
			}
			if (!hitRecord.didHit) 
			{ 
				return false; 
			}

			hitRecord.materialIndex = mesh.materialIndex;
			return true;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			temp.t = ray.max;
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//direction from target to light
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
				return light.color * (light.intensity /(light.origin - target).SqrMagnitude());
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
			for (auto& index : indices)
			{
				uint32_t i0 = (3 * index);
				uint32_t i1 = (3 * index) + 1;
				uint32_t i2 = (3 * index) + 2;

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