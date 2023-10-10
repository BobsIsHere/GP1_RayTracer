#pragma once
#include <cassert>
#include "Math.h"

namespace dae
{
	namespace BRDF
	{
		/**
		 * \param kd Diffuse Reflection Coefficient
		 * \param cd Diffuse Color
		 * \return Lambert Diffuse Color
		 */
		static ColorRGB Lambert(float kd, const ColorRGB& cd)
		{
			ColorRGB rho{ cd * kd };
			return rho / dae::PI;
		}

		static ColorRGB Lambert(const ColorRGB& kd, const ColorRGB& cd)
		{
			ColorRGB rho{ cd * kd };
			return rho / dae::PI;
		}

		/**
		 * \brief todo
		 * \param ks Specular Reflection Coefficient
		 * \param exp Phong Exponent
		 * \param l Incoming (incident) Light Direction
		 * \param v View Direction
		 * \param n Normal of the Surface
		 * \return Phong Specular Color
		 */
		static ColorRGB Phong(float ks, float exp, const Vector3& l, const Vector3& v, const Vector3& n)
		{
			const Vector3 reflect{ l - (2 * (Vector3::Dot(n, l)) * n) };
			const float angle{ std::max (0.f ,Vector3::Dot(reflect, v)) };
			const float reflection{ ks * std::powf(angle, exp) };
			
			return ColorRGB{ reflection, reflection, reflection };
		}

		/**
		 * \brief BRDF Fresnel Function >> Schlick
		 * \param h Normalized Halfvector between View and Light directions
		 * \param v Normalized View direction
		 * \param f0 Base reflectivity of a surface based on IOR (Indices Of Refrection), this is different for Dielectrics (Non-Metal) and Conductors (Metal)
		 * \return
		 */
		static ColorRGB FresnelFunction_Schlick(const Vector3& h, const Vector3& v, const ColorRGB& f0)
		{
			//float schlick{ 1 - Vector3::Dot(h, v) };

			return f0 + (ColorRGB{1.f,1.f,1.f} - f0) * powf((1.f - Vector3::Dot(h, v)), 5.f);
		}

		/**
		 * \brief BRDF NormalDistribution >> Trowbridge-Reitz GGX (UE4 implemetation - squared(roughness))
		 * \param n Surface normal
		 * \param h Normalized half vector
		 * \param roughness Roughness of the material
		 * \return BRDF Normal Distribution Term using Trowbridge-Reitz GGX
		 */
		static float NormalDistribution_GGX(const Vector3& n, const Vector3& h, float roughness)
		{
			/*const float roughnessSquared{ roughness * roughness };
			const float denominator{ (Vector3::Dot(n,h) * Vector3::Dot(n,h)) * (roughnessSquared - 1) + 1 };

			return (roughnessSquared / M_PI * (denominator * denominator)) - roughnessSquared;*/
			const float a{ Square(roughness) };
			const float dpSquared{ Square(Vector3::Dot(n,h)) };
			const float denominator{ dae::PI * Square(dpSquared * (a - 1.f) + 1.f) };

			return a / denominator;
		}


		/**
		 * \brief BRDF Geometry Function >> Schlick GGX (Direct Lighting + UE4 implementation - squared(roughness))
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param roughness Roughness of the material
		 * \return BRDF Geometry Term using SchlickGGX
		 */
		static float GeometryFunction_SchlickGGX(const Vector3& n, const Vector3& v, float roughness)
		{
			/*const float dp{ Vector3::Dot(n,v) };
			const float roughnessSquared{ roughness * roughness };

			const float directLighting{ (roughnessSquared + 1) * 2 / 8 };
			const float UE4Formula{ dp / dp * (1 - roughnessSquared) + roughnessSquared };

			return directLighting + UE4Formula - roughnessSquared;*/

			const float dp{Vector3::Dot(n,v)};
			const float k{ Square(roughness + 1.f) / 8.f };

			return dp / (dp * (1.f - k) + k);
		}

		/**
		 * \brief BRDF Geometry Function >> Smith (Direct Lighting)
		 * \param n Normal of the surface
		 * \param v Normalized view direction
		 * \param l Normalized light direction
		 * \param roughness Roughness of the material
		 * \return BRDF Geometry Term using Smith (> SchlickGGX(n,v,roughness) * SchlickGGX(n,l,roughness))
		 */
		static float GeometryFunction_Smith(const Vector3& n, const Vector3& v, const Vector3& l, float roughness)
		{
			const float smith1{ GeometryFunction_SchlickGGX(n, v, roughness) };
			const float smith2{ GeometryFunction_SchlickGGX(n, l, roughness) };

			return smith2 * smith1;
		}
	}
}