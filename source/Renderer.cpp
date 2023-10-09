//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"
#include <iostream>

using namespace dae;

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const float ascpectRatio{ m_Width / static_cast<float>(m_Height) };
	const float fov{ tanf( camera.fovAngle / 2 ) };

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			float x{ (2.f * ((static_cast<float>(px) + 0.5f ) / static_cast<float>(m_Width)) - 1.f) * (ascpectRatio * fov) };
			float y{ (1.f - (2.f * ((static_cast<float>(py) + 0.5f) / static_cast<float>(m_Height)))) * fov };

			Vector3 rayDirection{ x,y, 1.f };
			rayDirection = camera.CalculateCameraToWorld().TransformVector(rayDirection);
			rayDirection.Normalize();

			//Ray we are casting from camera towards each pixel
			Ray viewRay{ camera.origin, rayDirection };

			//Color to write to color buffer (default = black)
			ColorRGB finalColor{};

			//HitRecord containing more info about potential hit
			HitRecord closestHit{};
			pScene->GetClosestHit(viewRay, closestHit);

			if (closestHit.didHit)
			{
				//if we hit something, set finalColor to material color, else keep black
				//use HitRecord::materialindex to find corresponding material
				finalColor = materials[closestHit.materialIndex]->Shade();

				const float minLengthLight{ 0.0001f };
				const Vector3 movedHitOrigin{ closestHit.origin + closestHit.normal * minLengthLight };

				for (int idx{}; idx < lights.size(); ++idx)
				{
					//variables
					Vector3 directionLight{ LightUtils::GetDirectionToLight(lights[idx], movedHitOrigin)};
					const float distance{ directionLight.Normalize() /*- minLengthLight*/ };
					float observedArea{ Vector3::Dot(closestHit.normal, directionLight) };
					ColorRGB brdfRGB{ materials[closestHit.materialIndex]->Shade(closestHit, directionLight, rayDirection) };

					if (m_ShadowsEnabled)
					{
						const Ray lightRay{ movedHitOrigin, directionLight, minLengthLight, distance };
						if (pScene->DoesHit(lightRay))
						{
							continue;
						}
					}
					if (observedArea <= 0)
					{
						continue;
					}

					switch (m_CurrentLightingMode)
					{
					case dae::Renderer::LightingMode::ObservedArea:
						finalColor += ColorRGB{ 1.f, 1.f, 1.f } * observedArea;
						break;
					case dae::Renderer::LightingMode::Radiance:
						finalColor += LightUtils::GetRadiance(lights[idx], closestHit.origin);
						break;
					case dae::Renderer::LightingMode::BRDF:
						finalColor += brdfRGB;
						break;
					case dae::Renderer::LightingMode::Combined:
						finalColor += LightUtils::GetRadiance(lights[idx], closestHit.origin) * brdfRGB * observedArea;
						break;
					}
				}
			}

			finalColor.MaxToOne();

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

	const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	int temp{ static_cast<int>(m_CurrentLightingMode) };
	/*++temp;
	
	if (temp > int(LightingMode::Radiance))
	{
		temp = 0;
	}*/

	m_CurrentLightingMode = static_cast<LightingMode>((temp) % 4);

	switch (m_CurrentLightingMode)
	{
	case dae::Renderer::LightingMode::ObservedArea:
		std::cout << "observedArea";
		break;
	case dae::Renderer::LightingMode::Radiance:
		std::cout << "Radiance";
		break;
	case dae::Renderer::LightingMode::BRDF:
		std::cout << "BRDF";
		break;
	case dae::Renderer::LightingMode::Combined:
		std::cout << "Combined";
		break;
	}
}
