#define PARALLEL_EXECUTION

//External includes
#include "SDL.h"
#include "SDL_surface.h"
#include "Renderer.h"
#include <execution>

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
	const Matrix cameraToWorld{ camera.CalculateCameraToWorld() };

	//precompute constants
	const float ascpectRatio{ m_Width / static_cast<float>(m_Height) };
	const float fov{ tanf( (camera.fovAngle * TO_RADIANS) * 0.5f ) };
	const float minLengthLight{ 0.0001f };
	
	const uint32_t amountOfPixels{ uint32_t(m_Width * m_Height) };

#if defined(PARALLEL_EXECUTION)
	//parallel logic
	std::vector<uint32_t> pixelIndicies{};
	pixelIndicies.reserve(amountOfPixels);

	for (uint32_t idx{}; idx < amountOfPixels; ++idx)
	{
		pixelIndicies.emplace_back(idx);
	}

	std::for_each(std::execution::par, pixelIndicies.begin(), pixelIndicies.end(), [&](int idx) 
	{
		RenderPixel(pScene, idx, fov, ascpectRatio, cameraToWorld, camera.origin);
	} );

#else
	//sychronous logic (no threading)
	const uint32_t amountOfPixels{ uint32_t(m_Width * m_Height) };

	for (uint32_t pixelIndex{}; pixelIndex < amountOfPixels; ++pixelIndex)
	{
		RenderPixel(pScene, pixelIndex, fov, ascpectRatio, cameraToWorld, camera.origin);
	}

#endif
	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void Renderer::RenderPixel(Scene* pScene, uint32_t pixelIndex, float fov, float aspectRatio, const Matrix cameraToWorld, const Vector3 cameraOrigin) const
{
	//variables
	const auto& materials{ pScene->GetMaterials() };
	const auto& lights{ pScene->GetLights() };
	const float minLengthLight{ 0.0001f };

	const uint32_t px{ pixelIndex % m_Width }, py{ pixelIndex / m_Width };
	const float cx{ (2 * ((px + 0.5f) / float(m_Width)) - 1) * aspectRatio * fov };
	const float cy{ (1 - (2 * ((py + 0.5f) / float(m_Height)))) * fov };

	//code
	Vector3 rayDirection{ cx, cy, 1.f };
	rayDirection = cameraToWorld.TransformVector(rayDirection);
	rayDirection.Normalize();

	//color to write to color buffer (default = black)
	ColorRGB finalColor{};

	//ray we are casting from camera towards each pixel
	Ray viewRay{ cameraOrigin, rayDirection };
	
	//HitRecord containing more info about potential hit
	HitRecord closestHit{};
	pScene->GetClosestHit(viewRay, closestHit);

	if (closestHit.didHit)
	{
		const Vector3 movedHitOrigin{ closestHit.origin + closestHit.normal * minLengthLight };

		for (const auto& light : lights)
		{
			//variables
			Vector3 directionLight{ LightUtils::GetDirectionToLight(light, closestHit.origin) };
			const float distance{ directionLight.Normalize() - minLengthLight };
			const ColorRGB brdfRGB{ materials[closestHit.materialIndex]->Shade(closestHit, directionLight, -rayDirection) };

			const float observedArea{ Vector3::Dot(closestHit.normal, directionLight) };
			if (observedArea <= 0)
			{
				continue;
			}

			const Ray lightRay{ closestHit.origin, directionLight, minLengthLight, distance};
			if (m_ShadowsEnabled && pScene->DoesHit(lightRay))
			{
				continue;
			}

			switch (m_CurrentLightingMode)
			{
			case dae::Renderer::LightingMode::ObservedArea:
				finalColor += ColorRGB{ 1.f, 1.f, 1.f } * observedArea;
				break;
			case dae::Renderer::LightingMode::Radiance:
				finalColor += LightUtils::GetRadiance(light, closestHit.origin);
				break;
			case dae::Renderer::LightingMode::BRDF:
				finalColor += brdfRGB;
				break;
			case dae::Renderer::LightingMode::Combined:
				finalColor += LightUtils::GetRadiance(light, closestHit.origin) * brdfRGB * observedArea;
				break;
			}
		}
	}
	viewRay.direction = Vector3::Reflect(viewRay.direction, closestHit.normal);
	viewRay.origin = closestHit.origin;

	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));

	const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	int temp{ static_cast<int>(m_CurrentLightingMode) };
	m_CurrentLightingMode = static_cast<LightingMode>((++temp) % 4);
}
