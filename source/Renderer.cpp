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

	float ascpectRatio{ m_Width / static_cast<float>(m_Height) };

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			float x{ (2.f * ((static_cast<float>(px) + 0.5f ) / static_cast<float>(m_Width)) - 1.f) * ascpectRatio };
			float y{ (1.f - 2.f * ((static_cast<float>(py) + 0.5f) / static_cast<float>(m_Height)))};

			Vector3 rayDirection{ x,y, 1.f };

			//Ray we are casting from canera towards each pixel
			Ray viewRay{ {0,0,0}, rayDirection.Normalized() };

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
			}

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}
