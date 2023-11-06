#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"
#include "Renderer.h"

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{_fovAngle}
		{
		}


		Vector3 origin{};
		float fovAngle{90.f};

		Vector3 forward{Vector3::UnitZ};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{0.f};
		float totalYaw{0.f};

		Matrix cameraToWorld{};

		const float speed{ 10.f };


		Matrix CalculateCameraToWorld()
		{
			right = Vector3::Cross(Vector3::UnitY, forward);
			right = right.Normalized();
			up = Vector3::Cross(forward, right);
			up = up.Normalized();

			//rotation matrix
			cameraToWorld = { Vector4{right,0}, Vector4{up,0}, Vector4{forward,0}, Vector4{origin,1} };
			return cameraToWorld;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			//keyboard input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			if (pKeyboardState[SDL_SCANCODE_W])
			{
				origin.z += speed * deltaTime;
			}
			else if (pKeyboardState[SDL_SCANCODE_S])
			{
				origin.z -= speed * deltaTime;
			}
			else if (pKeyboardState[SDL_SCANCODE_D])
			{
				origin.x += speed * deltaTime;
			}
			else if (pKeyboardState[SDL_SCANCODE_A])
			{
				origin.x -= speed * deltaTime;
			}

			//mouse input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			if ((mouseState & SDL_BUTTON_RMASK) != 0)
			{
				if (!(mouseState & SDL_BUTTON_LMASK) != 0)
				{
					// Right mouse button is pressed
					totalYaw += mouseX;
					totalPitch += mouseY;

					// Create rotation matrices for pitch and yaw
					const Matrix rotationX = Matrix::CreateRotationX(totalPitch * TO_RADIANS);
					const Matrix rotationY = Matrix::CreateRotationY(totalYaw * TO_RADIANS);

					// Combine the two rotations
					const Matrix rotation = rotationX * rotationY;

					// Update the camera's forward vector based on the new orientation
					forward = rotation.TransformVector(Vector3::UnitZ).Normalized();
				}
			}
		}
	};
}
