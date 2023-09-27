#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

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
			right = Vector3::Cross(up, forward);
			right = right.Normalized();
			up = Vector3::Cross(forward, right);
			up = up.Normalized();

			return { right, up, forward, origin };
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			if (pKeyboardState[SDL_SCANCODE_W])
			{
				origin.z += speed * deltaTime;
			}
			if (pKeyboardState[SDL_SCANCODE_S])
			{
				origin.z -= speed * deltaTime;
			}
			if (pKeyboardState[SDL_SCANCODE_D])
			{
				origin.x += speed * deltaTime;
			}
			if (pKeyboardState[SDL_SCANCODE_A])
			{
				origin.x -= speed * deltaTime;
			}

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			/*if (mouseState == mouseX)
			{
				Matrix::CreateRotationX(origin.x += speed * deltaTime);
			}
			if (mouseState == mouseY)
			{
				Matrix::CreateRotationY(origin.y += speed * deltaTime);
			}*/

			if ((mouseState & SDL_BUTTON_RMASK) != 0)
			{
				if (!(mouseState & SDL_BUTTON_LMASK) != 0)
				{

					// Right mouse button is pressed
					totalYaw += mouseX;
					totalPitch += mouseY;

					// Create rotation matrices for pitch and yaw
					Matrix rotationX = Matrix::CreateRotationX(totalPitch * TO_RADIANS);
					Matrix rotationY = Matrix::CreateRotationY(totalYaw * TO_RADIANS);

					// Combine the two rotations
					Matrix rotation = rotationX * rotationY;

					// Update the camera's forward vector based on the new orientation
					forward = rotation.TransformVector(Vector3::UnitZ).Normalized();
				}
			}
		}
	};
}
