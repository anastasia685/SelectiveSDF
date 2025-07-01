#include "stdafx.h"
#include "Camera.h"

void Camera::Initialize(float aspectRatio, float fov)
{
	m_aspectRatio = aspectRatio;
	m_fov = fov;

	m_position = XMVectorSet(0, 0.6, 3.5, 0);

	XMVECTOR dir = XMVector3Normalize(XMVectorSubtract(XMVectorZero(), m_position)); // forward vector - position vector

	float fx = XMVectorGetX(dir);
	float fy = XMVectorGetY(dir);
	float fz = XMVectorGetZ(dir);

	m_yaw = atan2f(fx, fz);
	m_pitch = asinf(fy);

	UpdateOrientation(); // Compute forward/right/up

	Update();
	UpdateProjection();
}

void Camera::SetPosition(float x, float y, float z)
{
	m_position = XMVectorSet(x, y, z, 0.0f);
}

void Camera::Update()
{
	m_view = XMMatrixLookAtRH(m_position, XMVectorAdd(m_position, m_forward), m_up);
}

void Camera::SetAspectRatio(float aspectRatio)
{
	m_aspectRatio = aspectRatio;
	UpdateProjection();
}

void Camera::SetFov(float fov)
{
	m_fov = fov;
	UpdateProjection();
}

void Camera::UpdateProjection(float aspectRatio, float fov)
{
	m_aspectRatio = aspectRatio;
	m_fov = fov;
	UpdateProjection();
}

void Camera::UpdateProjection()
{
	float fovAngleY = m_fov * XM_PI / 180.0f;
	m_projection = XMMatrixPerspectiveFovRH(fovAngleY, m_aspectRatio, 0.1f, 1000.0f);
}

void Camera::UpdateOrientation()
{
	m_forward = XMVectorSet(
		cosf(m_pitch) * sinf(m_yaw),
		sinf(m_pitch),
		cosf(m_pitch) * cosf(m_yaw),
		0.0f
	);
	m_forward = XMVector3Normalize(m_forward);

	// Compute right vector (perpendicular to forward and worldUp)
	XMVECTOR worldUp = XMVectorSet(0, 1, 0, 0);
	XMVECTOR right = XMVector3Normalize(XMVector3Cross(worldUp, m_forward));

	// Compute up vector (perpendicular to forward and right)
	m_up = XMVector3Cross(m_forward, right);
}
