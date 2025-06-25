#pragma once

using namespace DirectX;

class Camera
{
public:
	const XMMATRIX& GetViewMatrix() const { return m_view; };
	const XMMATRIX& GetProjectionMatrix() const { return m_projection; };

	const XMVECTOR GetPosition() const { return m_position; };

	void SetPosition(float x, float y, float z);
	void SetRotation(float x, float y, float z);

	void Initialize(float aspectRatio, float fov = 45.0f);
	void Update();
	void SetAspectRatio(float aspectRatio);
	void SetFov(float fov);
	void UpdateProjection(float aspectRatio, float fov);

protected:
	XMMATRIX m_view;
	XMMATRIX m_projection;

	XMVECTOR m_position, m_forward, m_up;

	float m_yaw = 0.0f;
	float m_pitch = 0.0f;

	float m_aspectRatio;
	float m_fov; // in degrees

	void UpdateProjection();
	void UpdateOrientation();
};