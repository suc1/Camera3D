// Copyright 2020 Deeproute.ai. All Rights Reserved.
// Author: Yuchen Liu (yuchenliu@deeproute.ai)
#include "camera/camera_3d.h"

#include <cmath>
#include <iostream>

Camera3D::Camera3D(glm::vec3 position, float heading_radian,
                   float look_down_radian)
    : position_(position),
      roll_(glm::degrees(heading_radian)),
      pitch_(glm::degrees(look_down_radian)) {
  yaw_ = 0.0f;
  UpdateViewMatrix();
  UpdateMVPMatrix(size_w_, size_h_);
}

const glm::mat4& Camera3D::GetMVPMatrix(float size_w, float size_h) {
  UpdateMVPMatrix(size_w, size_h);
  return mvp_mat_;
}

// Returns the view matrix using the LookAt Matrix
const glm::mat4& Camera3D::GetModelViewMatrix() const { return view_mat_; }

const glm::mat4& Camera3D::GetProjectionMatrix(float size_w, float size_h) {
  UpdateMVPMatrix(size_w, size_h);
  return projection_mat_;
}

//由空间一点pos，求出在图象平面的坐标(u,v)
const std::pair<float, float> Camera3D::GetUVAtPos(const glm::vec3& pos) {
  auto posUV = mvp_mat_ * glm::vec4(pos, 1.0f);
  if(posUV.w<0.0f) return {1.1f, 1.1f};

  posUV /= posUV.w;
  if(posUV.x < -1.0f || posUV.x > 1.0f || posUV.y < -1.0f || posUV.y > 1.0f) return {1.1f, 1.1f};


  return std::make_pair(posUV.x, posUV.y);
}

void Camera3D::ProcessKeyboard(CameraMovement direction, float delta_time) {
  float velocity = movement_speed_ * delta_time;

  switch(direction) {
    case CameraMovement::FORWARD:
      position_ += front_ * velocity;
      break;
    case CameraMovement::BACKWARD:
      position_ -= front_ * velocity;
      break;

    case CameraMovement::LEFT:
      position_ += right_ * velocity;
      break;
    case CameraMovement::RIGHT:
      position_ -= right_ * velocity;
      break;

    case CameraMovement::DOWN:
      position_ -= up_ * velocity;
      break;
    case CameraMovement::UP:
      position_ += up_ * velocity;
      break;

    default:
      break;
  }

  UpdateViewMatrix();
  UpdateMVPMatrix();
}

void Camera3D::ProcessMouseMovement(float x_offset, float y_offset) {
  x_offset *= sensitivity_;
  y_offset *= sensitivity_;

  roll_ += x_offset;
  roll_ = std::fmod(roll_, 360);

  pitch_ += y_offset;
  pitch_ = std::fmod(pitch_, 180);

  UpdateViewMatrix();
  UpdateMVPMatrix();
}

// Calculates the front vector from the Camera's (updated) Euler Angles
void Camera3D::UpdateViewMatrix() {
  glm::mat4 rotate_x = glm::rotate(glm::mat4(1.0f), glm::radians(pitch_), glm::vec3(1.0f, 0.0f, 0.0f));
  glm::mat4 rotate_y = glm::rotate(glm::mat4(1.0f), glm::radians(yaw_),   glm::vec3(0.0f, 1.0f, 0.0f));
  glm::mat4 rotate_z = glm::rotate(glm::mat4(1.0f), glm::radians(roll_),  glm::vec3(0.0f, 0.0f, 1.0f));

  glm::mat4 translate = glm::translate(glm::mat4(1.0f), position_);

  glm::mat4 model = translate * rotate_z * rotate_y * rotate_x;

  right_ = glm::vec3(model[0][0], model[0][1], model[0][2]);
  up_    = glm::vec3(model[1][0], model[1][1], model[1][2]);
  front_ = glm::vec3(-model[2][0], -model[2][1], -model[2][2]);

  view_mat_ = glm::inverse(model);
}

void Camera3D::UpdateProjectionMatrix(float size_w, float size_h) {
  if (size_w == size_w_ && size_h == size_h_) return;

  size_w_ = size_w;
  size_h_ = size_h;
  projection_mat_ = glm::perspective(static_cast<float>(M_PI_4), size_w_/size_h_, 0.1f, 1000.0f);
}

void Camera3D::UpdateMVPMatrix(float size_w, float size_h) {
  UpdateProjectionMatrix(size_w, size_h);
  UpdateMVPMatrix();
}

void Camera3D::UpdateMVPMatrix() { mvp_mat_ = projection_mat_ * view_mat_; }
