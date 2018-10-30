#include "state_space_control.h"

StateSpaceControl::StateSpaceControl(void)
{

}

StateSpaceControl::~StateSpaceControl(void)
{

}

///////////////////////////////////////////////////////////////
// StateSpaceControl Public API
///////////////////////////////////////////////////////////////
int StateSpaceControl::checkRestrictions(struct PoseCtrl st_pose, struct PoseCtrl st_goal)
{
  struct PoseCtrl st_transform;
  struct PoseCtrl st_pose_in_goal_frame;
  int output = OK;

  //translation to goal frame
  st_transform.x = -1 * st_goal.x;
  st_transform.y = -1 * st_goal.y;
  st_transform.z = -1 * st_goal.z;
  st_transform.yaw = 0.0;
  st_transform.pitch = 0.0;
  st_transform.roll = 0.0;
  this->transformPoint(st_transform, st_pose, st_pose_in_goal_frame);

  //rotation to goal frame
  st_transform.x = 0.0;
  st_transform.y = 0.0;
  st_transform.z = 0.0;
  st_transform.yaw = st_goal.yaw;
  st_transform.pitch = st_goal.pitch;
  st_transform.roll = st_goal.roll;
  this->transformPoint(st_transform, st_pose_in_goal_frame, st_pose_in_goal_frame);

  //check restrictions
  output = this->restrictionRules(st_pose_in_goal_frame);

  return output;
}

int StateSpaceControl::transformPoint(struct PoseCtrl st_transform, struct PoseCtrl st_original_point,
                                      struct PoseCtrl &st_transformed_point)
{
  float transform_matrix[3][3];
  float temp, temporal_point_o[3], temporal_point_f[3];
  int i, j;

  //form the transformation matrix
  transform_matrix[0][0] = cos(st_transform.yaw) * cos(st_transform.pitch);
  transform_matrix[0][1] = sin(st_transform.yaw) * cos(st_transform.pitch);
  transform_matrix[0][2] = -sin(st_transform.pitch);
  transform_matrix[1][0] = cos(st_transform.yaw) * sin(st_transform.pitch) * sin(st_transform.roll)
      - sin(st_transform.yaw) * cos(st_transform.roll);
  transform_matrix[1][1] = sin(st_transform.yaw) * sin(st_transform.pitch) * sin(st_transform.roll)
      + cos(st_transform.yaw) * cos(st_transform.roll);
  transform_matrix[1][2] = cos(st_transform.pitch) * sin(st_transform.roll);
  transform_matrix[2][0] = cos(st_transform.yaw) * sin(st_transform.pitch) * cos(st_transform.roll)
      + sin(st_transform.yaw) * sin(st_transform.roll);
  transform_matrix[2][1] = sin(st_transform.yaw) * sin(st_transform.pitch) * cos(st_transform.roll)
      - cos(st_transform.yaw) * sin(st_transform.roll);
  transform_matrix[2][2] = cos(st_transform.pitch) * cos(st_transform.roll);

  //matrix multiplication
  temporal_point_o[0] = st_original_point.x;
  temporal_point_o[1] = st_original_point.y;
  temporal_point_o[2] = st_original_point.z;
  temp = 0;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      temp = temp + transform_matrix[i][j] * temporal_point_o[j];
    }
    temporal_point_f[i] = temp;
    temp = 0;
  }

  //translation and orientation of point
  st_transformed_point.x = temporal_point_f[0] + st_transform.x;
  st_transformed_point.y = temporal_point_f[1] + st_transform.y;
  st_transformed_point.z = temporal_point_f[2] + st_transform.z;
  st_transformed_point.yaw = st_original_point.yaw - st_transform.yaw;
  st_transformed_point.pitch = st_original_point.pitch - st_transform.pitch;
  st_transformed_point.roll = st_original_point.roll - st_transform.roll;

  //correction of angles
  if (st_transformed_point.yaw > PI)
  {
    st_transformed_point.yaw = st_transformed_point.yaw - 2 * PI;
  }
  else if (st_transformed_point.yaw < -PI)
  {
    st_transformed_point.yaw = st_transformed_point.yaw + 2 * PI;
  }

  if (st_transformed_point.pitch > PI)
  {
    st_transformed_point.pitch = st_transformed_point.pitch - 2 * PI;
  }
  else if (st_transformed_point.pitch < -PI)
  {
    st_transformed_point.pitch = st_transformed_point.pitch + 2 * PI;
  }

  if (st_transformed_point.roll > PI)
  {
    st_transformed_point.roll = st_transformed_point.roll - 2 * PI;
  }
  else if (st_transformed_point.roll < -PI)
  {
    st_transformed_point.roll = st_transformed_point.roll + 2 * PI;
  }

  return 0;
}

int StateSpaceControl::restrictionRules(struct PoseCtrl st_pose_in_goal_frame)
{
  int output = OK;

  if (st_pose_in_goal_frame.x >= 0.0)
  {
    output = CROSSED_GOAL;
  }
  if (st_pose_in_goal_frame.yaw >= PI / 2 || st_pose_in_goal_frame.yaw <= -PI / 2)
  {
    output = BAD_ORIENTATION;
  }
  return output;
}
