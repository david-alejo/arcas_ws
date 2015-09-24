# This contains the state estimation of the robotic arm on board. That is, the velocity and position values of the arm angles and the battery position. The angle values are
# always given in the following order: Yaw1, Pitch1, Pitch2, Roll1 Pitch3, Roll2, Gripper. These angles are shown in the code API documentation of the arcas_msgs node and the developer manual
# of the functional layer.

Header header

ArmStateEstimation arm_state_estimation