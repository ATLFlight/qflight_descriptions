#!/usr/bin/env python
# /****************************************************************************
#  *   Copyright (c) 2018 Michael Shomin. All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions
#  * are met:
#  *
#  * 1. Redistributions of source code must retain the above copyright
#  *    notice, this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright
#  *    notice, this list of conditions and the following disclaimer in
#  *    the documentation and/or other materials provided with the
#  *    distribution.
#  * 3. Neither the name ATLFlight nor the names of its contributors may be
#  *    used to endorse or promote products derived from this software
#  *    without specific prior written permission.
#  *
#  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  * POSSIBILITY OF SUCH DAMAGE.
#  *
#  * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
#  ****************************************************************************/
import rospy
import yaml
from tf import transformations as t
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

rospy.init_node("base_link_imu_static_publisher")
tf_pub = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)

board_mnt_file = rospy.get_param("~board_mounts")
imu_is_root_tf= rospy.get_param("~imu_is_root_tf")
board= rospy.get_param("~board")
vehicle= rospy.get_param("~vehicle")

if vehicle == "none":
  rospy.loginfo("Vehicle set to 'none', not publishing base_link -> imu")
  exit(0)

board_mount_yaml = file(board_mnt_file, 'r')
mounts = yaml.load(board_mount_yaml)


board_xyz = [float(xyz) for xyz in  mounts['vehicles'][vehicle]['xyz'].split(' ')]
board_rpy = [float(rpy) for rpy in  mounts['vehicles'][vehicle]['rpy'].split(' ')]
board_quat = t.quaternion_from_euler(board_rpy[0],board_rpy[1],board_rpy[2])

imu_xyz = [float(xyz) for xyz in  mounts['boards'][board]['xyz'].split(' ')]
imu_rpy = [float(rpy) for rpy in  mounts['boards'][board]['rpy'].split(' ')]
imu_quat = t.quaternion_from_euler(imu_rpy[0],imu_rpy[1],imu_rpy[2])

baselink_board = TransformStamped()
board_imu = TransformStamped()

if(imu_is_root_tf):
  board_tf_mat = t.concatenate_matrices(t.translation_matrix(board_xyz),
                                        t.quaternion_matrix(board_quat))
  board_tf_inv = t.inverse_matrix(board_tf_mat)
  board_xyz = t.translation_from_matrix(board_tf_inv)
  board_quat = t.quaternion_from_matrix(board_tf_inv)

  baselink_board.header.frame_id = "flight_board_center"
  baselink_board.child_frame_id = "base_link"


  imu_tf_mat = t.concatenate_matrices(t.translation_matrix(imu_xyz),
                                      t.quaternion_matrix(imu_quat))
  imu_tf_inv = t.inverse_matrix(imu_tf_mat)
  imu_xyz = t.translation_from_matrix(imu_tf_inv)
  imu_quat = t.quaternion_from_matrix(imu_tf_inv)

  board_imu.header.frame_id = "imu"
  board_imu.child_frame_id = "flight_board_center"

else:
  baselink_board.header.frame_id = "base_link"
  baselink_board.child_frame_id = "flight_board_center"

  board_imu.header.frame_id = "flight_board_center"
  board_imu.child_frame_id = "imu"

baselink_board.header.stamp = rospy.Time.now()
baselink_board.transform.translation.x = board_xyz[0]
baselink_board.transform.translation.y = board_xyz[1]
baselink_board.transform.translation.z = board_xyz[2]
baselink_board.transform.rotation.x = board_quat[0]
baselink_board.transform.rotation.y = board_quat[1]
baselink_board.transform.rotation.z = board_quat[2]
baselink_board.transform.rotation.w = board_quat[3]

board_imu.header.stamp = rospy.Time.now()
board_imu.transform.translation.x = imu_xyz[0]
board_imu.transform.translation.y = imu_xyz[1]
board_imu.transform.translation.z = imu_xyz[2]
board_imu.transform.rotation.x = imu_quat[0]
board_imu.transform.rotation.y = imu_quat[1]
board_imu.transform.rotation.z = imu_quat[2]
board_imu.transform.rotation.w = imu_quat[3]


static_tfs = TFMessage()
static_tfs.transforms.append(baselink_board)
static_tfs.transforms.append(board_imu)

tf_pub.publish(static_tfs)

rospy.spin()



