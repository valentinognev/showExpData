from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point, Quaternion
import rospy
import numpy as np


class arw_mrkr_class:
    def __init__(self, rgba=[1.0, 0.0, 0.0, 1.0], frame_id = 'mr18_frame'):

        self.arroy_mrkr = Marker()
        self.arroy_mrkr.header.seq = 0
        # arroy_mrkr.header.stamp = rospy.Time.now()
        self.arroy_mrkr.header.frame_id = frame_id
        self.arroy_mrkr.scale = Vector3(x=2.0, y=0.1, z=0.2)  # length, width, height
        self.arroy_mrkr.color = ColorRGBA(a=rgba[3], r=rgba[0], g=rgba[1], b=rgba[2])
        
        # msg_Marker_WF_cmd_vel.lifetime.secs = 0 # 0 means forever 
        # msg_Marker_WF_cmd_vel.lifetime.nsecs = 0 # 0 means forever 
        self.arroy_mrkr.type = Marker.ARROW
        self.arroy_mrkr.action = Marker.MODIFY

    def update_arrow_mrkr(self, x_pos, y_pos, stamp):

        yaw = np.arctan2(y_pos, x_pos)
        self.arroy_mrkr.header.seq += 1
        self.arroy_mrkr.header.stamp = stamp
        # self.arroy_mrkr.pose.position = Point(0.0,0.0,0.0)
        self.arroy_mrkr.pose.orientation =  Quaternion(0, 0, np.sin(yaw/2.0), np.cos(yaw/2.0))        





'''

tick
gyro.x
gyro.y
gyro.z
acc.x
acc.y
acc.z
baro.asl
baro.temp
baro.pressure
kalman.broAslAvg
stateEstimate.x
stateEstimate.y
stateEstimate.z
stateEstimate.vx
stateEstimate.vy
stateEstimate.vz
stateEstimate.yaw
stateEstimate.pitch
stateEstimate.roll
motor.m1
motor.m2
motor.m3
motor.m4
pm.vbat
range.zrange
motion.deltaX
motion.deltaY
motion.shutter
motion.squal
motion.max_OL
motion.cOL_cnt
motion.OF_valid
motion.numIL_16
kalman_pred.tof_cnt
kalman_pred.tof_innov
kalman_pred.tof_mxRj
kalman_pred.tof_stmp
kalman_pred.predNX
kalman_pr_s.predNX
kalman_pred.predNY
kalman_pr_s.predNY
kalman_pred.measNX
kalman_pred.measNY
kalman_pred.flow_dt
kalman_pred.flow_upd
kalman_pr_s.flow_upd
kalman_pred.var_vx
kalman_pred.var_vy
kalman_pred.res_flwX
kalman.SensorConfig
ransac.cpsi
ransac.spsi
ransac.rho
ransac.valid
ransac.slowDwn
ransac.Vx_LLAVD
ransac.Vy_LLAVD
ransac.rejCmdVx
ransac.rejCmdVy
LLAVD.pitch_LLAVD
LLAVD.roll_LLAVD
LLAVD.pitch_MLAVD
LLAVD.roll_MLAVD
FF_Angle.pitch_HL
FF_Angle.roll_HL
FF_Angle.pitch_cmd
FF_Angle.roll_cmd
FF_Angle.ptch_HL_flt
FF_Angle.roll_HL_flt
FF_Angle.var_avg
FF_Angle.HL_ratio
FF_Angle.HL_r_fd
FF_Angle.HL_r_lp
mr18.m0
mr18.m1
mr18.m2
mr18.m3
mr18.m4
mr18.m5
mr18.m6
mr18.m7
mr18.m8
mr18.m9
mr18.m10
mr18.m11
mr18.m12
mr18.m13
mr18.m14
mr18.m15
mr18.m16
mr18.m17
state_machine.vnom_sign
state_machine.P1_St_log
state_machine.SM_St_log
state_machine.vx_sp
state_machine.vy_sp
state_machine.TstSlwDwn
state_machine.ST_St_log
state_machine.ext_gf_brch
state_machine.ext_gf_cntr
state_machine.int_gf_brch
state_machine.int_gf_cntr
feat2cmd.Vxcmd
feat2cmd.Vycmd
feat2cmd.Vxnom
feat2cmd.Vynom
feat2cmd.stopped
feat2cmd.SlwDwnFct
controller.rstInt
kalman.rstEst
kalman.rstEst_f
kalman.varPX_f
kalman.varPY_f
kalman.stateX_f
kalman.stateY_f
kalman.stateZ_f
kalman.stateYaw_f
kalman.varPX
kalman.varPY
kalman.stateX
kalman.stateY
kalman.stateZ
kalman.statePX_f
kalman.statePY_f
kalman.statePX
kalman.statePY
kalman.statePZ
kalman.eFlwStd
kalman.mSigFlwX
kalman.eFlwStd_
kalman.mSigFlwX_
controller.pitchRate
controller.rollRate
controller.yawRate
controller.pitch
controller.roll
controller.yaw
ctrltarget.vx
ctrltarget.vy
ctrltarget.vz
posCtl.targetZ
assertLogs.assrt
optflow_data.deltaX
optflow_data.deltaY
optflow_data.frameID
optflow_data.deltaRoll
optflow_data.deltaPitch
optflow_data.dt
optflow_data.stdDevX
optflow_data.stdDevY
optflow_data.FOM
optflow_data.squal
optflow_data.numIL_16
optflow_data.skpFrm
optflow_data.readFail
optflow_data.HLr_open
optflow_data.arrIL
dvio_data.deltaX
dvio_data.deltaY
dvio_data.deltaZ
dvio_data.frameID
dvio_data.deltaRoll
dvio_data.deltaPitch
dvio_data.deltaYaw
dvio_data.dt
dvio_data.stdDevX
dvio_data.stdDevY
dvio_data.stdDevZ
dvio_data.FOM
dvio_data.squal
dvio_data.velDataVx
dvio_data.velDataVy
dvio_data.velDataVz
dvio_data.velStdDevx
dvio_data.velStdDevy
dvio_data.velStdDevz
dvio_data.stdFacX
dvio_data.stdFacY
dvio_data.stdFacZ
kalman_pr_s.measNX
kalman_pr_s.measNY
kalman_pr_s.dRoll_b
kalman_pr_s.dPitch_b
kalman_pr_s.flow_dt
kalman_pred.dRoll_b
kalman_pred.dPitch_b
kalman_pr_s.r22
kalman_pred.r22
kalman_states.vx
kalman_states.vy
kalman_states.vz
kalman_states.vx_2
kalman_states.vy_2
kalman_states.vz_2
kalman_states.useVel


'''