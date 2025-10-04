
When uploading to a new board,
connect to the terminal and delete the values in Flash

 save -d
 
 then  load the program again.
 
 
 alternatively ion cube Programmer,  erase flash before  programming.
 
 Observations:
 
 Need to set number of pole pairs - not sure where in code but
 in command ->   set par_pp   xxx
 
 also sees an undervoltage error after several attempts when starting from uart_req=1
 then going lower until the motor does not spin.
 
 Questions:
 \how do you clear errors in terminal?
 
 settings for multistart - 11 pole pairs
 
 settings that work:
 adc1| 0| 0| 4096| Raw ADC throttle
adc1_max| 2700| 0| 4096| ADC1 max val
adc1_min| 1200| 0| 4096| ADC1 min val
adc1_pol| 1.000000| -1.00| 1.00| ADC1 polarity
adc2_max| 4095| 0| 4096| ADC2 max val
adc2_min| 1200| 0| 4096| ADC2 min val
adc2_pol| 1.000000| -1.00| 1.00| ADC2 polarity
can_adc| 0| 0| 254| CAN ADC ID  0=disabled
ehz| 0.035364| -inf| inf| Motor electrical hz
error| 0| 0| 4294967295| System errors now
error_all| 0| 0| 4294967295| All errors encountered
FOC_angle| 40958| 0| 65535| FOC angle now
FOC_curr_BW| 3000.000000| 200.00| 10000.00| Current Controller Bandwidth
FOC_enc_ang| 0| 0| 65535| Encoder angle now
FOC_enc_oset| 25000| 0| 65535| Encoder alignment angle
FOC_enc_pol| 0| 0| 1| Encoder polarity
FOC_enc_PPR| 4096| 0| 65535| Encoder ABI PPR
FOC_flux_gain| 0.402492| 0.00| 100.00| Flux linkage gain
FOC_flux_gain| 0.402492| 0.00| 100.00| Flux linkage gain
FOC_flux_nlin| 5000.000000| 0.00| 10000.00| Flux centering gain
FOC_fpwm| 20000.000000| 0.00| 100000.00| PWM frequency
FOC_fw_ehz| 0.000000| 0.00| 6000.00| max eHz under field weakenning
FOC_hall_array_ok| 0| 0| 1| Hall array OK flag (set to 0 to restart live hall cal process)
FOC_hall_iir| 0.950000| 0.00| 1.00| Decay constant for hall preload (0-1.0)
FOC_hall_Vt| 2.000000| 0.00| 100.00| Hall transition voltage
FOC_hfi_eHz| 0.000000| 0.00| 2000.00| HFI Max Frequency
FOC_hfi_type| 0| 0| 3| HFI type [0=None, 1=45deg, 2=d axis]
FOC_hfi_volt| 1.000000| 0.00| 50.00| HFI voltage
FOC_Max_Mod| 0.950000| 0.10| 1.12| Max modulation index; typically 0.95, can over modulate to 1.12
FOC_obs_type| 1| 0| 3| Observer type, 0=None, 1=MXLEMMINGLambda, 2MXLEMMING, 3=OrtegaOrig
FOC_ol_step| 0| 0| 6000| Angle per PWM period openloop (65535 per erev)
FOC_ortega_gain| 1000000.000000| 1.00| 100000000.00| Ortega gain, typically 1M
Hall_flux| Array[12]| -10.00| 10.00| hall start table
id| 0.047168| -inf| inf| Phase Idq_d smoothed
input_opt| 8| 0| 128| Inputs [1=ADC1 2=ADC2 4=PPM 8=UART 16=Killswitch 32=CANADC1 64=CANADC2 128=ADC12DIFF]
iq| 0.008448| -inf| inf| Phase Idq_q smoothed
iqreq| 0.000000| -4096.00| 4096.00| mtr[0].FOC.Idq_req.q
meas_cl_curr| 8.500000| 0.50| 100.00| Measuring q closed loop current
meas_curr| 20.000000| 0.50| 100.00| Measuring current
meas_volt| 4.000000| 0.50| 100.00| Measuring voltage
node_id| 0| 1| 254| Node ID
opt_app_type| 0| 0| 3| App type, 0=none, 1=Vehicle, 2,3 = undefined
opt_circ_lim| 2| 0| 2| Circle limiter [0=OFF, 1=ON, 2=ON Vd]
opt_cont_type| 0| 0| 4| Cont type: 0=Torque, 1=Speed, 2=Duty, 3=Position, 4=Measuring, 5=Handbrake
opt_fw| 2| 0| 2| Field weakening [0=OFF, 1=ON, 2=ON V2]
opt_hall_start| false| false| true| Use hall start
opt_lr_obs| false| false| true| Use LR observer
opt_motor_temp| false| false| true| Motor has temperature sensor
opt_mtpa| 0| 0| 3| MTPA type = 0=none, 1=setpoint, 2=magnitude, 3=iq
opt_phase_bal| false| false| true| Use highhopes phase balancing
opt_pwm_type| 0| 0| 3| Modulator [0=SVPWM, 1=sinusoidal, 2=Bottom clamp, 3=Sin/bottom combo]
par_dir| 0| 0| 1| Motor direction
par_flux| 0.001620| 0.00| 100.00| Flux linkage
par_fw_curr| 0.000000| 0.00| 300.00| Max field weakenning current
par_i_max| 40.000000| 0.00| 1000.00| Max motor current
par_i_min| -10.000000| -1000.00| 0.00| Min motor current
par_i_park| 0.000000| 0.00| 300.00| Max current for handbrake
par_ibat_max| 10.000000| 0.00| 1000.00| Max battery current power
par_ld| 0.000047| 0.00| 10.00| Phase inductance
par_lq| 0.000047| 0.00| 10.00| Phase inductance
par_motor_sensor| 0| 0| 30| 0=SL, 1=Hall, 2=OL, 3=ABSENC, 4=INC_ENC, 5=HFI
par_p_max| 12000.000000| 0.00| 50000.00| Max power
par_pp| 11| 0| 255| Motor pole pairs
par_r| 0.172000| 0.00| 10.00| Phase resistance
par_rpm_max| 0| 0| 300000| Max RPM
par_SL_sensor| 0| 0| 30| 0=OL, 1=Hall, 2=PWMENC, 3=HFI
password| | Password for SU
safe_count| 100| 0| 1000| Live count before allowing throttle
safe_start| 100| 0| 1000| Countdown before allowing throttle
TMOS| 228.186386| 0.00| 4096.00| MOSFET temp, kelvin
TMOT| 179.512985| 0.00| 4096.00| Motor temp, kelvin
uart_dreq| 0.000000| -1000.00| 1000.00| Uart input
uart_req| 0.000000| -1000.00| 1000.00| Uart input
vbus| 16.508566| 0.00| inf| Read input voltage
Vd| 2.755023| -4096.00| 4096.00| FOC_Vdq_d
Vq| 0.925095| -4096.00| 4096.00| FOC_Vdq_q



Multistart  320kv

11 pole pairs
resistance 0.172 ohms
.047uh - using inductance meter - need to check if this is per phase or phase to phase
 should be divided by 2 - for phase inductance
 
 correct should be 0.025 uh /phase
flux - 0.00162 mWb


works(spins) with  original L value ---

 