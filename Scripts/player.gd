extends CharacterBody3D

# Body parts
@onready var head: Node3D = $head
@onready var body: CollisionShape3D = $body
@onready var wingL: Node3D = $body/wingL
@onready var wingR: Node3D = $body/wingR
@onready var tail: Node3D = $body/tail

# Attachments on the bird
@onready var springArm: SpringArm3D = $head/SpringArm3D
@onready var playerCam: Camera3D = $head/playerCam
@onready var springEndPos: Marker3D = $head/SpringArm3D/SpringEndPosition
@onready var label: Label3D = $head/velLabel3D
@onready var alt: SpringArm3D = $altitudeSpring

# Menu variables
var inMenu: bool = false

# Lerp Parameters
const MOUSE_SENS: float = 0.35
const CAMERA_LERP: float = 6.0
const BODY_LERP: float = 6.0

# Flight parameters
@export var C_L: float = 0.3
@export var C_D_induced: float = 0.3
@export var C_D_body: float = 0.05
@export var cl_from_aoa: Curve

# Physics variables
var lift_dir: Vector3
var F_flap_lift: Vector3
var F_glide_lift: Array = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var F_ground_lift: Vector3
var F_induced_drag: Array = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var F_body_drag: Vector3
var F_body: Vector3
var F_wingL: Vector3
var F_wingR: Vector3
var F_tail: Vector3
var F_run_force: Vector3
var F_run_drag: Vector3
var F_gravity: Vector3

# Physics constants
const RUN_FORCE: float = 30.0
const RUN_DRAG: float = 6.0
const JUMP_VEL: float = 4.5
const FLAP_FORCE: float = 350.0
const ROLL_RATE: float = 2*PI * 2.0
const PITCH_RATE: float = 2*PI * 2.0
const YAW_RATE: float = 2*PI * 2.0
const TORQUE_DAMPING_CONST: float = 2.0
const TORQUE_HOOKE_CONST: float = 5.0
const MASS: float = 0.450 # kg
const ONE_WINGED_AREA: float = (0.925/2.0) * 0.2 # m^2
const TAIL_AREA: float = 0.69420 # m^2

# Rotation quaternions
var Q_pitch: Quaternion
var Q_roll: Quaternion
var Q_yaw: Quaternion
var Q_head_pitch: Quaternion
var Q_head_roll: Quaternion

# Rotational mechanics
var omega: Vector3
var torque: Vector3
var torque_drag: Vector3
var torque_hooke: Vector3
var torque_PID: Vector3
var I: Basis = Basis.IDENTITY


func _ready() -> void:
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	head.position = Vector3(0.0, 0.05, -0.16)


func _unhandled_input(event: InputEvent) -> void:
	### Rotate camera w/ mouse
	if event is InputEventMouseMotion and !inMenu:
		head.rotation.y -= event.relative.x * MOUSE_SENS/180.0
		head.rotation.y = wrapf(head.rotation.y, 0.0, 2*PI)
		head.rotation.x -= event.relative.y * MOUSE_SENS/180.0
		head.rotation.x = clamp(head.rotation.x, -PI/2, PI/4)
	
	### Scroll to zoom camera
	if event.is_action_pressed('scroll_up'):
		springArm.spring_length += 1
	if event.is_action_pressed('scroll_down'):
		springArm.spring_length -= 1
	
	### Handle mouse capture with ESC
	if event.is_action_pressed('ui_cancel'):
		if inMenu:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
		inMenu = !inMenu


func _physics_process(dt: float) -> void:
	### Set gravity
	F_gravity = Vector3.ZERO#get_gravity()
	reset_flight_values(false)
	
	### Grab player input
	var input2D:Vector2 = Input.get_vector('left', 'right', 'forward', 'back')
	var inputQE:float = Input.get_axis('yaw_left', 'yaw_right')
	var inputWS:float = Input.get_axis('back', 'forward')
	var inputAD:float = Input.get_axis('left', 'right')
	var direction:Vector3 = Vector3(input2D.x, 0, input2D.y).normalized()
	
	### Camera Interpolation
	playerCam.position = playerCam.position.lerp( springEndPos.position, dt * CAMERA_LERP )
	
	### GROUND vs. AIR movement schemes
	### ===--- GROUND ---===
	if is_on_floor():
		# Reset air stuff
		reset_flight_values(true)
		
		# Jumping off of ground
		if Input.is_action_just_pressed('flap'):
			velocity.y += JUMP_VEL
		
		# Rotate body with mouse+camera (on ground plane)
		# TODO: Process valid landings (if you land close to feet-down, your legs snap to the perched orientation)
		direction = direction.rotated(Vector3.UP, playerCam.global_rotation.y)
		if !direction.is_zero_approx():
			body.quaternion = body.quaternion.slerp( head.basis.rotated(head.basis.x, -head.rotation.x+PI/2), dt*BODY_LERP )
		
		# Process velocity on 2D ground
		F_run_force = direction * RUN_FORCE
		F_run_drag = -velocity * RUN_DRAG
		velocity += (F_run_force + F_run_drag) * dt
	### End of ===--- GROUND ---===
	
	### ===--- AIR ---===
	else:
		### Wing commands
		if Input.is_action_pressed('flap'): # Wings tucked
			C_L = 0.0
			C_D_induced = 0.01
			I.y.y = 1.0
		else: # Wings outspread
			C_L = 0.4
			C_D_induced = 0.05
			I.y.y = 1.0
		
		### Flapping; attenuated if close to ground
		var attenuation: float = sqrt(alt.get_hit_length()/alt.spring_length)
		if Input.is_action_just_released('flap'):
			F_flap_lift = attenuation * (FLAP_FORCE * -body.basis.z)
		
		### Calculate lift & drag
		var angle_of_attack: Array = [rad_to_deg(velocity.angle_to(-wingL.basis.y)), rad_to_deg(velocity.angle_to(-wingR.basis.y)) ]
		var rho: float = 1.225 # kg m^-3 #TODO: Altitude-dependent density
		
		#F_glide_lift[0] = 0.5 * cl_from_aoa.sample(angle_of_attack[0]) * ONE_WINGED_AREA * rho * velocity.dot(wingL.basis.y)**2 * -wingL.basis.z
		#F_glide_lift[1] = 0.5 * cl_from_aoa.sample(angle_of_attack[1]) * ONE_WINGED_AREA * rho * velocity.dot(wingR.basis.y)**2 * -wingR.basis.z
		F_glide_lift[0] = 0.5 * 0.1*cos(deg_to_rad(angle_of_attack[0])) * ONE_WINGED_AREA * rho * velocity.dot(wingL.basis.y)**2 * -wingL.basis.z
		F_glide_lift[1] = 0.5 * 0.1*cos(deg_to_rad(angle_of_attack[1])) * ONE_WINGED_AREA * rho * velocity.dot(wingR.basis.y)**2 * -wingR.basis.z
		#F_ground_lift = 1/(attenuation - 0.01) * lift_dir
		F_induced_drag[0] = 0.5 * 0.1*sin(angle_of_attack[0]) * ONE_WINGED_AREA * rho * velocity.dot(wingL.basis.y)**2 * wingL.basis.y
		F_induced_drag[1] = 0.5 * 0.1*sin(angle_of_attack[1]) * ONE_WINGED_AREA * rho * velocity.dot(wingR.basis.y)**2 * wingR.basis.y
		F_body_drag = C_D_body * -velocity
		
		#F_glide_lift[0] =  
		
		# Collect forces acting on each body part
		F_body = F_gravity
		F_wingL = F_flap_lift/2# + F_glide_lift[0]# + F_induced_drag[0]
		F_wingR = -F_flap_lift/2# + F_glide_lift[1]# + F_induced_drag[1]
		#F_tail = F_glide_lift[2] + F_induced_drag[2]
		
		### Angular forces
		torque = Vector3( inputWS*PITCH_RATE, inputAD*ROLL_RATE,  inputQE*YAW_RATE ) + reaction_CoM([wingL.position, wingR.position, tail.position], [F_wingL, F_wingR, F_tail])[1]
		torque_drag = -omega * TORQUE_DAMPING_CONST
		#torque_hooke = TORQUE_HOOKE_CONST * -theta_PRY
		
		#EXPERIMENTING W/ PID
		#var last_y = body.quaternion
		#torque_PID = PID()
		### End Angular forces
		
		### Process velocity & angular velocity (omega), in the air
		## The velocity equals integral( accel * dt ), and omega equals integral( alpha * dt )
		velocity += (1/MASS) * reaction_CoM([Vector3.ZERO, wingL.position, wingR.position, tail.position], [F_body, F_wingL, F_wingR, F_tail])[0] * dt
		#velocity += (1/MASS) * (F_gravity + F_flap_lift + F_ground_lift + F_glide_lift + F_body_drag + F_induced_drag) * dt
		omega += I.inverse() * (torque+torque_drag) * dt
		#omega += I.inverse() * (torque + torque_drag + torque_hooke + torque_PID) * dt
		
		#TODO: Fix !v.is_finite() bug
		if !velocity.is_finite():
			velocity = Vector3.ZERO
		
		Q_pitch = Quaternion(-body.basis.x, omega.x * dt)
		Q_roll = Quaternion(-body.basis.y, omega.y * dt)
		Q_yaw = Quaternion(body.basis.z, omega.z * dt)
		body.quaternion = (Q_pitch * Q_roll * Q_yaw * body.quaternion).normalized()
	### End of ===--- AIR ---===
	
	# Update body parts
	## wingL reaction
	#wingL.position = wingL.position
	#wingL.quaternion = body.quaternion
	## wingR reaction
	#wingR.position = wingR.position
	#wingR.quaternion = body.quaternion
	## tail reaction
	#tail.position = tail.position
	#tail.quaternion = body.quaternion
	
	# Process collision & linear forces
	move_and_slide()
	### ======----- END OF MAIN _physics_process() CODE -----===== ###
	
	
	### ===========---- DEBUG -------=============
	# Write label text
	label.text = str('vx: ') + String.num(velocity.x,3) + str(' vy: ') + String.num(velocity.y,3) + str(' vz: ') + String.num(velocity.z,3) + str('\nv: ') + String.num(velocity.length(), 4)
	label.font_size = 36
	label.pixel_size = 0.001
	
	# Debug arrows!!
	#DebugDraw3D.draw_arrow_ray(body.global_position, velocity, velocity.length(), Color.RED, 0.1)
	#DebugDraw3D.draw_arrow_ray(body.global_position, F_flap_lift, F_flap_lift.length(), Color.BLACK, 0.01)
	#DebugDraw3D.draw_arrow_ray(body.global_position, F_glide_lift, F_glide_lift.length()/100, Color.DARK_BLUE, 0.01, true)
	#DebugDraw3D.draw_arrow_ray(body.global_position, F_induced_drag, F_induced_drag.length()/10, Color.DARK_RED, 0.01)
	#DebugDraw3D.draw_arrow_ray(body.global_position, alt.basis.z, alt.get_hit_length(), Color.AQUA, 0.1)
	DebugDraw3D.draw_arrow_ray(body.global_position, F_body, F_body.length()/10, Color.WHITE, 0.1)
	DebugDraw3D.draw_arrow_ray(body.global_position, torque, torque.length(), Color.PINK, 0.1)
	DebugDraw3D.draw_arrow(body.position, body.basis*wingL.position, Color.DEEP_PINK, 0.1)
	DebugDraw3D.draw_arrow(body.position, body.basis*wingR.position, Color.DEEP_PINK, 0.1)
	DebugDraw3D.draw_arrow(body.position, body.basis*tail.position, Color.DEEP_PINK, 0.1)
	
	# Draw bases
	#drawBasis(body)
	#drawBasis(wingL.get_parent())
	#drawBasis(wingR)
	#drawBasis(tail)
	
	# Draw wing forces
	DebugDraw3D.draw_arrow_ray(wingL.global_position, F_wingL, F_wingL.length(), Color.BLACK, 0.01)
	DebugDraw3D.draw_arrow_ray(wingR.global_position, F_wingR, F_wingR.length(), Color.BLACK, 0.01)
	DebugDraw3D.draw_arrow_ray(tail.global_position, F_tail, F_tail.length(), Color.BLACK, 0.01)
	
	# Draw angular terms
	#DebugDraw3D.draw_arrow_ray(body.global_position, Q_pitch.get_axis(), 50*Q_pitch.get_angle(), Color.PINK, 0.05 )
	#DebugDraw3D.draw_arrow_ray(body.global_position, Q_roll.get_axis(), 50*Q_roll.get_angle(), Color.PINK, 0.05 )
	#DebugDraw3D.draw_arrow_ray(body.global_position, Q_yaw.get_axis(), 50*Q_yaw.get_angle(), Color.PINK, 0.05 )
### ======----- END OF _physics_process() -----===== ###


func PID(y: float, last_y:Vector3, setpoint: float, PID_vec: Vector3, dt: float) -> Vector3:
	var err: float = y - setpoint
	# Return an acceleration
	return Vector3( (PID_vec.x * err*Vector3.ONE) + (PID_vec.z * (last_y-Vector3.ONE*y)/dt) )


func reaction_CoM(force_origins: Array, forces: Array) -> Array:
	# Takes Vec3 forces, applied anywhere on the body of the bird, 
	# and converts them into one Array = [force, torque] at the center-of-mass.
	var _force := Vector3.ZERO
	var _torque := Vector3.ZERO
	
	for i in range(len(forces)):
		_force += forces[i]
		_torque += (force_origins[i]).cross(forces[i])
	
	return [_force, body.basis*_torque]


func drawBasis(node: Node3D) -> void:
	DebugDraw3D.draw_arrow_ray(node.global_position, node.basis.x, 0.5, Color.RED, 0.01)
	DebugDraw3D.draw_arrow_ray(node.global_position, node.basis.y, 0.5, Color.GREEN, 0.01)
	DebugDraw3D.draw_arrow_ray(node.global_position, node.basis.z, 0.5, Color.BLUE, 0.01)


func reset_flight_values(and_rotations: bool) -> void:
	if and_rotations:
		torque = Vector3.ZERO
		omega = Vector3.ZERO
		Q_pitch = Quaternion.IDENTITY
		Q_roll = Quaternion.IDENTITY
		Q_yaw = Quaternion.IDENTITY
	
	F_body = Vector3.ZERO
	F_wingL = Vector3.ZERO
	F_wingR = Vector3.ZERO
	F_tail = Vector3.ZERO
	F_flap_lift = Vector3.ZERO
	F_glide_lift = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
	F_ground_lift = Vector3.ZERO
	F_induced_drag = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
	F_body_drag = Vector3.ZERO

	F_run_force = Vector3.ZERO
	F_run_drag = Vector3.ZERO
