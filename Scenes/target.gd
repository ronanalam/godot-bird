extends Area3D

@export var position_range: Vector3 = Vector3.ONE

func _ready() -> void:
	global_position = Vector3(0.0, 0.0, 5.0)

func _physics_process(_delta: float) -> void:
	if has_overlapping_bodies():
		position = position_range * Vector3( randf(), randf(), randf() )
