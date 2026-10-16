extends SceneTree
# Walking with the stick pushed up, the camera turned 90° round him: he keeps going the way he
# was. Let go and pushed up again, he goes away from the camera as it is now.
# Run: godot --headless --path . --script res://tests/joint_walk_camera_test.gd

func _initialize():
	call_deferred("run")

func away(camera):
	var a = -camera.global_transform.basis.z
	a.y = 0
	return a.normalized()

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	var camera = lab.find_children("*", "Camera3D", true, false)[0]
	for i in 60:
		lab._Process(1.0 / 60)
	var first = away(camera)
	lab.WalkInput = Vector2(0, -1)
	for i in 60:
		lab._Process(1.0 / 60)
	# R1 and the stick right for half a second: the camera goes 90° round.
	for i in 30:
		lab.CameraStickFor(Vector2(1, 0), true, false, 1.0 / 60)
		lab._Process(1.0 / 60)
	lab.CameraStickFor(Vector2.ZERO, false, false, 1.0 / 60)
	for i in 30:
		lab._Process(1.0 / 60)
	var kept = lab.WalkVelocity.normalized().dot(first)
	lab.WalkInput = Vector2.ZERO
	for i in 60:
		lab._Process(1.0 / 60)
	var second = away(camera)
	lab.WalkInput = Vector2(0, -1)
	for i in 60:
		lab._Process(1.0 / 60)
	var follows = lab.WalkVelocity.normalized().dot(second)
	print("camera turned %.0f°; walk kept its way %.3f; next walk away from the camera %.3f" % [rad_to_deg(first.angle_to(second)), kept, follows])
	if first.angle_to(second) > deg_to_rad(60) and kept > 0.99 and follows > 0.99:
		print("PASS walk camera: turning the camera does not turn the walk under way")
		quit()
	else:
		push_error("Turning the camera turns the walk")
		quit(1)
