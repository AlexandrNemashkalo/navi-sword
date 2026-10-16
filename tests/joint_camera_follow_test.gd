extends SceneTree
# The steps turn the whole stance; the camera turns with it, so the fighter keeps the same side
# to it (before, standing still, it ended up looking at his back). Swings the sword arm far back
# so the trunk turns and the feet step, and checks the angle between the line of the feet and
# the way the camera looks is the same after the steps as before: within 8° (a foot is not
# stepped while it is within 5.5 cm of its place, which leaves the feet's line up to ~10° off;
# without following, the camera would be off by the whole ~30° turn).
# Run: godot --headless --path . --script res://tests/joint_camera_follow_test.gd

func _initialize():
	call_deferred("run")

func flat_angle(v):
	return atan2(v.x, v.z)

func relative(lab, camera):
	# The way the camera looks, against the line of the feet.
	var feet = lab.FootPosition(1) - lab.FootPosition(0)
	return wrapf(rad_to_deg(flat_angle(camera.global_transform.basis.z) - flat_angle(feet)), -180, 180)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	var camera = lab.find_children("*", "Camera3D", true, false)[0]
	lab.CentrePad()
	for i in 60:
		lab.DrivePad(Vector3.ZERO, Vector2.ZERO, 0.0, false)
		lab._Process(1.0 / 60)
	var before = relative(lab, camera)
	var orbit = lab.Orbit
	var turned = 0.0
	for side in [1.0, -1.0]:
		# Out and back behind (pad turned right), or round across (left), held for 3 s.
		for i in 180:
			lab.DrivePad(Vector3(20, side * 40.0, 0), Vector2.ZERO, 0.0, false)
			lab._Process(1.0 / 60)
		var after = relative(lab, camera)
		var feet = rad_to_deg(flat_angle(lab.FootPosition(1) - lab.FootPosition(0)))
		print("pad %s: camera to the feet' line %.1f° (was %.1f°), feet turned to %.1f°" % ["out and behind" if side > 0 else "across", after, before, feet])
		turned = max(turned, abs(wrapf(after - before, -180, 180)))
	print("default orbit %.0f°, largest change of the camera's side to the fighter %.1f°" % [orbit, turned])
	if orbit == 180 and turned < 8:
		print("PASS camera follows the stance")
		quit()
	else:
		push_error("The camera does not keep its side to the fighter")
		quit(1)
