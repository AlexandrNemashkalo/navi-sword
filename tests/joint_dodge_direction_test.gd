extends SceneTree

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	lab.WalkInput = Vector2.ZERO
	var starts = [Quaternion.IDENTITY,
		Quaternion(Vector3.UP, deg_to_rad(179)),
		Quaternion(Vector3.RIGHT, deg_to_rad(85)),
		Quaternion(Vector3.RIGHT, deg_to_rad(110)),
		Quaternion(Vector3.FORWARD, deg_to_rad(170)) * Quaternion(Vector3.UP, deg_to_rad(120))]
	var axes = [Vector3.RIGHT, Vector3.UP, Vector3.BACK]
	var expected = [Vector3(-18, 0, 0), Vector3(0, -15, 0), Vector3(0, 0, -18)]
	var worst = 0.0
	var return_error = 0.0
	for start in starts:
		for axis in 3:
			lab.SetDodge(0, 0, 0, 0, 0)
			for i in 90:
				lab._Process(1.0 / 60)
			# No R1 needed: direction is independent of absolute orientation.
			lab.DrivePadAttitude(start, Vector2.ZERO, 0.0, false, false, 1.0, 1.0 / 60)
			for i in 30:
				var attitude = start * Quaternion(axes[axis], deg_to_rad((i + 1) * 0.5))
				# q and -q represent the same orientation; switching signs must not move the body.
				if i % 2 == 0:
					attitude = Quaternion(-attitude.x, -attitude.y, -attitude.z, -attitude.w)
				lab.DrivePadAttitude(attitude, Vector2.ZERO, 0.0, false, false, 1.0, 1.0 / 60)
				lab._Process(1.0 / 60)
			for i in 90:
				lab._Process(1.0 / 60)
			worst = max(worst, lab.DodgeAngles.distance_to(expected[axis]))
			for i in 30:
				var attitude = start * Quaternion(axes[axis], deg_to_rad((29 - i) * 0.5))
				lab.DrivePadAttitude(attitude, Vector2.ZERO, 0.0, false, false, 1.0, 1.0 / 60)
				lab._Process(1.0 / 60)
			for i in 90:
				lab._Process(1.0 / 60)
			return_error = max(return_error, lab.DodgeAngles.length())
			lab.DrivePadAttitude(start, Vector2.ZERO, 0.0, false, false, 0.0, 1.0 / 60)
	print("Local dodge directions across heading seam, vertical/inverted grips: error %.4f; inverse gesture %.4f" % [worst, return_error])
	if worst < 0.02 and return_error < 0.02:
		print("PASS stable dodge direction without R1")
		quit()
	else:
		push_error("Dodge direction flipped or mixed axes")
		quit(1)
