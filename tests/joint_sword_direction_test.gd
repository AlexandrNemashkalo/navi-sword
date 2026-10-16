extends SceneTree

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	var reference = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	root.add_child(reference)
	lab.set_process(false)
	reference.set_process(false)
	var worst = 0.0
	var axes = [Vector3.RIGHT, Vector3.UP, Vector3.BACK]
	var starts = [Quaternion.IDENTITY, Quaternion(Vector3.UP, deg_to_rad(179)),
		Quaternion(Vector3.RIGHT, deg_to_rad(85)), Quaternion(Vector3.RIGHT, deg_to_rad(110)),
		Quaternion(Vector3.FORWARD, deg_to_rad(170))]
	for start in starts:
		for mode in 3:
			for axis in 3:
				lab.CentrePad()
				reference.CentrePad()
				for i in 60:
					lab.DrivePadAttitude(Quaternion.IDENTITY, Vector2.ZERO, 0.0, false, false, 0.0, 1.0 / 60)
					reference.DrivePad(Vector3.ZERO, Vector2.ZERO, 0.0, false)
				# Establish a new grip either directly, under R1, or under L2.
				if mode == 0:
					lab.CentrePad()
				else:
					for i in 30:
						lab.DrivePadAttitude(Quaternion.IDENTITY.slerp(start, float(i + 1) / 30), Vector2.ZERO, 0.0, false, mode == 1, 1.0 if mode == 2 else 0.0, 1.0 / 60)
				for i in 60:
					lab.DrivePadAttitude(start, Vector2.ZERO, 0.0, false, false, 0.0, 1.0 / 60)
				# Matching local gestures must produce exactly the normal sword controls.
				for i in 90:
					var degrees = min(i + 1, 30) * 0.5
					var attitude = start * Quaternion(axes[axis], deg_to_rad(degrees))
					var pad = Vector3.ZERO
					pad[axis] = degrees if axis == 0 else -degrees
					lab.DrivePadAttitude(attitude, Vector2.ZERO, 0.0, false, false, 0.0, 1.0 / 60)
					reference.DrivePad(pad, Vector2.ZERO, 0.0, false)
					for d in 7:
						worst = max(worst, abs(lab.Target(0, d) - reference.Target(0, d)))
	print("Sword directions: normal / R1 regrip / L2 exit, five grips, three axes: error %.5f" % worst)
	if worst < 0.02:
		print("PASS ordinary sword controls stay consistent across grips and modes")
		quit()
	else:
		push_error("Sword direction or mode rebase regression")
		quit(1)
