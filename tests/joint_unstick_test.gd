extends SceneTree
# The elbow tucked against the side, then pulled away with the gyro, slowly and quickly. While
# pressed, the solver turns joints the player is not driving out of the way; once the arm is
# clear they must come back at once — they used to return at 40°/s, so the arm seemed to stick
# to the body and then caught up with a jump.
# Run: godot --headless --path . --script res://tests/joint_unstick_test.gd

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	var pressed = Vector3(-30, -15, -20)
	var ok = true
	for frames in [40, 10]:
		lab.CentrePad()
		for i in 120:
			lab.DrivePad(pressed, Vector2(0, 0.8), 0.0, false)
			lab._Process(1.0 / 60)
		var reached = -1
		for f in 120:
			var t = min(float(f) / frames, 1.0)
			lab.DrivePad(pressed + Vector3(30, 15, 20) * t, Vector2(0, 0.8), 0.0, false)
			lab._Process(1.0 / 60)
			if reached < 0 and f >= frames:
				var there = abs(lab.PlaneFromFront(0) - lab.ReachablePlane(0)) <= 2
				for d in range(1, 7):
					there = there and abs(lab.Angle(0, d) - lab.Target(0, d)) <= 2
				if there:
					reached = f
		print("pulled away over %d frames: every joint on target at frame %d" % [frames, reached])
		ok = ok and reached >= 0 and reached <= frames + 5
	if ok:
		print("PASS the arm leaves the body at once")
		quit()
	else:
		push_error("The arm sticks to the body after it is pulled away")
		quit(1)
