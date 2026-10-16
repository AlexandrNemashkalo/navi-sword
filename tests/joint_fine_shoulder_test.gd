extends SceneTree
# In fine work (L1) the gyro moves the forearm, and the shoulder follows a little — a quarter of
# what it would out of fine work — rather than standing frozen. Entering L1 the arm comes round to
# the thrust's guard (the point forward) gradually, no faster than the arm's live speed (7.5° a frame);
# leaving it moves nothing by itself. Run: godot --headless --path . --script res://tests/joint_fine_shoulder_test.gd

func _initialize():
	call_deferred("run")

func settle(lab, pad, fine):
	for i in 60:
		lab.DrivePad(pad, Vector2.ZERO, 0.0, fine)
		lab._Process(1.0 / 60)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	settle(lab, Vector3.ZERO, false)
	var out_rise = lab.Target(0, 1)
	settle(lab, Vector3(10, 0, 0), false)
	var out_gain = lab.Target(0, 1) - out_rise
	settle(lab, Vector3.ZERO, false)
	var before = [lab.Target(0, 0), lab.Target(0, 1), lab.Target(0, 3)]
	# Into the guard: the arm's largest step a frame.
	var steepest = 0.0
	var last = []
	for d in 7:
		last.append(lab.Angle(0, d))
	for i in 60:
		lab.DrivePad(Vector3.ZERO, Vector2.ZERO, 0.0, true)
		lab._Process(1.0 / 60)
		for d in 7:
			steepest = max(steepest, abs(lab.Angle(0, d) - last[d]))
			last[d] = lab.Angle(0, d)
	var entered = [lab.Target(0, 0), lab.Target(0, 1), lab.Target(0, 3)]
	settle(lab, Vector3(10, 10, 0), true)
	var rise = lab.Target(0, 1) - entered[1]
	var swing = lab.Target(0, 0) - entered[0]
	var elbow = lab.Target(0, 3) - entered[2]
	var during = [lab.Target(0, 0), lab.Target(0, 1)]
	settle(lab, Vector3(10, 10, 0), false)
	var left = [lab.Target(0, 0), lab.Target(0, 1)]
	var jump = steepest
	var jump_out = max(abs(left[0] - during[0]), abs(left[1] - during[1]))
	print("pad nose up 10°: out of fine work elevation +%.1f°; in fine work elevation +%.1f°, plane %.1f° (turned 10°), elbow +%.1f°; into the guard at most %.2f° a frame, leaving %.2f°" % [
		out_gain, rise, swing, elbow, jump, jump_out])
	if abs(rise - 0.25 * out_gain) < 0.5 and abs(swing) > 3 and elbow > 10 and jump <= 7.5 and jump_out < 0.5:
		print("PASS fine shoulder: it follows a quarter of the way, the forearm the rest; the guard comes smoothly")
		quit()
	else:
		push_error("In fine work the shoulder does not follow, or the mode change jumps")
		quit(1)
