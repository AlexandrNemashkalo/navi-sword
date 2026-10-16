extends SceneTree
# In fine work (L1) the gyro moves the forearm, and the shoulder follows a little — a quarter of
# what it would out of fine work — rather than standing frozen. Entering L1 the arm comes round to
# the thrust's guard (the point forward) gradually, no faster than the arm's live speed (7.5° a frame);
# leaving it restores the full base guard. Run: godot --headless --path . --script res://tests/joint_fine_shoulder_test.gd

func _initialize():
	call_deferred("run")

func settle(lab, pad, fine):
	for i in 60:
		lab.DrivePad(pad, Vector2.ZERO, 0.0, fine)
		lab._Process(1.0 / 60)

func guard_error(lab):
	var guard = [80.0, 38.0, 0.0, 70.0, 0.0, 0.0, 0.0]
	var error = 0.0
	for d in 7:
		error = max(error, abs(lab.Target(0, d) - guard[d]))
	return error

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
	var jump_out = guard_error(lab)
	print("pad nose up 10°: out of fine work elevation +%.1f°; in fine work elevation +%.1f°, plane %.1f° (turned 10°), elbow +%.1f°; into the guard at most %.2f° a frame, leaving %.2f°" % [
		out_gain, rise, swing, elbow, jump, jump_out])
	# L1 stick offsets both wrist axes; letting go must restore both, smoothly.
	for i in 60:
		lab.DrivePad(Vector3(10, 10, 0), Vector2(0.7, 0.6), 0.0, true)
	var wrist_before = Vector2(lab.Target(0, 5), lab.Target(0, 6))
	var wrist_last = wrist_before
	var wrist_step = 0.0
	for i in 60:
		lab.DrivePad(Vector3(10, 10, 0), Vector2.ZERO, 0.0, false)
		var wrist = Vector2(lab.Target(0, 5), lab.Target(0, 6))
		wrist_step = max(wrist_step, (wrist - wrist_last).length())
		wrist_last = wrist
	var wrist_ok = abs(wrist_before.x) > 5 and abs(wrist_before.y) > 1 and wrist_last.length() < 0.01 and wrist_step < 4
	print("L1 wrist return: from %s to %s; largest step %.3f" % [wrist_before, wrist_last, wrist_step])
	# Repeated complete cycles must converge to the same seven joint values.
	var cycle_error = 0.0
	for cycle in 8:
		for i in 30:
			lab.DrivePad(Vector3(cycle * 2, -cycle, cycle), Vector2(0.4, -0.3), 0.0, true)
		settle(lab, Vector3(cycle * 2, -cycle, cycle), false)
		cycle_error = max(cycle_error, guard_error(lab))
	# Rapid toggling interrupts both transitions; the final release must still reset all axes.
	for cycle in 12:
		for i in 5:
			lab.DrivePad(Vector3(10, 5, 3), Vector2.ZERO, 0.0, true)
		for i in 5:
			lab.DrivePad(Vector3(10, 5, 3), Vector2.ZERO, 0.0, false)
	settle(lab, Vector3(10, 5, 3), false)
	cycle_error = max(cycle_error, guard_error(lab))
	print("Full guard error after repeated and interrupted L1 cycles: %.4f" % cycle_error)
	if abs(rise - 0.25 * out_gain) < 0.5 and abs(swing) > 3 and elbow > 10 and jump <= 7.5 and jump_out < 0.01 and wrist_ok and cycle_error < 0.01:
		print("PASS fine shoulder: it follows a quarter of the way, the forearm the rest; the guard comes smoothly")
		quit()
	else:
		push_error("In fine work the shoulder does not follow, or the mode change jumps")
		quit(1)
