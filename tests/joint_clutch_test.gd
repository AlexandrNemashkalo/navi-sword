extends SceneTree
# R1 holds the pose indefinitely; releasing a short tap eases to guard.
# Long-hold release rebases the pad without a jump, in normal and fine modes.
# Run: godot --headless --path . --script res://tests/joint_clutch_test.gd

func _initialize():
	call_deferred("run")

func targets(lab):
	var t = []
	for d in 7:
		t.append(lab.Target(0, d))
	return t

func biggest(a, b):
	var m = 0.0
	for d in 7:
		m = max(m, abs(a[d] - b[d]))
	return m

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	var rng = RandomNumberGenerator.new()
	rng.seed = 4
	var pad = Vector3(15, 10, 5)
	for i in 60:
		lab.DrivePadClutched(pad, Vector2.ZERO, 0.0, false, false)
		lab._Process(1.0 / 60)
	var before = targets(lab)
	# R1 held while the pad is brought round 40° and back to a new hold, with a noisy sensor.
	var moved = 0.0
	for f in 120:
		var p = pad + Vector3(-30, 25, -20) * (f / 119.0) + Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3), 0)
		lab.DrivePadClutched(p, Vector2.ZERO, 0.0, false, true)
		lab._Process(1.0 / 60)
		moved = max(moved, biggest(targets(lab), before))
	var held = pad + Vector3(-30, 25, -20)
	# Let go: the first frames must not jump.
	var jump = 0.0
	var last = targets(lab)
	for f in 30:
		lab.DrivePadClutched(held, Vector2.ZERO, 0.0, false, false)
		lab._Process(1.0 / 60)
		jump = max(jump, biggest(targets(lab), last))
		last = targets(lab)
	# And the pad moves the arm again from there.
	var from = lab.Target(0, 1)
	for f in 30:
		lab.DrivePadClutched(held + Vector3(10, 0, 0), Vector2.ZERO, 0.0, false, false)
		lab._Process(1.0 / 60)
	var raised = lab.Target(0, 1) - from
	print("R1 held: targets moved %.4f°; let go: largest step %.3f°; then nose up 10°: elevation +%.1f°" % [moved, jump, raised])
	var guard = [80.0, 38.0, 0.0, 70.0, 0.0, 0.0, 0.0]
	var smooth = 0.0
	last = targets(lab)
	for f in 6:
		lab.DrivePadClutched(Vector3(f * 3, -f, f), Vector2.ONE, 1.0, false, true)
		moved = max(moved, biggest(targets(lab), last))
	for f in 13:
		lab.DrivePadClutched(Vector3(-80, 40, 20), Vector2.ZERO, 0.0, false, false)
		smooth = max(smooth, biggest(targets(lab), last))
		last = targets(lab)
	var guard_error = biggest(last, guard)
	lab.DrivePadClutched(Vector3(-80, 40, 20), Vector2.ZERO, 0.0, false, false)
	var release_error = biggest(targets(lab), guard)
	# A new hold interrupts the return and holds the intermediate pose, also in L1.
	var partial_error = 0.0
	for fine in [false, true]:
		for f in 40:
			lab.DrivePadClutched(Vector3(-60, 30, 10), Vector2.ZERO, 0.0, fine, false)
		for f in 6:
			lab.DrivePadClutched(Vector3.ZERO, Vector2.ZERO, 0.0, fine, true)
		for f in 5:
			lab.DrivePadClutched(Vector3.ZERO, Vector2.ZERO, 0.0, fine, false)
		last = targets(lab)
		for f in 35:
			lab.DrivePadClutched(Vector3(f, f, f), Vector2.ZERO, 0.0, fine, true)
			partial_error = max(partial_error, biggest(targets(lab), last))
		last = targets(lab)
		for f in 10:
			lab.DrivePadClutched(Vector3(70, -50, 40), Vector2.ZERO, 0.0, fine, false)
			partial_error = max(partial_error, biggest(targets(lab), last))
	print("Hold: guard error %.4f; largest step %.3f; release %.4f; partial release %.4f" % [guard_error, smooth, release_error, partial_error])
	if moved == 0.0 and jump < 0.5 and raised > 12 and guard_error < 0.01 and smooth < 6 and release_error < 0.01 and partial_error < 0.01:
		print("PASS clutch: long hold freezes, short tap returns, interrupted return stays continuous")
		quit()
	else:
		push_error("The R1 clutch moves the arm, or jumps when let go")
		quit(1)
