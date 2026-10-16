extends SceneTree
# Plays the gyro: raises and lowers the shoulder with the pad's nose, slowly and in a quick
# flick, with a little sensor noise, and checks that the arm keeps up without freezing and
# that frames stay short. Run: godot --headless --path . --script res://tests/joint_gyro_shoulder_test.gd

func _initialize():
	call_deferred("run")

func play(lab, label, frames, nose):
	var rng = RandomNumberGenerator.new()
	rng.seed = 5
	var lag = 0.0
	var frozen = 0
	var moving = 0
	var times = []
	var previous = lab.Angle(0, 1)
	for f in frames:
		var pitch = nose.call(f) + rng.randf_range(-0.3, 0.3)
		lab.DrivePad(Vector3(pitch, 0, 0), Vector2.ZERO, 0.0, false)
		var wanted = lab.Target(0, 1)
		var start = Time.get_ticks_usec()
		lab._Process(1.0 / 60)
		times.append((Time.get_ticks_usec() - start) / 1000.0)
		var now = lab.Angle(0, 1)
		if abs(wanted - previous) > 0.5:
			moving += 1
			if abs(now - previous) < 0.01:
				frozen += 1
		previous = now
		lag = max(lag, abs(now - wanted))
	times.sort()
	print("%s: worst lag %.1f°, frozen %d of %d moving frames, frame ms median %.2f p95 %.2f max %.2f" % [
		label, lag, frozen, moving, times[times.size() / 2], times[int(times.size() * 0.95)], times[times.size() - 1]])
	return [lag, frozen, times[int(times.size() * 0.95)]]

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	# R1, as a player presses it: the arm jumps from rest to the guard while the gyro, never
	# quite still, keeps nudging the target every frame. The arm must still get there.
	lab.CentrePad()
	var noise = RandomNumberGenerator.new()
	noise.seed = 9
	for i in 120:
		lab.DrivePad(Vector3(noise.randf_range(-0.3, 0.3), noise.randf_range(-0.3, 0.3), noise.randf_range(-0.3, 0.3)), Vector2.ZERO, 0.0, false)
		lab._Process(1.0 / 60)
	var reached = abs(lab.Angle(0, 1) - lab.Target(0, 1)) < 3 and abs(lab.Angle(0, 3) - lab.Target(0, 3)) < 3
	print("after R1 with a noisy gyro: elevation %.1f of %.1f, elbow %.1f of %.1f" % [lab.Angle(0, 1), lab.Target(0, 1), lab.Angle(0, 3), lab.Target(0, 3)])
	if not reached:
		push_error("The arm did not move to the guard while the gyro was live")
		quit(1)
		return
	# Slow: the nose up and down by 20° over four seconds (the arm 15–75°, clear of the body;
	# lower, the sword hand comes to rest against the thigh, which is meant to stop it).
	var slow = play(lab, "slow raise and lower", 240, func(f): return 20.0 * sin(f * TAU / 240))
	# Quick: a flick of the nose up 40° and back, in half a second.
	var quick = play(lab, "quick flick         ", 60, func(f): return 40.0 * sin(clamp(f / 30.0, 0, 1) * PI))
	# Frames: well inside the 16.7 ms of a 60 Hz frame (timing varies a millisecond or two between runs).
	var ok = slow[0] < 8 and slow[1] <= 5 and quick[1] <= 5 and slow[2] < 12 and quick[2] < 12
	if ok:
		print("PASS gyro shoulder: the arm follows the pad without freezing, frames short")
		quit()
	else:
		push_error("The arm lags, freezes or frames are slow")
		quit(1)
